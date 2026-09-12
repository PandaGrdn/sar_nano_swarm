#include "radarays_gz2/RadarSensorSystem.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>

using namespace radarays_gz2;
namespace rm = rmagine;

namespace
{
// ONE Embree map per mesh file, shared by every drone's plugin instance in this
// gz-sim process.
//
// Each instance used to import its own copy: on the 2026-09-11 live run a
// single cave_world.obj map cost ~5.3 GB RSS, so three drones needed ~16 GB on
// a 7.9 GB host — the import thrashed and never returned, and
// /cf_*/radar/points stayed silent forever. The map is immutable after commit
// and Embree ray queries are thread-safe, so one copy serves all of them.
std::mutex g_map_cache_mtx;
std::map<std::string, rm::EmbreeMapPtr> g_map_cache;

rm::EmbreeMapPtr acquire_map(const std::string &meshPath, bool &built)
{
  std::lock_guard<std::mutex> lock(g_map_cache_mtx);
  auto it = g_map_cache.find(meshPath);
  if (it != g_map_cache.end()) {
    built = false;
    return it->second;
  }
  rm::EmbreeMapPtr m = rm::import_embree_map(meshPath);
  g_map_cache[meshPath] = m;
  built = true;
  return m;
}
}  // namespace

RadarSensorSystem::RadarSensorSystem() {}

RadarSensorSystem::~RadarSensorSystem()
{
  stop_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void RadarSensorSystem::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  sensorEntity_ = entity;

  std::string meshPath = "/home/ethan/crazyflie_ws/src/darpa_subt_worlds/meshes/tunnel.dae";
  if (sdf->HasElement("mesh_path")) {
    meshPath = sdf->Get<std::string>("mesh_path");
  }
  meshPath_ = meshPath;

  // Per-drone topic (phase0_gate.sh injects <topic>/cf_<i>/radar/points</topic>
  // so each vehicle's cloud — and its Doppler — stays separable for RIO).
  if (sdf->HasElement("topic")) {
    topic_ = sdf->Get<std::string>("topic");
  }

  radarModel_.theta.min = -M_PI;
  radarModel_.theta.inc = (2.0 * M_PI) / 90.0;
  radarModel_.theta.size = 90;

  const double vfov_deg = 40.0;
  const int vfov_rows = 8;
  radarModel_.phi.min = -(vfov_deg / 2.0) * M_PI / 180.0;
  radarModel_.phi.inc = (vfov_deg * M_PI / 180.0) / (vfov_rows - 1);
  radarModel_.phi.size = vfov_rows;

  radarModel_.range.min = 0.1;
  radarModel_.range.max = 30.0;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rosNode_ = std::make_shared<rclcpp::Node>(
      "radarays_gz2_node_" + std::to_string(entity));
  pub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS());
  // Build the Embree map HERE, on the Gazebo main thread, ONCE at load.
  //
  // It must not move to the worker: Embree's BVH build (rtcCommitScene, reached
  // from import_embree_map) runs on TBB, and committing a scene from a
  // non-main thread deadlocks on this host — two libtbb runtimes are visible
  // (CMake warns that /usr/lib/x86_64-linux-gnu/libtbb.so.12 may be hidden by
  // /usr/local/lib). Symptom (2026-09-11 live run): every worker parsed the
  // whole mesh and then hung forever inside import_embree_map, so
  // /cf_*/radar/points stayed silent while PreUpdate ran normally.
  //
  // What must stay OFF this thread is the PER-STEP raycasting — that is the
  // freeze this design exists to prevent, and rtcIntersect1 is TBB-free and
  // thread-safe, so the worker below still does every scan.
  bool built = false;
  try {
    map_ = acquire_map(meshPath_, built);
    sim_.setModel(radarModel_);
    sim_.setMap(map_);
  } catch (const std::exception &e) {
    gzerr << "[radarays_gz2] map init FAILED for '" << meshPath_ << "': "
          << e.what() << " — no radar cloud will be published.\n";
    return;
  }
  if (!map_) {
    gzerr << "[radarays_gz2] import_embree_map('" << meshPath_
          << "') returned null — no radar cloud will be published.\n";
    return;
  }
  gzmsg << "[radarays_gz2] map ready (" << meshPath_ << ", "
        << (built ? "imported" : "shared") << "), topic " << topic_ << ", "
        << radarModel_.size() << " rays/scan\n";

  worker_ = std::thread(&RadarSensorSystem::workerLoop, this);
}

sensor_msgs::msg::PointCloud2 RadarSensorSystem::buildCloud(
    const gz::math::Pose3d &pose, const gz::math::Vector3d &v_sensor,
    double simTimeSec)
{
  rm::Transform T = rm::Transform::Identity();
  T.t.x = pose.Pos().X();
  T.t.y = pose.Pos().Y();
  T.t.z = pose.Pos().Z();
  T.R.x = pose.Rot().X();
  T.R.y = pose.Rot().Y();
  T.R.z = pose.Rot().Z();
  T.R.w = pose.Rot().W();

  rm::Memory<rm::Transform, rm::RAM> Tbm(1);
  Tbm[0] = T;

  using ResultT = rm::Bundle<rm::Points<rm::RAM> >;
  ResultT result = sim_.simulate<ResultT>(Tbm);

  sensor_msgs::msg::PointCloud2 cloud;
  // SIM time, not wall time: every consumer (rio_bridge dt, EKF timestamp
  // ordering, UWB edge stamps) works in sim seconds (plan §9).
  cloud.header.stamp.sec = static_cast<int32_t>(simTimeSec);
  cloud.header.stamp.nanosec = static_cast<uint32_t>(
      (simTimeSec - static_cast<double>(static_cast<int32_t>(simTimeSec))) * 1e9);
  cloud.header.frame_id = "radar_link";
  cloud.height = 1;
  cloud.width = radarModel_.size();
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "doppler", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(radarModel_.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_d(cloud, "doppler");

  for (uint32_t i = 0; i < radarModel_.size();
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_d) {
    const float px = result.points[i].x;
    const float py = result.points[i].y;
    const float pz = result.points[i].z;
    *iter_x = px;
    *iter_y = py;
    *iter_z = pz;
    const float r = std::sqrt(px * px + py * py + pz * pz);
    *iter_i = (r > 1e-3f) ? (1.0f / r) : 0.0f;
    if (r > 1e-3f) {
      const float inv_r = 1.0f / r;
      *iter_d = -(px * inv_r * static_cast<float>(v_sensor.X()) +
                  py * inv_r * static_cast<float>(v_sensor.Y()) +
                  pz * inv_r * static_cast<float>(v_sensor.Z()));
    } else {
      *iter_d = 0.0f;
    }
  }
  return cloud;
}

void RadarSensorSystem::workerLoop()
{
  // Per-step raycasting only. The map is already built (Configure); nothing
  // here touches the Gazebo physics thread, which is the point of this thread.
  while (!stop_) {
    gz::math::Pose3d pose;
    gz::math::Vector3d vel;
    double simTime = 0.0;
    bool work = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (haveJob_) {
        pose = jobPose_;
        vel = jobVel_;
        simTime = jobSimTime_;
        haveJob_ = false;
        work = true;
      }
    }
    if (!work) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    try {
      cloud = buildCloud(pose, vel, simTime);
    } catch (const std::exception &e) {
      gzerr << "[radarays_gz2] simulate() threw on " << topic_ << ": " << e.what()
            << "\n";
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(mtx_);
      pending_ = std::move(cloud);
      havePending_ = true;
    }
    if (++nScans_ == 1 || nScans_ % 200 == 0) {
      gzmsg << "[radarays_gz2] " << topic_ << " scans=" << nScans_
            << " published=" << nPub_ << " jobs=" << nJobs_ << "\n";
    }
  }
}

void RadarSensorSystem::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
  if (info.paused) {
    return;
  }
  double simTime = std::chrono::duration<double>(info.simTime).count();

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (havePending_ && pub_) {
      pub_->publish(pending_);
      havePending_ = false;
      ++nPub_;
    }
  }

  if (nJobs_ == 0) {
    gzmsg << "[radarays_gz2] PreUpdate live on " << topic_ << " (simTime "
          << simTime << ")\n";
  }

  if (simTime - lastUpdateTime_ < updatePeriod_) {
    return;
  }
  lastUpdateTime_ = simTime;

  gz::math::Pose3d pose = gz::sim::worldPose(sensorEntity_, ecm);
  gz::math::Vector3d v_sensor(0, 0, 0);
  if (havePrevPose_) {
    const double dt = simTime - prevSimTime_;
    if (dt > 1e-6) {
      const gz::math::Vector3d v_world = (pose.Pos() - prevPose_.Pos()) / dt;
      v_sensor = pose.Rot().Inverse().RotateVector(v_world);
    }
  }
  prevPose_ = pose;
  prevSimTime_ = simTime;
  havePrevPose_ = true;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    jobPose_ = pose;
    jobVel_ = v_sensor;
    jobSimTime_ = simTime;
    haveJob_ = true;
  }
  ++nJobs_;
}

GZ_ADD_PLUGIN(
  radarays_gz2::RadarSensorSystem,
  gz::sim::System,
  radarays_gz2::RadarSensorSystem::ISystemConfigure,
  radarays_gz2::RadarSensorSystem::ISystemPreUpdate)
