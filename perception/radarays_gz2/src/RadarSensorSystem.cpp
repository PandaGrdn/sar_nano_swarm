#include "radarays_gz2/RadarSensorSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>

using namespace radarays_gz2;
namespace rm = rmagine;

RadarSensorSystem::RadarSensorSystem() {}
RadarSensorSystem::~RadarSensorSystem() {}

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
  map_ = rm::import_embree_map(meshPath);

  radarModel_.theta.min = -M_PI;
  radarModel_.theta.inc = (2.0 * M_PI) / 360.0;
  radarModel_.theta.size = 360;

  // Vertical FOV: +/-20 deg across 32 rows (placeholder mmWave-like geometry,
  // still not a real sensor spec -- see CLAUDE.md Tier C).
  const double vfov_deg = 40.0;
  const int vfov_rows = 32;
  radarModel_.phi.min = -(vfov_deg / 2.0) * M_PI / 180.0;
  radarModel_.phi.inc = (vfov_deg * M_PI / 180.0) / (vfov_rows - 1);
  radarModel_.phi.size = vfov_rows;

  radarModel_.range.min = 0.1;
  radarModel_.range.max = 30.0;

  sim_.setModel(radarModel_);
  sim_.setMap(map_);

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rosNode_ = std::make_shared<rclcpp::Node>("radarays_gz2_node");
  pub_ = rosNode_->create_publisher<sensor_msgs::msg::PointCloud2>("radar/points", 10);
}

void RadarSensorSystem::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
  double simTime = std::chrono::duration<double>(info.simTime).count();
  if (simTime - lastUpdateTime_ < updatePeriod_) return;
  lastUpdateTime_ = simTime;

  gz::math::Pose3d pose = gz::sim::worldPose(sensorEntity_, ecm);

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
  cloud.header.stamp = rosNode_->get_clock()->now();
  cloud.header.frame_id = "radar_link";
  cloud.height = 1;
  cloud.width = radarModel_.size();
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(radarModel_.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (uint32_t i = 0; i < radarModel_.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = result.points[i].x;
    *iter_y = result.points[i].y;
    *iter_z = result.points[i].z;
  }

  pub_->publish(cloud);
  rclcpp::spin_some(rosNode_);
}

GZ_ADD_PLUGIN(
  radarays_gz2::RadarSensorSystem,
  gz::sim::System,
  radarays_gz2::RadarSensorSystem::ISystemConfigure,
  radarays_gz2::RadarSensorSystem::ISystemPreUpdate)
