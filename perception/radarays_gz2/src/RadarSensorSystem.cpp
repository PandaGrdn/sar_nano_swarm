#include "radarays_gz2/RadarSensorSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>

#include <cstdlib>
#include <string>

using namespace radarays_gz2;
namespace rm = rmagine;

namespace
{
std::string ResolveMeshPath(const std::string &path)
{
  if (path.empty() || path.front() == '/') {
    return path;
  }

  const char *root = std::getenv("SAR_NANO_SWARM_ROOT");
  if (root == nullptr || root[0] == '\0') {
    return path;
  }

  std::string resolved = root;
  if (resolved.back() != '/') {
    resolved += '/';
  }
  return resolved + path;
}
}  // namespace

RadarSensorSystem::RadarSensorSystem() {}
RadarSensorSystem::~RadarSensorSystem() {}

void RadarSensorSystem::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  sensorEntity_ = entity;

  if (!sdf->HasElement("mesh_path")) {
    gzerr << "radarays_gz2: <mesh_path> is required in the plugin SDF block."
          << std::endl;
    return;
  }

  const std::string meshPath = ResolveMeshPath(sdf->Get<std::string>("mesh_path"));
  map_ = rm::import_embree_map(meshPath);

  radarModel_.theta.min = -M_PI;
  radarModel_.theta.inc = (2.0 * M_PI) / 360.0;
  radarModel_.theta.size = 360;
  radarModel_.phi.min = 0.0;
  radarModel_.phi.inc = 1.0;
  radarModel_.phi.size = 1;
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
