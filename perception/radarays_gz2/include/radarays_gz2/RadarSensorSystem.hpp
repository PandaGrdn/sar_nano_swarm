#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/simulation/SimulationResults.hpp>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace radarays_gz2
{

class RadarSensorSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  RadarSensorSystem();
  ~RadarSensorSystem() override;

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

private:
  void workerLoop();
  sensor_msgs::msg::PointCloud2 buildCloud(
      const gz::math::Pose3d &pose, const gz::math::Vector3d &v_sensor,
      double simTimeSec);

  gz::sim::Entity sensorEntity_;
  std::string meshPath_;
  std::string topic_{"radar/points"};
  rmagine::EmbreeMapPtr map_;
  rmagine::SphereSimulatorEmbree sim_;
  rmagine::SphericalModel radarModel_;
  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  double updatePeriod_{0.1};
  double lastUpdateTime_{0.0};
  bool havePrevPose_{false};
  gz::math::Pose3d prevPose_{};
  double prevSimTime_{0.0};

  std::mutex mtx_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  gz::math::Pose3d jobPose_{};
  gz::math::Vector3d jobVel_{};
  double jobSimTime_{0.0};
  bool haveJob_{false};
  sensor_msgs::msg::PointCloud2 pending_;
  bool havePending_{false};
  // Liveness counters — the 2026-09-11 live run had a silent /cf_*/radar/points
  // with no way to tell "PreUpdate never ran" from "worker never produced".
  std::atomic<uint64_t> nJobs_{0};
  std::atomic<uint64_t> nPub_{0};
  std::atomic<uint64_t> nScans_{0};
};

}  // namespace radarays_gz2
