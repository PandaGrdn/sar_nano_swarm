#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/simulation/SimulationResults.hpp>

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
  gz::sim::Entity sensorEntity_;
  rmagine::EmbreeMapPtr map_;
  rmagine::SphereSimulatorEmbree sim_;
  rmagine::SphericalModel radarModel_;
  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  double updatePeriod_{0.1}; // 10 Hz default
  double lastUpdateTime_{0.0};
};

}  // namespace radarays_gz2
