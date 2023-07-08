#ifndef PLUGIN_DRONE_H
#define PLUGIN_DRONE_H

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>

#include "pid_controller.h"

#define LANDED_MODEL 0
#define FLYING_MODEL 1
#define TAKINGOFF_MODEL 2
#define LANDING_MODEL 3

using namespace std::placeholders;

namespace gazebo
{
class DroneSimpleController : public ModelPlugin
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  void UpdateDynamics(double dt);
  void UpdateState(double dt);
  virtual void Reset();

private:
  double m_timeAfterCmd;
  bool m_posCtrl;
  bool m_velMode;
  unsigned int navi_state;

  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_handle_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posctrl_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // extra robot control command
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_mode_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_gt_pose_; //for publishing ground truth pose
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_vec_; //ground truth velocity in the body frame
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_acc_; //ground truth acceleration in the body frame

  geometry_msgs::msg::Twist cmd_val;
  // callback functions for subscribers
  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void LandCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Time state_stamp_;
  ignition::math::v6::Pose3<double> pose;
  ignition::math::v6::Vector3<double> euler;
  ignition::math::v6::Vector3<double> velocity, acceleration, angular_velocity, position;

  std::string link_name_;
  std::string model_name_;
  std::string cmd_normal_topic_;
  std::string switch_mode_topic_;
  std::string posctrl_topic_;
  std::string imu_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
  std::string gt_topic_;
  std::string gt_vel_topic_;
  std::string gt_acc_topic_;
  
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
  } controllers_;

  ignition::math::v6::Vector3<double> inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // PLUGIN_DRONE_HPP