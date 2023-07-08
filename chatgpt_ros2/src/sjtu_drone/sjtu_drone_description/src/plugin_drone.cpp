#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

DroneSimpleController::DroneSimpleController()
{ 
  navi_state = LANDED_MODEL;
  m_posCtrl = false;
  m_velMode = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  this->updateConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  if(!rclcpp::ok()){
    RCLCPP_FATAL(rclcpp::get_logger("DroneSimpleController"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
  }

  
  world = _model->GetWorld();
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin is loading!");
  
  //default parameters
  model_name_ = _model->GetName().substr(0, _model->GetName().find("::"));
  cmd_normal_topic_ = "cmd_vel";
  imu_topic_ = "imu";
  takeoff_topic_ = "takeoff";
  land_topic_ = "land";
  reset_topic_ = "reset";
  posctrl_topic_ = "posctrl";
  switch_mode_topic_ = "dronevel_mode";
  gt_topic_ = "gt_pose";
  gt_vel_topic_ = "gt_vel";
  gt_acc_topic_ = "gt_acc";
  
  
  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
  }

  if (!link)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DroneSimpleController"), "gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("DroneSimpleController"), "Using following parameters: \n" <<
                      "\t\tlink_name: "<<  link_name_.c_str() << ",\n" <<
                      "\t\tmax_force: "<<  max_force_ << ",\n" <<
                      "\t\tmotion_small_noise: "<<  motion_small_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise: "<<  motion_drift_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise_time: "<<  motion_drift_noise_time_
                    );

  // get inertia and mass of quadrotor body
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();

  node_handle_ = std::make_shared<rclcpp::Node>("control", model_name_);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  ////////////////////////////////////////////////////////////////////////////////
  // Subscribers
  // subscribe command: control command
  if (!cmd_normal_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(cmd_normal_topic_ + "/statistics");

    cmd_subscriber_ = node_handle_->create_subscription<geometry_msgs::msg::Twist>(cmd_normal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&DroneSimpleController::CmdCallback, this, std::placeholders::_1),sub_opt);
    if (cmd_subscriber_->get_topic_name()[0] != '\0') 
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using cmd_topic %s", cmd_normal_topic_.c_str());
    else 
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the cmd_topic: %s !", cmd_normal_topic_.c_str());
  } else
    RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No cmd_topic defined!");
  
  if (!posctrl_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group( rclcpp::CallbackGroupType::Reentrant);

    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(posctrl_topic_ + "/statistics");

    posctrl_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::PosCtrlCallback, this, std::placeholders::_1),
        sub_opt);

    if (posctrl_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using position control topic: %s!", posctrl_subscriber_->get_topic_name());
    else
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the position control topic: %s !", posctrl_topic_.c_str());
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No position control defined!");

  // subscribe imu
  if (!imu_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(imu_topic_ + "/statistics");
    imu_subscriber_ = node_handle_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ImuCallback, this, std::placeholders::_1),
      sub_opt);

    if (imu_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using imu on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the IMU topic: %s !", imu_topic_.c_str());
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No imu topic defined!");


  // subscribe command: take off command
  if (!takeoff_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(takeoff_topic_ + "/statistics");

    takeoff_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::TakeoffCallback, this, std::placeholders::_1),
      sub_opt
    );

    if (takeoff_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the takeoff topic: %s", takeoff_subscriber_->get_topic_name() );
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the takeoff topic: %s !", takeoff_topic_.c_str());
  }else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No takeoff topic defined!");

  // subscribe command: land command
  if (!land_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(land_topic_ + "/statistics");
    land_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::LandCallback, this, std::placeholders::_1),
      sub_opt);

      if (land_subscriber_->get_topic_name()[0] != '\0') 
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the land topic: %s", land_subscriber_->get_topic_name() );
      else
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the land topic: %s !", land_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No land topic defined!");

  // subscribe command: reset command
  if (!reset_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(reset_topic_ + "/statistics");
    reset_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ResetCallback, this, std::placeholders::_1),
      sub_opt);

    if (reset_subscriber_->get_topic_name()[0] != '\0')
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the reset topic: %s", reset_subscriber_->get_topic_name() );
      else 
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the reset topic: %s !", reset_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No reset topic defined!");
  
  if (!switch_mode_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(switch_mode_topic_ + "/statistics");
    switch_mode_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::SwitchModeCallback, this, std::placeholders::_1),
      sub_opt);

    if (switch_mode_subscriber_->get_topic_name()[0] != '\0')
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the switch mode topic: %s", switch_mode_subscriber_->get_topic_name() );
      else 
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the switch mode topic: %s !", switch_mode_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No switch mode topic defined!");


  ////////////////////////////////////////////////////////////////////////////////
  // Publishers
  if (!gt_topic_.empty()){
    pub_gt_pose_ = node_handle_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_,1024);  
    if (pub_gt_pose_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth pose topic on: %s !", pub_gt_pose_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground truth topic: %s !", gt_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth topic defined!");

  if (!gt_vel_topic_.empty())
  {
    pub_gt_vec_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_vel", 1024);
  if (pub_gt_vec_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth velocity topic on: %s !", pub_gt_vec_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground truth velocity topic: %s !", gt_vel_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth velocity topic defined!");

  if (!gt_acc_topic_.empty())
  {
    pub_gt_acc_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_acc", 1024);
    if (pub_gt_acc_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth acceleration topic on: %s !", pub_gt_acc_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground truth acceleration topic: %s !", gt_acc_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth acceleration topic defined!");

  

  LoadControllerSettings(_model, _sdf);
  
  Reset();

  executor_->add_node(node_handle_);
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DroneSimpleController::Update, this));

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin finished loading!");
}

/**
 * @brief Initiliaze the PID params
 * 
 * @param _model shared pointer to the model object
 * @param _sdf shared pointer to the sdf object
 */
void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("DroneSimpleController"), "Using the PID parameters: \n" <<
                        "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p << ", kI: " << controllers_.roll.gain_i << ",kD: " << controllers_.roll.gain_d << ", Limit: " << controllers_.roll.limit << ", Time Constant: " << controllers_.roll.time_constant << "\n" << 
                        "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p << ", kI: " << controllers_.yaw.gain_i << ",kD: " << controllers_.yaw.gain_d << ", Limit: " << controllers_.yaw.limit << ", Time Constant: " << controllers_.yaw.time_constant << "\n" << 
                        "\tVelocity X:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p << ", kI: " << controllers_.velocity_x.gain_i << ",kD: " << controllers_.velocity_x.gain_d << ", Limit: " << controllers_.velocity_x.limit << ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" << 
                        "\tVelocity Y:\n" << "\t\tkP: " << controllers_.velocity_y.gain_p << ", kI: " << controllers_.velocity_y.gain_i << ",kD: " << controllers_.velocity_y.gain_d << ", Limit: " << controllers_.velocity_y.limit << ", Time Constant: " << controllers_.velocity_y.time_constant << "\n" << 
                        "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p << ", kI: " << controllers_.velocity_z.gain_i << ",kD: " << controllers_.velocity_z.gain_d << ", Limit: " << controllers_.velocity_z.limit << ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" << 
                        "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p << ", kI: " << controllers_.pos_x.gain_i << ",kD: " << controllers_.pos_x.gain_d << ", Limit: " << controllers_.pos_x.limit << ", Time Constant: " << controllers_.pos_x.time_constant << "\n" << 
                        "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p << ", kI: " << controllers_.pos_z.gain_i << ",kD: " << controllers_.pos_z.gain_d << ", Limit: " << controllers_.pos_z.limit << ", Time Constant: " << controllers_.pos_z.time_constant
    );
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
/**
* @brief Callback function for the drone command topic.
* This function is called whenever a new message is received on the drone command topic. It updates
* the cmd_val member variable with the new command message. It also generates motion noise for the
* drone's angular and linear velocities by adding drift and small noise values to the command message.
* The amount of noise added is determined by the motion_drift_noise_time_ and motion_small_noise_
* member variables of the DroneSimpleController class.
* The function uses the world->SimTime() function to get the current simulator time and calculate the
* time difference between the current and last simulation time. The time difference is used to update
* the drift noise values if the time_counter_for_drift_noise is greater than motion_drift_noise_time_.
* The updated command message is then used to update the cmd_val member variable.
* 
* @param cmd Pointer to the command message containing linear and angular velocities.
*/
void DroneSimpleController::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  cmd_val = *cmd;


  static common::Time last_sim_time = world->SimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->SimTime();
  double dt = (cur_sim_time - last_sim_time).Double();
  // save last time stamp
  last_sim_time = cur_sim_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_)
  {
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);

}

/**
* @brief Callback function for position control command.
* This function is called when a new position control command is received.
* It sets the m_posCtrl flag to the value of the command data.
* @param cmd The position control command message.
*/
void DroneSimpleController::PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd)
{
    m_posCtrl = cmd->data;
}

/**
* @brief Callback function to handle IMU sensor data.
* @param imu Shared pointer to IMU sensor data.
* The function reads the quaternion data from the IMU sensor and updates the orientation and angular velocity of the drone.
*/
void DroneSimpleController::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(ignition::math::v6::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

/**
* @brief Callback function to initiate taking off of the drone.
* @param msg Empty message.
*/
void DroneSimpleController::TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Quadrotor takes off!!");
  }
}

/**
* @brief Callback function to initiate landing of the drone.
* @param msg Empty message.
*/
void DroneSimpleController::LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
  {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Quadrotor lands!!");
  }
}

/**
* @brief Callback function for reset command
* This function resets the controller and the drone's state.
* @param msg Empty message
*/
void DroneSimpleController::ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Reset quadrotor!!");
  Reset();
}

/**
* @brief Callback function for receiving a message to switch between velocity and position control modes
* @param msg Shared pointer to the message containing the boolean value for switching the mode
* The function switches between velocity and position control modes based on the boolean value in the message.
* If the boolean value is true, the control mode is switched to velocity control and if it's false, the control
* mode is switched to position control. It also resets the integral term of the controllers for the new mode.
*/
void DroneSimpleController::SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_velMode = msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
/**
* @brief Update method called by the Gazebo simulator every simulation iteration.
*/
void DroneSimpleController::Update()
{ 
    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0) return;
    
    executor_->spin_some(std::chrono::milliseconds(100));
    UpdateState(dt);
    UpdateDynamics(dt);
    
    // save last time stamp
    last_time = sim_time;   
}

/**
* @brief Updates the current state of the drone.
* This method is responsible for updating the current state of the drone based on the navigation state. If the drone is taking off, it checks if the time after the command is greater than 0.5 seconds. If it is, it sets the navigation state to flying. If the drone is landing, it checks if the time after the command is greater than 1 second. If it is, it sets the navigation state to landed. If the drone is neither taking off nor landing, it resets the time after the command to zero.
* 
* @param dt The time elapsed since the last update, in seconds.
*/
void DroneSimpleController::UpdateState(double dt){
    if(navi_state == TAKINGOFF_MODEL){
        
        m_timeAfterCmd += dt;
        if (m_timeAfterCmd > 0.5){
            navi_state = FLYING_MODEL;
            std::cout << "Entering flying model!" << std::endl;
        }
    }else if(navi_state == LANDING_MODEL){
        m_timeAfterCmd += dt;
        if(m_timeAfterCmd > 1.0){
            navi_state = LANDED_MODEL;
            std::cout << "Landed!" <<std::endl;
        }
    }else
        m_timeAfterCmd = 0;
}


/**
* @brief Update the dynamics of the drone.
* This method updates the dynamics of the drone based on its current state and the current
* commands being received. It computes the force and torque to be applied to the drone, and
* updates its position, velocity, and orientation accordingly. It also publishes the ground
* truth pose, velocity, and acceleration of the drone to ROS topics.
* 
* @param dt The time step to use for the update.
*/
void DroneSimpleController::UpdateDynamics(double dt){
  ignition::math::v6::Vector3<double> force, torque;
   
  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  //  if (imu_subscriber_.getTopic()=="")
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
    }
   // if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
    
    
    //publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::msg::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();
    
    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();
    pub_gt_pose_->publish(gt_pose);
    
    //convert the acceleration and velocity into the body frame
    ignition::math::v6::Vector3 body_vel = pose.Rot().RotateVector(velocity);
    ignition::math::v6::Vector3 body_acc = pose.Rot().RotateVector(acceleration);
    
    //publish the velocity
    geometry_msgs::msg::Twist tw;
    tw.linear.x = body_vel.X();
    tw.linear.y = body_vel.Y();
    tw.linear.z = body_vel.Z();
    pub_gt_vec_->publish(tw);
    
    //publish the acceleration
    tw.linear.x = body_acc.X();
    tw.linear.y = body_acc.Y();
    tw.linear.z = body_acc.Z();
    pub_gt_acc_->publish(tw);
    
            
    ignition::math::v6::Vector3 poschange = pose.Pos() - position;
    position = pose.Pos();
    
  
    // Get gravity
    ignition::math::v6::Vector3 gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity
  
    // Rotate vectors to coordinate frames relevant for control
    ignition::math::v6::Quaternion heading_quaternion(cos(euler[2]/2), 0.0, 0.0, sin(euler[2]/2));
    ignition::math::v6::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::v6::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::v6::Vector3 angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
  
    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    
    if( m_posCtrl){
        //position control
        if(navi_state == FLYING_MODEL){
            double vx = controllers_.pos_x.update(cmd_val.linear.x, position[0], poschange[0], dt);
            double vy = controllers_.pos_y.update(cmd_val.linear.y, position[1], poschange[1], dt);
            double vz = controllers_.pos_z.update(cmd_val.linear.z, position[2], poschange[2], dt);

            ignition::math::v6::Vector3 vb = heading_quaternion.RotateVectorReverse(ignition::math::v6::Vector3(vx,vy,vz));
            
            double pitch_command =  controllers_.velocity_x.update(vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
            torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);            
            force[2]  = mass      * (controllers_.velocity_z.update(vz,  velocity[2], acceleration[2], dt) + load_factor * gravity);
        }
    }else{
        //normal control
        if( navi_state == FLYING_MODEL )
        {
          //hovering
          double pitch_command =  controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
          double roll_command  = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
          torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
        }else{
          //control by velocity
          if( m_velMode){
              double pitch_command =  controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
              double roll_command  = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
              torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
              torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);              
          }else{
            //control by tilting
            torque[0] = inertia[0] *  controllers_.roll.update(cmd_val.angular.x, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(cmd_val.angular.y, euler[1], angular_velocity_body[1], dt);
          }
        }
        torque[2] = inertia[2] *  controllers_.yaw.update(cmd_val.angular.z, angular_velocity[2], 0, dt);
        force[2]  = mass      * (controllers_.velocity_z.update(cmd_val.linear.z,  velocity[2], acceleration[2], dt) + load_factor * gravity);
    }

    
    if (max_force_ > 0.0 && force[2] > max_force_) force[2] = max_force_;
    if (force[2] < 0.0) force[2] = 0.0;
    
    
  
    // process robot state information
    if(navi_state == LANDED_MODEL)
    {
  
    }
    else if(navi_state == FLYING_MODEL)
    {
      link->AddRelativeForce(force);
      link->AddRelativeTorque(torque);
    }
    else if(navi_state == TAKINGOFF_MODEL)
    {
      link->AddRelativeForce(force*1.5);
      link->AddRelativeTorque(torque*1.5);
    }
    else if(navi_state == LANDING_MODEL)
    {
      link->AddRelativeForce(force*0.8);
      link->AddRelativeTorque(torque*0.8);
    }
   
}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
/**
 * @brief Reset the state of the drone controller and its associated objects to their initial values.
 * This method is called when the simulation is reset.
 */
void DroneSimpleController::Reset()
{
  // Reset the values of the controllers
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  // Set the force and torque acting on the drone to zero
  link->SetForce(ignition::math::Vector3(0.0, 0.0, 0.0));
  link->SetTorque(ignition::math::v6::Vector3(0.0, 0.0, 0.0));

  // Reset the state of the drone
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp_ = rclcpp::Time();

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo