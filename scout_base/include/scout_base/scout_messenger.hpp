/*
 * scout_messenger.hpp
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

 #ifndef SCOUT_MESSENGER_HPP
 #define SCOUT_MESSENGER_HPP
 
 #include <string>
 #include <mutex>
 #include <memory>
 
 #include <rclcpp/rclcpp.hpp>
 #include <nav_msgs/msg/odometry.hpp>
 #include <geometry_msgs/msg/twist.hpp>
 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
 #include "scout_msgs/msg/scout_status.hpp"
 #include "scout_msgs/msg/scout_light_cmd.hpp"
 
 #include "ugv_sdk/mobile_robot/scout_robot.hpp"
 
 namespace westonrobot {
 template <typename ScoutType>
 class ScoutMessenger {
  public:
   ScoutMessenger(std::shared_ptr<ScoutType> scout, rclcpp::Node *node)
       : scout_(scout), node_(node), current_time_(node->now()) {
     last_time_sec_ = current_time_.seconds();
   }
 
   void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
   void SetBaseFrame(std::string frame) { base_frame_ = frame; }
   void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
 
   void SetSimulationMode(int loop_rate) {
     simulated_robot_ = true;
     sim_control_rate_ = loop_rate;
   }
 
   void SetupSubscription() {
     odom_pub_ =
         node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
     status_pub_ = node_->create_publisher<scout_msgs::msg::ScoutStatus>(
         "/scout_status", 10);
 
     motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
         "/cmd_vel", 5,
         std::bind(&ScoutMessenger::TwistCmdCallback, this,
                   std::placeholders::_1));
     light_cmd_sub_ = node_->create_subscription<scout_msgs::msg::ScoutLightCmd>(
         "/light_control", 5,
         std::bind(&ScoutMessenger::LightCmdCallback, this,
                   std::placeholders::_1));
 
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
   }
 
   void PublishStateToROS() {
     current_time_ = node_->now();
 
     static bool init_run = true;
     if (init_run) {
       last_time_sec_ = current_time_.seconds();
       init_run = false;
       return;
     }
     
     // Matemática pura de decimais (double) para evitar erro de type match
     double dt = current_time_.seconds() - last_time_sec_;
     if (dt < 0.0 || dt > 1.0) { 
         dt = 0.02; 
     }
 
     auto state = scout_->GetRobotState();
 
     scout_msgs::msg::ScoutStatus status_msg;
     status_msg.header.stamp = current_time_;
     status_msg.linear_velocity = state.motion_state.linear_velocity;
     status_msg.angular_velocity = state.motion_state.angular_velocity;
     status_msg.vehicle_state = state.system_state.vehicle_state;
     status_msg.control_mode = state.system_state.control_mode;
     status_msg.error_code = state.system_state.error_code;
     status_msg.battery_voltage = state.system_state.battery_voltage;
 
     auto actuator = scout_->GetActuatorState();
 
     for (int i = 0; i < 4; ++i) {
       uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;
       status_msg.actuator_states[motor_id].rpm = actuator.actuator_hs_state[i].rpm;
       status_msg.actuator_states[motor_id].current = actuator.actuator_hs_state[i].current;
 
       motor_id = actuator.actuator_ls_state[i].motor_id;
       status_msg.actuator_states[motor_id].driver_voltage = actuator.actuator_ls_state[i].driver_voltage;
       status_msg.actuator_states[motor_id].driver_temperature = actuator.actuator_ls_state[i].driver_temp;
       status_msg.actuator_states[motor_id].motor_temperature = actuator.actuator_ls_state[i].motor_temp;
     }
 
     status_msg.light_control_enabled = state.light_state.enable_cmd_ctrl;
     status_msg.front_light_state.mode = state.light_state.front_light.mode;
     status_msg.front_light_state.custom_value = state.light_state.front_light.custom_value;
     status_msg.rear_light_state.mode = state.light_state.rear_light.mode;
     status_msg.rear_light_state.custom_value = state.light_state.rear_light.custom_value;
     status_pub_->publish(status_msg);
 
     PublishOdometryToROS(state.motion_state, dt);
 
     // Registra o tempo final
     last_time_sec_ = current_time_.seconds();
   }
 
  private:
   std::shared_ptr<ScoutType> scout_;
   rclcpp::Node *node_;
 
   std::string odom_frame_;
   std::string base_frame_;
   std::string odom_topic_name_;
 
   bool simulated_robot_ = false;
   int sim_control_rate_ = 50;
 
   std::mutex twist_mutex_;
   geometry_msgs::msg::Twist current_twist_;
 
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
   rclcpp::Publisher<scout_msgs::msg::ScoutStatus>::SharedPtr status_pub_;
 
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
   rclcpp::Subscription<scout_msgs::msg::ScoutLightCmd>::SharedPtr light_cmd_sub_;
 
   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 
   double position_x_ = 0.0;
   double position_y_ = 0.0;
   double theta_ = 0.0;
 
   // AQUI FOI ONDE A MÁGICA FOI CONSERTADA: Retornamos para double puro
   double last_time_sec_ = 0.0;
   rclcpp::Time current_time_;
 
   void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
     if (!simulated_robot_) {
       SetScoutMotionCommand(scout_, msg);
     } else {
       std::lock_guard<std::mutex> guard(twist_mutex_);
       current_twist_ = *msg.get();
     }
   }
 
   template <typename T,
             std::enable_if_t<!std::is_base_of<ScoutMiniOmniRobot, T>::value,
                              bool> = true>
   void SetScoutMotionCommand(std::shared_ptr<T> base,
                              const geometry_msgs::msg::Twist::SharedPtr &msg) {
     base->SetMotionCommand(msg->linear.x, msg->angular.z);
   }
 
   template <typename T,
             std::enable_if_t<std::is_base_of<ScoutMiniOmniRobot, T>::value,
                              bool> = true>
   void SetScoutMotionCommand(std::shared_ptr<T> base,
                              const geometry_msgs::msg::Twist::SharedPtr &msg) {
     base->SetMotionCommand(msg->linear.x, msg->angular.z, msg->linear.y);
   }
 
   void LightCmdCallback(const scout_msgs::msg::ScoutLightCmd::SharedPtr msg) {
     if (!simulated_robot_) {
       if (msg->cmd_ctrl_allowed) {
         AgxLightMode f_mode;
         uint8_t f_value;
 
         switch (msg->front_mode) {
           case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF: {
             f_mode = AgxLightMode::CONST_OFF;
             break;
           }
           case scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_ON: {
             f_mode = AgxLightMode::CONST_ON;
             break;
           }
           case scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH: {
             f_mode = AgxLightMode::BREATH;
             break;
           }
           case scout_msgs::msg::ScoutLightCmd::LIGHT_CUSTOM: {
             f_mode = AgxLightMode::CUSTOM;
             f_value = msg->front_custom_value;
             break;
           }
         }
         scout_->SetLightCommand(f_mode, f_value, AgxLightMode::CONST_ON, 0);
       } else {
         scout_->DisableLightControl();
       }
     }
   }
 
   geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
     tf2::Quaternion q;
     q.setRPY(0, 0, yaw);
     return tf2::toMsg(q);
   }
 
   void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
     double linear_speed = msg.linear_velocity;
     double angular_speed = msg.angular_velocity;
     double lateral_speed = 0;
 
     if (std::is_base_of<ScoutMiniOmniRobot, ScoutType>::value) {
       lateral_speed = msg.lateral_velocity;
     } else {
       lateral_speed = 0;
     }
 
     double d_x = linear_speed * std::cos(theta_) * dt;
     double d_y = (linear_speed * std::sin(theta_) + lateral_speed) * dt;
     double d_theta = angular_speed * dt;
 
     position_x_ += d_x;
     position_y_ += d_y;
     theta_ += d_theta;
 
     geometry_msgs::msg::Quaternion odom_quat =
         createQuaternionMsgFromYaw(theta_);
 
     geometry_msgs::msg::TransformStamped tf_msg;
     tf_msg.header.stamp = current_time_;
     tf_msg.header.frame_id = odom_frame_;
     tf_msg.child_frame_id = base_frame_;
 
     tf_msg.transform.translation.x = position_x_;
     tf_msg.transform.translation.y = position_y_;
     tf_msg.transform.translation.z = 0.0;
     tf_msg.transform.rotation = odom_quat;
 
     tf_broadcaster_->sendTransform(tf_msg);
 
     nav_msgs::msg::Odometry odom_msg;
     odom_msg.header.stamp = current_time_;
     odom_msg.header.frame_id = odom_frame_;
     odom_msg.child_frame_id = base_frame_;
 
     odom_msg.pose.pose.position.x = position_x_;
     odom_msg.pose.pose.position.y = position_y_;
     odom_msg.pose.pose.position.z = 0.0;
     odom_msg.pose.pose.orientation = odom_quat;
 
     odom_msg.twist.twist.linear.x = linear_speed;
     odom_msg.twist.twist.linear.y = lateral_speed;
     odom_msg.twist.twist.angular.z = angular_speed;
 
     odom_pub_->publish(odom_msg);
   }
 };
 }  // namespace westonrobot
 
 #endif /* SCOUT_MESSENGER_HPP */