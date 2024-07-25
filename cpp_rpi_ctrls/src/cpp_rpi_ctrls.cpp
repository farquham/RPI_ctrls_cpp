#include "../include/cpp_rpi_ctrls/cpp_rpi_ctrls.h"

void RPI::rpictrls::odom_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg) {
    this->pos_[0] = msg->pose.position.x;
    this->pos_[1] = -1*msg->pose.position.y;
    this->pos_[2] = -1* msg->pose.position.z;
    this->quat_.w() = msg->pose.orientation.w;
    this->quat_.x() = msg->pose.orientation.x;
    this->quat_.y() = -1*msg->pose.orientation.y;
    this->quat_.z() = -1*msg->pose.orientation.z;
    //RCLCPP_INFO(this->get_logger(), "Received odom pose: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f", this->pos_[0], this->pos_[1], this->pos_[2], this->quat_.w(), this->quat_.x(), this->quat_.y(), this->quat_.z());
}


void RPI::rpictrls::publish_odom(const Eigen::Vector3d pos, const Eigen::Quaterniond quat) {
    auto odom_msg = px4_msgs::msg::VehicleOdometry();
    odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    odom_msg.pose_frame = 2;
    odom_msg.position[0] = pos[0];
    odom_msg.position[1] = pos[1];
    odom_msg.position[2] = pos[2];
    odom_msg.q[0] = quat.w();
    odom_msg.q[1] = quat.x();
    odom_msg.q[2] = quat.y();
    odom_msg.q[3] = quat.z();
    odom_msg.position_variance = {0.0, 0.0, 0.0};
    odom_msg.orientation_variance = {0.0, 0.0, 0.0};
    odom_pub_->publish(odom_msg);
    //RCLCPP_INFO(this->get_logger(), "Published odom: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f", pos[0], pos[1], pos[2], quat.w(), quat.x(), quat.y(), quat.z());
}

void RPI::rpictrls::arm() {
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0);
    RCLCPP_INFO(this->get_logger(), "Arming vehicle");
}

void RPI::rpictrls::disarm() {
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 0);
    RCLCPP_INFO(this->get_logger(), "Disarming vehicle");
}

void RPI::rpictrls::publish_vehicle_command(uint16_t command, float param1, float param2) {
    auto vehicle_command_msg = px4_msgs::msg::VehicleCommand();
    vehicle_command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_msg.command = command;
    vehicle_command_msg.param1 = param1;
    vehicle_command_msg.param2 = param2;
    vehicle_command_msg.target_system = 1;
    vehicle_command_msg.target_component = 1;
    vehicle_command_msg.source_system = 1;
    vehicle_command_msg.source_component = 1;
    vehicle_command_msg.from_external = true;
    vehicle_command_pub_->publish(vehicle_command_msg);
}

void RPI::rpictrls::publish_obctrl_mode() {
    auto offboard_control_mode_msg = px4_msgs::msg::OffboardControlMode();
    offboard_control_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_msg.position = true;
    offboard_control_mode_msg.velocity = false;
    offboard_control_mode_msg.acceleration = false;
    offboard_control_mode_msg.attitude = false;
    offboard_control_mode_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_control_mode_msg);
}

void RPI::rpictrls::publish_trajectory_setpoint(const Eigen::Vector3d pos) {
    auto trajectory_setpoint_msg = px4_msgs::msg::TrajectorySetpoint();
    trajectory_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_msg.position = {float(pos[0]), float(pos[1]), float(pos[2])};
    trajectory_setpoint_msg.yaw = 0.0;
    trajectory_setpoint_pub_->publish(trajectory_setpoint_msg);
    RCLCPP_INFO(this->get_logger(), "Published trajectory setpoint: x=%f, y=%f, z=%f", pos[0], pos[1], pos[2]);
}

void RPI::rpictrls::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr & msg) {
    this->quat_local.w() = msg->q[0];
    this->quat_local.x() = msg->q[2];
    this->quat_local.y() = msg->q[1];
    this->quat_local.z() = -1*msg->q[3];
}

void RPI::rpictrls::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr & msg) {
    this->pos_local[0] = msg->y;
    this->pos_local[1] = msg->x;
    this->pos_local[2] = -1*msg->z;
}

void RPI::rpictrls::vehicle_acceleration_callback(const px4_msgs::msg::VehicleAcceleration::UniquePtr & msg) {
    this->accel_local[0] = msg->xyz[1];
    this->accel_local[1] = msg->xyz[0];
    this->accel_local[2] = -1*msg->xyz[2];
}

void RPI::rpictrls::vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr & msg) {
    this->avel_local[0] = msg->xyz[1];
    this->avel_local[1] = msg->xyz[0];
    this->avel_local[2] = -1*msg->xyz[2];
}

void RPI::rpictrls::ext_setpoint_callback(const geometry_msgs::msg::Point::UniquePtr & msg) {
    this->pos_cmd_[0] = msg->y;
    this->pos_cmd_[1] = msg->x;
    this->pos_cmd_[2] = -1*msg->z;
}

void RPI::rpictrls::publish_state(const Eigen::Vector3d poslocal, const Eigen::Quaterniond quatlocal, const Eigen::Vector3d avellocal, const Eigen::Vector3d accellocal) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.pose.position.x = poslocal[0];
    pose_msg.pose.position.y = poslocal[1];
    pose_msg.pose.position.z = poslocal[2];
    pose_msg.pose.orientation.w = quatlocal.w();
    pose_msg.pose.orientation.x = quatlocal.x();
    pose_msg.pose.orientation.y = quatlocal.y();
    pose_msg.pose.orientation.z = quatlocal.z();
    pose_pub_->publish(pose_msg);

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->get_clock()->now();
    twist_msg.twist.linear.x = avellocal[0];
    twist_msg.twist.linear.y = avellocal[1];
    twist_msg.twist.linear.z = avellocal[2];
    twist_pub_->publish(twist_msg);

    auto accel_msg = geometry_msgs::msg::AccelStamped();
    accel_msg.header.stamp = this->get_clock()->now();
    accel_msg.accel.linear.x = accellocal[0];
    accel_msg.accel.linear.y = accellocal[1];
    accel_msg.accel.linear.z = accellocal[2];
    accel_pub_->publish(accel_msg);

    //RCLCPP_INFO(this->get_logger(), "Published state: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f, avelx=%f, avely=%f, avelz=%f, accelx=%f, accely=%f, accelz=%f", poslocal[0], poslocal[1], poslocal[2], quatlocal.w(), quatlocal.x(), quatlocal.y(), quatlocal.z(), avellocal[0], avellocal[1], avellocal[2], accellocal[0], accellocal[1], accellocal[2]);
}
