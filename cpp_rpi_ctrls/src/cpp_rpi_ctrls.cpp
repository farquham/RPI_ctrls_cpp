#include "../include/cpp_rpi_ctrls/cpp_rpi_ctrls.h"

void LocalNavigation::odom_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg) {
    this->pos_ros[0] = msg->pose.position.x;
    this->pos_ros[1] = msg->pose.position.y;
    this->pos_ros[2] = msg->pose.position.z;
    this->quat_ros.w() = msg->pose.orientation.w;
    this->quat_ros.x() = msg->pose.orientation.x;
    this->quat_ros.y() = msg->pose.orientation.y;
    this->quat_ros.z() = msg->pose.orientation.z;
    this->pos_px4 = px4_ros_com::frame_transforms::enu_to_ned_local_frame(this->pos_ros);
    this->quat_px4 = px4_ros_com::frame_transforms::ros_to_px4_orientation(this->quat_ros);
    //RCLCPP_INFO(this->get_logger(), "Received odom pose: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f", this->pos_ros[0], this->pos_ros[1], this->pos_ros[2], this->quat_ros.w(), this->quat_ros.x(), this->quat_ros.y(), this->quat_ros.z());
}

void LocalNavigation::updateLocalPosition() {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = _node.get_clock()->now();

    local_position_measurement.velocity_xy = Eigen::Vector2f {float(this->pos_px4[0]), float(this->pos_px4[1])};
    // local_position_measurement.velocity_xy_variance = Eigen::Vector2f {0.3f, 0.4f};
    local_position_measurement.velocity_xy_variance = Eigen::Vector2f {0.0f, 0.0f};

    local_position_measurement.position_z = float(this->pos_px4[2]);
    // local_position_measurement.position_z_variance = 0.33f;
    local_position_measurement.position_z_variance = 0.0f;

    local_position_measurement.attitude_quaternion = Eigen::Quaternionf {float(this->quat_px4.w()), float(this->quat_px4.x()), float(this->quat_px4.y()), float(this->quat_px4.z())};
    // local_position_measurement.attitude_variance = Eigen::Vector3f {0.2f, 0.1f, 0.05f};
    local_position_measurement.attitude_variance = Eigen::Vector3f {0.0f, 0.0f, 0.0f};

    try {
        update(local_position_measurement);
        RCLCPP_DEBUG(_node.get_logger(),"Successfully sent position update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e){
        RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
}

void LocalNavigation::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr & msg) {
    this->quat_local.w() = msg->q[0];
    this->quat_local.x() = msg->q[1];
    this->quat_local.y() = msg->q[2];
    this->quat_local.z() = msg->q[3];
}

void LocalNavigation::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr & msg) {
    this->pos_local[0] = msg->x;
    this->pos_local[1] = msg->y;
    this->pos_local[2] = msg->z;
}

void LocalNavigation::vehicle_acceleration_callback(const px4_msgs::msg::VehicleAcceleration::UniquePtr & msg) {
    this->accel_local[0] = msg->xyz[0];
    this->accel_local[1] = msg->xyz[1];
    this->accel_local[2] = msg->xyz[2];
}

void LocalNavigation::vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr & msg) {
    this->avel_local[0] = msg->xyz[0];
    this->avel_local[1] = msg->xyz[1];
    this->avel_local[2] = msg->xyz[2];
}

void LocalNavigation::ext_setpoint_callback(const geometry_msgs::msg::Point::UniquePtr & msg) {
    this->pos_cmd_[0] = msg->x;
    this->pos_cmd_[1] = msg->y;
    this->pos_cmd_[2] = msg->z;
}

void LocalNavigation::publish_state(const Eigen::Vector3d poslocal, const Eigen::Quaterniond quatlocal, const Eigen::Vector3d avellocal, const Eigen::Vector3d accellocal) {
    Eigen::Vector3d poslocal_ros;
    Eigen::Quaterniond quatlocal_ros;
    poslocal_ros = px4_ros_com::frame_transforms::ned_to_enu_local_frame(poslocal);
    quatlocal_ros = px4_ros_com::frame_transforms::px4_to_ros_orientation(quatlocal);
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = _node.get_clock()->now();
    pose_msg.pose.position.x = poslocal_ros[0];
    pose_msg.pose.position.y = poslocal_ros[1];
    pose_msg.pose.position.z = poslocal_ros[2];
    pose_msg.pose.orientation.w = quatlocal_ros.w();
    pose_msg.pose.orientation.x = quatlocal_ros.x();
    pose_msg.pose.orientation.y = quatlocal_ros.y();
    pose_msg.pose.orientation.z = quatlocal_ros.z();
    pose_pub_->publish(pose_msg);

    Eigen::Vector3d avellocal_ros;
    avellocal_ros = px4_ros_com::frame_transforms::ned_to_enu_local_frame(avellocal);
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = _node.get_clock()->now();
    twist_msg.twist.linear.x = avellocal_ros[0];
    twist_msg.twist.linear.y = avellocal_ros[1];
    twist_msg.twist.linear.z = avellocal_ros[2];
    twist_pub_->publish(twist_msg);

    Eigen::Vector3d accellocal_ros;
    accellocal_ros = px4_ros_com::frame_transforms::ned_to_enu_local_frame(accellocal);
    auto accel_msg = geometry_msgs::msg::AccelStamped();
    accel_msg.header.stamp = _node.get_clock()->now();
    accel_msg.accel.linear.x = accellocal_ros[0];
    accel_msg.accel.linear.y = accellocal_ros[1];
    accel_msg.accel.linear.z = accellocal_ros[2];
    accel_pub_->publish(accel_msg);

    //RCLCPP_INFO(this->get_logger(), "Published state: x=%f, y=%f, z=%f, qw=%f, qx=%f, qy=%f, qz=%f, avelx=%f, avely=%f, avelz=%f, accelx=%f, accely=%f, accelz=%f", poslocal_ros[0], poslocal_ros[1], poslocal_ros[2], quatlocal_ros.w(), quatlocal_ros.x(), quatlocal_ros.y(), quatlocal_ros.z(), avellocal_ros[0], avellocal_ros[1], avellocal_ros[2], accellocal_ros[0], accellocal_ros[1], accellocal_ros[2]);
}
