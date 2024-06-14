#include "../include/cpp_rpi_ctrls/cpp_rpi_ctrls.h"

void RPI::rpictrls::publish_odom(const Eigen::Vector3d pos, const Eigen::Quaterniond quat) {
    auto odom_msg = px4_msgs::msg::VehicleOdometry();
    odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    odom_msg.pose_frame = 1;
    odom_msg.position[0] = pos[0];
    odom_msg.position[1] = pos[2];
    odom_msg.position[2] = -1*pos[1];
    odom_msg.q[0] = quat.w();
    odom_msg.q[1] = quat.x();
    odom_msg.q[2] = quat.z();
    odom_msg.q[3] = -1*quat.y();
    odom_msg.position_variance = {0.0, 0.0, 0.0};
    odom_msg.orientation_variance = {0.0, 0.0, 0.0};
    odom_pub_->publish(odom_msg);
}