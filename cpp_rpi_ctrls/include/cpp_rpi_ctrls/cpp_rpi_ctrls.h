#ifndef RPICONTROLS_H
#define RPICONTROLS_H

// basic cpp libs
#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include <cstdio>
#include <stdint.h>

// ros2 libs
#include <rclcpp/rclcpp.hpp>

// geo ros msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// px4 ros msgs
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_acceleration.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
//offboard control
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

// some nanespace stuff
using namespace std::chrono;
using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;

namespace RPI {
	// class for offboard control which starts a ROS2 node
	class rpictrls : public rclcpp::Node {
	public:
		rpictrls() : Node("rpi_ctrls")
		{
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/baby_michael/pose", qos,
            [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
                pos_[0] = msg->pose.position.x;
                pos_[1] = msg->pose.position.y;
                pos_[2] = msg->pose.position.z;
                quat_.w() = msg->pose.orientation.w;
                quat_.x() = msg->pose.orientation.x;
                quat_.y() = msg->pose.orientation.y;
                quat_.z() = msg->pose.orientation.z;
            });

            odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

            auto timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                publish_odom(pos_, quat_);			
            };

			timer_ = this->create_wall_timer(10ms, timer_callback);
		}

	private:
		// needed optitrack stuff
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
        // offboard control stuff
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        // drone state subscribers
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAcceleration>::SharedPtr vehicle_acceleration_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr vehicle_angular_velocity_sub_;
        // gc comms stuff
        // ext setpoint sub
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ext_setpoint_sub_;
        // state publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::atomic<uint64_t> timestamp_;
		
        void publish_odom(const Eigen::Vector3d pos, const Eigen::Quaterniond quat);

        Eigen::Vector3d pos_;
        Eigen::Quaterniond quat_;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RPI::rpictrls>());
	rclcpp::shutdown();
	return 0;
}

#endif


