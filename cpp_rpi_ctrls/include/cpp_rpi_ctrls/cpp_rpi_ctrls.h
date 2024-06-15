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
            // required qos settings
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            // odom publisher and subscriber
            odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/baby_michael/pose", qos, std::bind(&rpictrls::odom_callback, this, std::placeholders::_1));
            odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

            // offboard control stuff
            offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

            // drone state subscribers
            vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&rpictrls::vehicle_attitude_callback, this, std::placeholders::_1));
            vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&rpictrls::vehicle_local_position_callback, this, std::placeholders::_1));
            vehicle_acceleration_sub_ = this->create_subscription<px4_msgs::msg::VehicleAcceleration>("/fmu/out/vehicle_acceleration", qos, std::bind(&rpictrls::vehicle_acceleration_callback, this, std::placeholders::_1));
            vehicle_angular_velocity_sub_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos, std::bind(&rpictrls::vehicle_angular_velocity_callback, this, std::placeholders::_1));

            // ext setpoint sub
            ext_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/rpi/in/ext_cmdPoint", qos, std::bind(&rpictrls::ext_setpoint_callback, this, std::placeholders::_1));

            // state publishers
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/rpi/out/vehicle_pose", 10);
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/rpi/out/vehicle_velocity", 10);
            accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("/rpi/out/vehicle_acceleration", 10);


            auto odom_timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                publish_odom(pos_, quat_);			
            };

            offboard_setpoint_counter_ = 0;

            auto ob_timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                if (offboard_setpoint_counter_ == 10) {
                    // Change to Offboard mode after 10 setpoints
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                    // Arm the vehicle
                    this->arm();
                }

                // offboard_control_mode needs to be paired with trajectory_setpoint
                publish_obctrl_mode();
                publish_trajectory_setpoint(pos_cmd_);

                // stop the counter after reaching 11
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }		
            };

            auto state_timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                publish_state(pos_local, quat_local, avel_local, accel_local);			
            };

			odom_timer_ = this->create_wall_timer(10ms, odom_timer_callback);
            ob_timer_ = this->create_wall_timer(10ms, ob_timer_callback);
            state_timer_ = this->create_wall_timer(10ms, state_timer_callback);
		}
        void arm();
        void disarm();

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

        rclcpp::TimerBase::SharedPtr odom_timer_;
        rclcpp::TimerBase::SharedPtr ob_timer_;
        rclcpp::TimerBase::SharedPtr state_timer_;

        std::atomic<uint64_t> timestamp_;

        void odom_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg);
        void publish_odom(const Eigen::Vector3d pos, const Eigen::Quaterniond quat);

        void publish_vehicle_command(uint16_t command, float param1, float param2);
        void publish_obctrl_mode();
        void publish_trajectory_setpoint(const Eigen::Vector3d pos);
        uint64_t offboard_setpoint_counter_;

        void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr & msg);
        void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr & msg);
        void vehicle_acceleration_callback(const px4_msgs::msg::VehicleAcceleration::UniquePtr & msg);
        void vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr & msg);

        void ext_setpoint_callback(const geometry_msgs::msg::Point::UniquePtr & msg);

        void publish_state(const Eigen::Vector3d poslocal, const Eigen::Quaterniond quatlocal, const Eigen::Vector3d avellocal, const Eigen::Vector3d accellocal);

        Eigen::Vector3d pos_;
        Eigen::Quaterniond quat_;

        Eigen::Vector3d pos_cmd_;

        Eigen::Vector3d pos_local;
        Eigen::Quaterniond quat_local;
        Eigen::Vector3d avel_local;
        Eigen::Vector3d accel_local;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RPI::rpictrls>());
	rclcpp::shutdown();
	return 0;
}

#endif


