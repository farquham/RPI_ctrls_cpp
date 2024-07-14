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
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_acceleration.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>

// px4 lib stuff
#include <px4_ros_com/frame_transforms.h>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>

// some nanespace stuff
using namespace std::chrono;
using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;


class LocalNavigation : public px4_ros2::LocalPositionMeasurementInterface {
    public:
        explicit LocalNavigation(rclcpp::Node & node): LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalNED, px4_ros2::VelocityFrame::LocalNED) {
            // required qos settings
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            // odom publisher and subscriber
            odom_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/baby_michael/pose", qos, std::bind(&LocalNavigation::odom_callback, this, std::placeholders::_1));

            // drone state subscribers
            vehicle_attitude_sub_ = node.create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos, std::bind(&LocalNavigation::vehicle_attitude_callback, this, std::placeholders::_1));
            vehicle_local_position_sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&LocalNavigation::vehicle_local_position_callback, this, std::placeholders::_1));
            vehicle_acceleration_sub_ = node.create_subscription<px4_msgs::msg::VehicleAcceleration>("/fmu/out/vehicle_acceleration", qos, std::bind(&LocalNavigation::vehicle_acceleration_callback, this, std::placeholders::_1));
            vehicle_angular_velocity_sub_ = node.create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos, std::bind(&LocalNavigation::vehicle_angular_velocity_callback, this, std::placeholders::_1));

            // ext setpoint sub
            ext_setpoint_sub_ = node.create_subscription<geometry_msgs::msg::Point>("/rpi/in/ext_cmdPoint", qos, std::bind(&LocalNavigation::ext_setpoint_callback, this, std::placeholders::_1));

            // state publishers
            pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/rpi/out/vehicle_pose", 10);
            twist_pub_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("/rpi/out/vehicle_velocity", 10);
            accel_pub_ = node.create_publisher<geometry_msgs::msg::AccelStamped>("/rpi/out/vehicle_acceleration", 10);

            // use px4_ros2 lib to control the drone too

            auto state_timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                publish_state(pos_local, quat_local, avel_local, accel_local);			
            };

            state_timer_ = node.create_wall_timer(10ms, state_timer_callback);

            position_timer = node.create_wall_timer(10ms, [this] {updateLocalPosition();});

            RCLCPP_INFO(node.get_logger(), "example_local_navigation_node running!");
        }

    private:
        // needed optitrack stuff
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
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

        rclcpp::TimerBase::SharedPtr state_timer_;
        rclcpp::TimerBase::SharedPtr position_timer;

        std::atomic<uint64_t> timestamp_;

        void odom_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg);

        void updateLocalPosition();

        void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr & msg);
        void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr & msg);
        void vehicle_acceleration_callback(const px4_msgs::msg::VehicleAcceleration::UniquePtr & msg);
        void vehicle_angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::UniquePtr & msg);

        void ext_setpoint_callback(const geometry_msgs::msg::Point::UniquePtr & msg);

        void publish_state(const Eigen::Vector3d poslocal, const Eigen::Quaterniond quatlocal, const Eigen::Vector3d avellocal, const Eigen::Vector3d accellocal);

        Eigen::Vector3d pos_ros;
        Eigen::Quaterniond quat_ros;
        Eigen::Vector3d pos_px4;
        Eigen::Quaterniond quat_px4;

        Eigen::Vector3d pos_cmd_;

        Eigen::Vector3d pos_local;
        Eigen::Quaterniond quat_local;
        Eigen::Vector3d avel_local;
        Eigen::Vector3d accel_local;
};

namespace RPI {
	// class for offboard control which starts a ROS2 node
	class rpictrls : public rclcpp::Node {
	public:
		rpictrls() : Node("rpi_ctrls")
		{
            _interface = std::make_unique<LocalNavigation>(*this);

            if (!_interface->doRegister()) {
                throw std::runtime_error("Registration failed");
            }
		}

	private:
		std::unique_ptr<LocalNavigation> _interface;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RPI::rpictrls>());
	rclcpp::shutdown();
	return 0;
}

#endif


