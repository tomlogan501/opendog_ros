#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// using namespace std::chrono_literals;
using std::placeholders::_1;

// Real-hardware equivalent of opendog_gazebo_joint_ctrl_node.
//
// IK_node publishes 12 joint angles, in degrees, on
// "opendog_jointController/commands", in the order:
//   FR_hip, FR_uleg, FR_lleg, FL_hip, FL_uleg, FL_lleg,
//   BR_hip, BR_uleg, BR_lleg, BL_hip, BL_uleg, BL_lleg
//
// The real hardware layer's controller (position_controllers/
// JointGroupPositionController, named "all_joints_controller" in
// opendog_hardware_layer_can/config/odrive_can_controllers.yaml)
// expects radians on "all_joints_controller/commands", with the
// SAME joint order (confirmed directly from that config file) --
// so no reordering is needed here, only unit conversion.
//
// IMPORTANT: unlike the sim version, sending an out-of-range value
// to real hardware can cause real damage. This node does NOT clamp
// values -- any real safety limiting must happen either in the
// real ODrive's own configuration, or be added here explicitly
// once the real joint limits are confirmed against the physical
// robot (not just copied from the simulated URDF).
class OpendogHardwareJointCtrl : public rclcpp::Node
{
public:
    OpendogHardwareJointCtrl()
        : Node("opendog_hardware_joint_ctrl_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "opendog_jointController/commands", 30,
                std::bind(&OpendogHardwareJointCtrl::topic_callback, this, _1));

            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "all_joints_controller/commands", 30);
        }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg_rx) const
    {
        auto joint_angles = std_msgs::msg::Float64MultiArray();
        for (float ang : msg_rx->data) {
            joint_angles.data.push_back(double(ang * M_PI / 180));
        }
        publisher_->publish(joint_angles);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpendogHardwareJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
