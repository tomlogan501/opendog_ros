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

using std::placeholders::_1;

// Isolated-test variant of opendog_hardware_joint_ctrl_node.
//
// IK_node always publishes all 12 joint angles, in degrees, in the
// standard order:
//   [0]FR_hip [1]FR_uleg [2]FR_lleg [3]FL_hip [4]FL_uleg [5]FL_lleg
//   [6]BR_hip [7]BR_uleg [8]BR_lleg [9]BL_hip [10]BL_uleg [11]BL_lleg
//
// For this isolated 2-joint hardware test, only indices 1 (FR_uleg)
// and 2 (FR_lleg) are extracted and republished, in that order, to
// fr_leg_test_controller/commands.
//
// IMPORTANT: this does NOT clamp values to any safe range. The leg
// must be physically unloaded / restrained during testing.
class FrLegTestJointCtrl : public rclcpp::Node
{
public:
    FrLegTestJointCtrl()
        : Node("fr_leg_test_joint_ctrl_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "opendog_jointController/commands", 30,
                std::bind(&FrLegTestJointCtrl::topic_callback, this, _1));

            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "fr_leg_test_controller/commands", 30);
        }

private:
    static constexpr size_t FR_ULEG_INDEX = 1;
    static constexpr size_t FR_LLEG_INDEX = 2;

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg_rx) const
    {
        if (msg_rx->data.size() <= FR_LLEG_INDEX) {
            RCLCPP_WARN(this->get_logger(),
                "Received joint command array with only %zu values, expected 12 -- ignoring",
                msg_rx->data.size());
            return;
        }

        auto joint_angles = std_msgs::msg::Float64MultiArray();
        joint_angles.data.push_back(double(msg_rx->data[FR_ULEG_INDEX] * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[FR_LLEG_INDEX] * M_PI / 180));
        publisher_->publish(joint_angles);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrLegTestJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
