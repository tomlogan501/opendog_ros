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

// Isolated BACK-OF-ROBOT test: BL + BR legs (6 joints) across ODrive
// 4 (hips), 2 (BL leg) and 5 (BR leg). ODrive 4 has a known
// unresolved CAN wiring fault (see hardware layer notes) -- watch
// candump/can0 error counters closely on activation.
//
// IK_node always publishes all 12 joint angles, in degrees, in the
// standard order:
//   [0]FR_hip [1]FR_uleg [2]FR_lleg [3]FL_hip [4]FL_uleg [5]FL_lleg
//   [6]BR_hip [7]BR_uleg [8]BR_lleg [9]BL_hip [10]BL_uleg [11]BL_lleg
//
// Republished to back_test_controller/commands in the order the
// controller declares its joints:
//   ["BL_hip_joint","BL_uleg_joint","BL_lleg_joint",
//    "BR_hip_joint","BR_uleg_joint","BR_lleg_joint"]
//
// IMPORTANT: this does NOT clamp values to any safe range. Only
// BR_uleg/BR_lleg have confirmed gear_ratio/zero_offset -- BL_hip,
// BR_hip, BL_uleg, BL_lleg are all UNVERIFIED for direction. Watch
// closely on first activation.
class BackTestJointCtrl : public rclcpp::Node
{
public:
    BackTestJointCtrl()
        : Node("back_test_joint_ctrl_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "opendog_jointController/commands", 30,
                std::bind(&BackTestJointCtrl::topic_callback, this, _1));

            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "back_test_controller/commands", 30);
        }

private:
    static constexpr size_t BR_HIP_INDEX  = 6;
    static constexpr size_t BR_ULEG_INDEX = 7;
    static constexpr size_t BR_LLEG_INDEX = 8;
    static constexpr size_t BL_HIP_INDEX  = 9;
    static constexpr size_t BL_ULEG_INDEX = 10;
    static constexpr size_t BL_LLEG_INDEX = 11;

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg_rx) const
    {
        if (msg_rx->data.size() <= BL_LLEG_INDEX) {
            RCLCPP_WARN(this->get_logger(),
                "Received joint command array with only %zu values, expected 12 -- ignoring",
                msg_rx->data.size());
            return;
        }

        auto joint_angles = std_msgs::msg::Float64MultiArray();
        // order matches back_test_controller's declared joints:
        // BL_hip, BL_uleg, BL_lleg, BR_hip, BR_uleg, BR_lleg
        joint_angles.data.push_back(double(msg_rx->data[BL_HIP_INDEX]  * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BL_ULEG_INDEX] * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BL_LLEG_INDEX] * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BR_HIP_INDEX]  * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BR_ULEG_INDEX] * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BR_LLEG_INDEX] * M_PI / 180));
        publisher_->publish(joint_angles);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackTestJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
