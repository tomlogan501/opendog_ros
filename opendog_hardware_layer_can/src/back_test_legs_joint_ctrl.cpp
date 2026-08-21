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

class BackTestLegsJointCtrl : public rclcpp::Node
{
public:
    BackTestLegsJointCtrl()
        : Node("back_test_legs_joint_ctrl_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "opendog_jointController/commands", 30,
                std::bind(&BackTestLegsJointCtrl::topic_callback, this, _1));

            publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "back_test_legs_controller/commands", 30);
        }

private:
    static constexpr size_t BR_ULEG_INDEX = 7;
    static constexpr size_t BR_LLEG_INDEX = 8;
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
        joint_angles.data.push_back(double(msg_rx->data[BL_ULEG_INDEX] * M_PI / 180));
        joint_angles.data.push_back(double(msg_rx->data[BL_LLEG_INDEX] * M_PI / 180));
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
    rclcpp::spin(std::make_shared<BackTestLegsJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
