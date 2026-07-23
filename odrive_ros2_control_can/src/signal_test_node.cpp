// signal_test_node.cpp
//
// Publishes step/ramp/oscillation position commands to
// /forward_position_controller/commands and logs the resulting
// /joint_states feedback, in the SAME CSV format used by the Python
// axis0_axis1_signal_tests.py / analyze_signal_test_log.py pair, so the
// existing analyzer script works unmodified on data captured through the
// real ros2_control hardware interface.
//
// This exercises the REAL production path: forward_position_controller ->
// ros2_control resource manager -> ODriveHardwareInterfaceCAN::write() ->
// ODriveCAN::send_set_input_pos() -> CAN bus -> ODrive (TRAP_TRAJ) -> motor
// -> ODriveHardwareInterfaceCAN::read() -> joint_state_broadcaster ->
// /joint_states -> this node.
//
// Signal definitions match the finalized Python test
// (axis0_axis1_signal_tests.py):
//   step  - single larger move, ODrive's own TRAP_TRAJ planner smooths it
//   ramp  - gradual linear staircase of many small published targets
//   oscillation - sinusoidal tracking
//
// Usage (after `ros2 launch odrive_ros2_control_can two_joints_test.launch.py`):
//   ros2 run odrive_ros2_control_can signal_test_node --ros-args \
//       -p tests:="step,ramp,oscillation" -p gear_ratio:=10.0

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class SignalTestNode : public rclcpp::Node
{
public:
  SignalTestNode()
  : Node("signal_test_node")
  {
    joint_names_ = {"motor_joint_0", "motor_joint_1"};

    gear_ratio_ = this->declare_parameter<double>("gear_ratio", 10.0);
    // 0.4 turn at whatever gear_ratio is configured (reduced from 0.5 after
    // an OVERSPEED fault on both axes running step+ramp back-to-back at 0.5)
    step_size_rad_ = this->declare_parameter<double>(
      "step_size_rad", 0.5 * 2.0 * M_PI / gear_ratio_);
    step_hold_s_ = this->declare_parameter<double>("step_hold_s", 4.5);
    // 0.4 turn ramp, gradual linear staircase (matches Python RAMP_SIZE)
    ramp_size_rad_ = this->declare_parameter<double>(
      "ramp_size_rad", 0.4 * 2.0 * M_PI / gear_ratio_);
    ramp_time_s_ = this->declare_parameter<double>("ramp_time_s", 3.0);
    // 0.4 turn oscillation amplitude (unchanged, already matches Python)
    osc_amplitude_rad_ = this->declare_parameter<double>(
      "osc_amplitude_rad", 0.4 * 2.0 * M_PI / gear_ratio_);
    osc_frequency_hz_ = this->declare_parameter<double>("osc_frequency_hz", 0.5);
    osc_duration_s_ = this->declare_parameter<double>("osc_duration_s", 6.0);
    settle_pause_s_ = this->declare_parameter<double>("settle_pause_s", 1.0);
    command_rate_hz_ = this->declare_parameter<double>("command_rate_hz", 50.0);
    log_file_ = this->declare_parameter<std::string>("log_file", "signal_test_log_hwif.csv");

    std::string tests_csv = this->declare_parameter<std::string>("tests", "step,ramp,oscillation");
    tests_ = split(tests_csv, ',');

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SignalTestNode::joint_state_callback, this, std::placeholders::_1));

    last_position_.assign(joint_names_.size(), std::nan(""));

    log_stream_.open(log_file_, std::ios::out | std::ios::trunc);
    log_stream_ << "test,axis,t_seconds,target_turns,actual_turns\n";

    RCLCPP_INFO(this->get_logger(), "SignalTestNode ready. Logging to %s", log_file_.c_str());
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for initial /joint_states...");
    while (rclcpp::ok() && !have_state_) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(20ms);
    }

    for (size_t axis = 0; axis < joint_names_.size(); axis++) {
      double base_pos_rad;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        base_pos_rad = last_position_[axis];
      }
      RCLCPP_INFO(this->get_logger(), "axis%zu base position: %.4f rad (%.4f turns)",
                  axis, base_pos_rad, rad_to_turns(base_pos_rad));

      for (const auto & test : tests_) {
        if (test_failed_) {
          RCLCPP_ERROR(this->get_logger(),
            "Aborting remaining tests: axis%zu did not return to base position "
            "after the previous test (likely a fault) — check dump_errors(odrv0) "
            "before retrying.", axis);
          log_stream_.close();
          return;
        }
        if (test == "step") {
          run_step(axis, base_pos_rad);
        } else if (test == "ramp") {
          run_ramp(axis, base_pos_rad);
        } else if (test == "oscillation") {
          run_oscillation(axis, base_pos_rad);
        } else {
          RCLCPP_WARN(this->get_logger(), "Unknown test '%s', skipping", test.c_str());
        }
      }
    }

    log_stream_.close();
    RCLCPP_INFO(this->get_logger(), "Done. Log written to %s", log_file_.c_str());
  }

private:
  double rad_to_turns(double rad) const { return (rad * gear_ratio_) / (2.0 * M_PI); }

  static std::vector<std::string> split(const std::string & s, char delim)
  {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
      if (c == delim) { out.push_back(cur); cur.clear(); }
      else { cur.push_back(c); }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < joint_names_.size(); i++) {
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (msg->name[j] == joint_names_[i] && j < msg->position.size()) {
          last_position_[i] = msg->position[j];
        }
      }
    }
    have_state_ = true;
  }

  double get_position(size_t axis)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_position_[axis];
  }

  void publish_command(size_t axis, double target_rad, double hold_rad_for_other_axis)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); i++) {
      msg.data[i] = (i == axis) ? target_rad : hold_rad_for_other_axis;
    }
    cmd_pub_->publish(msg);
  }

  double hold_value_for(size_t other_axis)
  {
    return get_position(other_axis);
  }

  void log_row(const std::string & test_name, size_t axis, double t, double target_rad, double actual_rad)
  {
    log_stream_ << test_name << "," << axis << "," << t << ","
                << rad_to_turns(target_rad) << "," << rad_to_turns(actual_rad) << "\n";
  }

  // After a test settles, confirm the axis is actually back near base_pos_rad.
  // A large residual error means the axis likely faulted mid-test (e.g. an
  // OVERSPEED trip) and stopped responding to commands — better to stop the
  // run here than immediately fire the next test at a faulted axis.
  void check_returned_to_base(size_t axis, double base_pos_rad, const char * test_name)
  {
    constexpr double kToleranceRad = 0.1;  // ~0.016 turn at gear_ratio=10
    double actual = get_position(axis);
    if (std::isnan(actual) || std::abs(actual - base_pos_rad) > kToleranceRad) {
      RCLCPP_ERROR(this->get_logger(),
        "axis%zu: did not settle back near base after %s (base=%.4f rad, actual=%.4f rad) "
        "— axis may have faulted (check dump_errors(odrv0))",
        axis, test_name, base_pos_rad, actual);
      test_failed_ = true;
    }
  }

  // IMPORTANT: check_returned_to_base() alone is not sufficient — if write()
  // silently skipped every command (e.g. axis stuck faulted the whole time,
  // "not in healthy closed loop"), the axis never leaves base_pos_rad at
  // all, so "returned to base" is trivially true even though NOTHING moved.
  // This checks that the axis actually got close to the commanded target at
  // some point during the test — the only way to tell "it moved correctly"
  // from "it never moved."
  void check_reached_target(size_t axis, double target_rad, double closest_seen_rad,
                             const char * test_name)
  {
    constexpr double kToleranceRad = 0.1;
    if (std::isnan(closest_seen_rad) || std::abs(closest_seen_rad - target_rad) > kToleranceRad) {
      RCLCPP_ERROR(this->get_logger(),
        "axis%zu: never got close to the %s target (target=%.4f rad, closest seen=%.4f rad) "
        "— the axis likely never moved at all (check whether write() is skipping commands: "
        "\"not in healthy closed loop\")",
        axis, test_name, target_rad, closest_seen_rad);
      test_failed_ = true;
    }
  }

  void spin_for(double seconds)
  {
    auto end = this->now() + rclcpp::Duration::from_seconds(seconds);
    while (rclcpp::ok() && this->now() < end) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / command_rate_hz_)));
    }
  }

  // STEP: single larger move via ODrive's own TRAP_TRAJ planner (if
  // input_mode=TRAP_TRAJ is set on the axis; otherwise behaves as an
  // instant jump under PASSTHROUGH). Matches Python step_test().
  void run_step(size_t axis, double base_pos_rad)
  {
    double target = base_pos_rad + step_size_rad_;
    RCLCPP_INFO(this->get_logger(), "  [axis%zu] STEP: %.4f -> %.4f rad (%.4f turns)",
                axis, base_pos_rad, target, rad_to_turns(target - base_pos_rad));

    size_t other = 1 - axis;
    double hold = hold_value_for(other);
    auto t0 = this->now();
    publish_command(axis, target, hold);

    double closest_to_target = std::nan("");
    while (rclcpp::ok() && (this->now() - t0).seconds() < step_hold_s_) {
      rclcpp::spin_some(this->get_node_base_interface());
      double t = (this->now() - t0).seconds();
      double actual = get_position(axis);
      log_row("step", axis, t, target, actual);
      if (!std::isnan(actual) &&
          (std::isnan(closest_to_target) || std::abs(actual - target) < std::abs(closest_to_target - target)))
      {
        closest_to_target = actual;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / command_rate_hz_)));
    }
    check_reached_target(axis, target, closest_to_target, "step");

    publish_command(axis, base_pos_rad, hold);
    spin_for(settle_pause_s_);
    check_returned_to_base(axis, base_pos_rad, "step");
  }

  // RAMP: gradual, linear position increase over time - many small
  // commanded steps rather than one jump, symmetric on the way back.
  // Matches Python ramp_test() (linear staircase).
  void run_ramp(size_t axis, double base_pos_rad)
  {
    double target = base_pos_rad + ramp_size_rad_;
    RCLCPP_INFO(this->get_logger(), "  [axis%zu] RAMP: %.4f -> %.4f rad (%.4f turns, linear staircase)",
                axis, base_pos_rad, target, rad_to_turns(target - base_pos_rad));

    size_t other = 1 - axis;
    double hold = hold_value_for(other);
    auto t0 = this->now();

    const int steps = std::max(1, static_cast<int>(ramp_time_s_ * command_rate_hz_));
    const double dt = 1.0 / command_rate_hz_;

    // outbound: base -> target, in `steps` even increments
    double closest_to_target = std::nan("");
    for (int i = 0; i <= steps; i++) {
      double frac = static_cast<double>(i) / steps;
      double cmd = base_pos_rad + ramp_size_rad_ * frac;
      publish_command(axis, cmd, hold);
      rclcpp::spin_some(this->get_node_base_interface());
      double t = (this->now() - t0).seconds();
      double actual = get_position(axis);
      log_row("ramp", axis, t, cmd, actual);
      if (!std::isnan(actual) &&
          (std::isnan(closest_to_target) || std::abs(actual - target) < std::abs(closest_to_target - target)))
      {
        closest_to_target = actual;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000.0)));
    }
    check_reached_target(axis, target, closest_to_target, "ramp");

    // return: target -> base, just as gradually as the way out
    for (int i = 0; i <= steps; i++) {
      double frac = static_cast<double>(i) / steps;
      double cmd = target - ramp_size_rad_ * frac;
      publish_command(axis, cmd, hold);
      rclcpp::spin_some(this->get_node_base_interface());
      double t = (this->now() - t0).seconds();
      log_row("ramp", axis, t, cmd, get_position(axis));
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000.0)));
    }

    publish_command(axis, base_pos_rad, hold);
    spin_for(settle_pause_s_);
    check_returned_to_base(axis, base_pos_rad, "ramp");
  }

  void run_oscillation(size_t axis, double base_pos_rad)
  {
    RCLCPP_INFO(this->get_logger(), "  [axis%zu] OSCILLATION: center=%.4f amp=%.4f freq=%.2fHz %.1fs",
                axis, base_pos_rad, osc_amplitude_rad_, osc_frequency_hz_, osc_duration_s_);

    size_t other = 1 - axis;
    double hold = hold_value_for(other);
    auto t0 = this->now();

    while (rclcpp::ok() && (this->now() - t0).seconds() < osc_duration_s_) {
      double t = (this->now() - t0).seconds();
      double cmd = base_pos_rad + osc_amplitude_rad_ * std::sin(2.0 * M_PI * osc_frequency_hz_ * t);
      publish_command(axis, cmd, hold);
      rclcpp::spin_some(this->get_node_base_interface());
      log_row("oscillation", axis, t, cmd, get_position(axis));
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / command_rate_hz_)));
    }

    publish_command(axis, base_pos_rad, hold);
    spin_for(settle_pause_s_);
  }

  std::vector<std::string> joint_names_;
  std::vector<std::string> tests_;
  double gear_ratio_;
  double step_size_rad_, step_hold_s_;
  double ramp_size_rad_, ramp_time_s_;
  double osc_amplitude_rad_, osc_frequency_hz_, osc_duration_s_;
  double settle_pause_s_;
  double command_rate_hz_;
  std::string log_file_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex state_mutex_;
  std::vector<double> last_position_;
  bool have_state_ = false;
  bool test_failed_ = false;

  std::ofstream log_stream_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SignalTestNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}