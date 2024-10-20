#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/state_change_publisher.hpp"
#include "px4_ros2/mode_executor_base.hpp"
#include "px4_ros2/goto_setpoint_type.hpp"
#include <eigen3/Eigen/Dense>
#include "std_msgs/msg/string.hpp"

using namespace px4_msgs::msg;

class MyModeExecutor : public px4_ros2::ModeExecutorBase {
public:
    MyModeExecutor(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode) : 
        ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
        _node(node) {
        // Subscribe to the "signal_A" and "signal_B" topic
        _signal_subscription = _node.create_subscription<std_msgs::msg::String>(
            "signal", 10,
            std::bind(&MyModeExecutor::signalCallback, this, std::placeholders::_1));
    }

    enum class State {
        Reset,
        TakingOff,
        Hovering,
        Landing,
        WaitUntilDisarmed,
    };

    void onActivate() override {
        // Default activation doesn't start the takeoff
        RCLCPP_INFO(_node.get_logger(), "MyModeExecutor activated, waiting for signal 'A' or 'B'");
    }

    void onDeactivate(DeactivateReason reason) override {}

    // Callback for when a signal is received
    void signalCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "A") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'A' received, switching to takeoff and hover mode");
            runState(State::TakingOff, px4_ros2::Result::Success);
        } else if (msg->data == "B") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'B' received, switching to landing mode");
            runState(State::Landing, px4_ros2::Result::Success);
        }
    }

    void runState(State state, px4_ros2::Result previous_result) {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
            return;
        }

        switch (state) {
            case State::Reset:
                break;

            case State::TakingOff:
                // Takeoff and then transition to hovering state
                takeoff([this](px4_ros2::Result result) { runState(State::Hovering, result); });
                break;

            case State::Hovering:
                // Hovering in place after takeoff
                RCLCPP_INFO(_node.get_logger(), "Drone is hovering...");
                break;

            case State::Landing:
                // Land the drone when signal "B" is received
                land([this](px4_ros2::Result result) { runState(State::WaitUntilDisarmed, result); });
                break;

            case State::WaitUntilDisarmed:
                waitUntilDisarmed([this](px4_ros2::Result result) {
                    RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
                });
                break;
        }
    }

private:
    rclcpp::Node & _node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _signal_subscription;
};

