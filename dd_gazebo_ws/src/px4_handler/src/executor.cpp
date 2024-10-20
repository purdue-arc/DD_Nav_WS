#include "rclcpp/rclcpp.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/components/mode.hpp"
#include "std_msgs/msg/string.hpp"


// colcon build --executor sequential --packages-select px4_handler
// ros2 run px4_handler executor 
// ros2 topic pub /signal std_msgs/msg/String "{data: 'A'}"
// ros2 launch px4_offboard offboard_velocity_control.launch.py
// ros2 launch drone_nav navigation.launch.py


class SimpleModeBase : public px4_ros2::ModeBase {
public:
    explicit SimpleModeBase(rclcpp::Node & node):
        ModeBase(node, Settings{"Custom Mode"}),
        _node(node)
    {
        RCLCPP_INFO(_node.get_logger(), "SimpleModeBase initialized");
    }

    void onActivate() override {
        // Logic when mode is activated
        RCLCPP_INFO(_node.get_logger(), "SimpleModeBase activated");
    }

    void onDeactivate() override {
        // Logic when mode is deactivated
        RCLCPP_INFO(_node.get_logger(), "SimpleModeBase deactivated");
    }
private:
    rclcpp::Node & _node;
};

class DroneStateManager : public px4_ros2::ModeExecutorBase {
public:
    // Constructor
    DroneStateManager(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode):
        ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode, "drone_state_manager"),
      _node(node)  // Initialize _node in the initializer list
    {
        // Subscribe to the "signal" topic
        _signal_subscription = _node.create_subscription<std_msgs::msg::String>(
            "signal", 10,
            std::bind(&DroneStateManager::signalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(_node.get_logger(), "DroneStateManager activated, waiting for signals 'A' or 'B'");
    }

    enum class State {
        Reset,
        Idle,
        TakingOff,
        Hovering,
        Landing,
        WaitUntilDisarmed,
    };

    void onActivate() override {
        // Node activation logic
        RCLCPP_INFO(_node.get_logger(), "DroneStateManager node activated");
    }

    void onDeactivate(DeactivateReason _reason) override {
        // Node deactivation logic
        (void)_reason;
        RCLCPP_INFO(_node.get_logger(), "DroneStateManager node deactivated");
    }

private:
    void signalCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "A") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'A' received, initiating takeoff and hover");
            runState(State::TakingOff, px4_ros2::Result::Success);
        } else if (msg->data == "B") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'B' received, initiating landing");
            runState(State::Landing, px4_ros2::Result::Success);
        } else {
            RCLCPP_WARN(_node.get_logger(), "Unknown signal received: '%s'", msg->data.c_str());
        }
    }

    void runState(State state, px4_ros2::Result previous_result) {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
            return;
        }

        switch (state) {
            case State::Reset:
                // Reset logic if needed
                break;

            case State::TakingOff:
                takeoff([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Takeoff successful, now hovering");
                        // Optionally transition to Hovering state
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Takeoff failed: %s", resultToString(result));
                    }
                });
                break;

            case State::Landing:
                land([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Landing initiated, waiting until disarmed");
                        runState(State::WaitUntilDisarmed, px4_ros2::Result::Success);
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Landing failed: %s", resultToString(result));
                    }
                });
                break;

            case State::WaitUntilDisarmed:
                waitUntilDisarmed([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Drone disarmed, operation complete");
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Disarm failed: %s", resultToString(result));
                    }
                });
                break;

            default:
                RCLCPP_WARN(_node.get_logger(), "Unknown state");
                break;
        }
    }

    // Class members
    rclcpp::Node & _node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _signal_subscription;
};


int main(int argc, char ** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the rclcpp::Node
    auto node = std::make_shared<rclcpp::Node>("drone_state_manager_node");

    // Create a SimpleModeBase object (required by DroneStateManager)
    auto mode = std::make_shared<SimpleModeBase>(*node);

    // Instantiate DroneStateManager with the node and mode objects
    auto drone_state_manager = std::make_shared<DroneStateManager>(*node, *mode);

    // Activate the DroneStateManager node
    drone_state_manager->onActivate();

    // Spin the node (this will block and handle callbacks)
    rclcpp::spin(node);

    // Shutdown ROS 2 system once the spinning is done
    rclcpp::shutdown();
    return 0;
}