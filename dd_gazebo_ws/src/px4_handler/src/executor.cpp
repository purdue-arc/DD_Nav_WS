#include "rclcpp/rclcpp.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/components/mode.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleModeBase : public px4_ros2::ModeBase {
public:
    explicit SimpleModeBase(rclcpp::Node & node) : ModeBase(node, Settings{"Custom Mode"}) {
        // Initialize manual control and rates setpoint
        // _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
        // _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    }

    void onActivate() override {
        // Logic when mode is activated
        // RCLCPP_INFO(_node.get_logger(), "SimpleModeBase activated");
    }

    void onDeactivate() override {
        // Logic when mode is deactivated
        // RCLCPP_INFO(_node.get_logger(), "SimpleModeBase deactivated");
    }

    void updateSetpoint(const rclcpp::Duration & dt) {
        // RCLCPP_INFO(_node.get_logger(), "SimpleModeBase update setpoint");
        // Update thrust and rates setpoints based on manual control input
        // Eigen::Vector3f thrust_sp{0.0F, 0.0F, -_manual_control_input->throttle()};
        // Eigen::Vector3f rates_sp{
        //     _manual_control_input->roll() * 150.0F * M_PI / 180.0F,
        //     -_manual_control_input->pitch() * 150.0F * M_PI / 180.0F,
        //     _manual_control_input->yaw() * 100.0F * M_PI / 180.0F
        // };

        // _rates_setpoint->update(rates_sp, thrust_sp);
    }

private:
    // std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;
    // std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
};

class DroneStateManager : public px4_ros2::ModeExecutorBase {
public:
    // Constructor
    DroneStateManager(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
    : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode, "drone_state_manager"),
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
    auto mode = std::make_shared<SimpleModeBase>();

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