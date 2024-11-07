// cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && colcon build --executor sequential --packages-select px4_handler && source install/setup.bash

// MicroXRCEAgent udp4 -p 8888

// cd ~/Dev/PX4-Autopilot && make px4_sitl gazebo-classic

// cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch px4_offboard offboard_velocity_control.launch.py
// cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch drone_nav navigation.launch.py

// cd ~/Dev && ./QGroundControl.AppImage

// cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 run px4_handler executor 

// ros2 topic pub -1 /signal std_msgs/msg/String "{data: 'A'}"
// ros2 topic pub -1 /signal std_msgs/msg/String "{data: 'B'}"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

// Constants for PX4 modes and commands
constexpr uint8_t VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1 << 7; // 128
constexpr uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
constexpr uint16_t VEHICLE_CMD_NAV_GUIDED_ENABLE = 92;
constexpr uint16_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;

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
    DroneStateManager(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode):
        ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode, "drone_state_manager"),
        _node(node)
    {
        arming_timer_ = nullptr;
        offboard_setpoint_counter_ = 0;

        // Initialize variables
        vehicle_armed_ = false;
        vehicle_landed_ = false;
        current_position_x_ = 0.0f;
        current_position_y_ = 0.0f;
        current_altitude_ = 0.0f;
        takeoff_altitude_ = -5.0f;

        // Subscribe to the "signal" topic
        _signal_subscription = _node.create_subscription<std_msgs::msg::String>(
            "signal", 10,
            std::bind(&DroneStateManager::signalCallback, this, std::placeholders::_1));

        // Publishers for offboard control
        offboard_control_mode_publisher_ = _node.create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/offboard_control_mode/in", 10);
        trajectory_setpoint_publisher_ = _node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/trajectory_setpoint/in", 10);
        vehicle_command_publisher_ = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/vehicle_command/in", 10);

        RCLCPP_INFO(_node.get_logger(), "DroneStateManager activated, waiting for signals 'A'/'B'");

        vehicle_status_subscriber_ = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out", 10,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                vehicle_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
            });

        vehicle_local_position_subscriber_ = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/vehicle_local_position/out", 10,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                current_altitude_ = msg->z;
                current_position_x_ = msg->x;
                current_position_y_ = msg->y;
            });

        vehicle_land_detected_subscriber_ = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/vehicle_land_detected/out", 10,
            [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
                vehicle_landed_ = msg->landed;
            });
    }

    enum class State {
        Reset,
        Idle,
        Arming,
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
            RCLCPP_INFO(_node.get_logger(), "Signal 'A' received, initiating arming sequence");
            runState(State::Arming);
        } else if (msg->data == "B") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'B' received, initiating landing");
            runState(State::Landing);
        } else {
            RCLCPP_WARN(_node.get_logger(), "Unknown signal received: '%s'", msg->data.c_str());
        }
    }

    void runState(State state) {
        switch (state) {
            case State::Arming:
                RCLCPP_INFO(_node.get_logger(), "Beginning arming sequence");

                offboard_setpoint_counter_ = 0; // Reset the counter
                arming_timer_ = _node.create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        publishOffboardControlMode();
                        publishTrajectorySetpoint();

                        if (offboard_setpoint_counter_ == 10) {
                            // Enable guided mode
                            publishVehicleCommand(VEHICLE_CMD_NAV_GUIDED_ENABLE, 1.0f);

                            // Set offboard mode
                            publishVehicleCommand(
                                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                                VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                PX4_CUSTOM_MAIN_MODE_OFFBOARD);

                            // Arm the vehicle
                            arm();
                        }

                        // After a few iterations, check if the vehicle is armed
                        if (offboard_setpoint_counter_ > 20) {
                            if (isVehicleArmed()) {
                                RCLCPP_INFO(_node.get_logger(), "Vehicle is armed, proceeding to takeoff");
                                arming_timer_->cancel();
                                runState(State::TakingOff);
                            }
                        }
                        offboard_setpoint_counter_++;
                    });
                break;

            case State::TakingOff:
                RCLCPP_INFO(_node.get_logger(), "Initiating takeoff");
                // Implement takeoff logic here
                // For example, set a higher altitude in the trajectory setpoint
                takeoff_altitude_ = -5.0f; // Desired altitude in meters (negative down)
                takeoff_timer_ = _node.create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        publishOffboardControlMode();
                        publishTakeoffSetpoint();

                        // Check if the vehicle has reached the desired altitude
                        if (hasReachedAltitude(takeoff_altitude_)) {
                            RCLCPP_INFO(_node.get_logger(), "Takeoff successful, now hovering");
                            takeoff_timer_->cancel();
                            runState(State::Hovering);
                        }
                    });
                break;

            case State::Hovering:
                RCLCPP_INFO(_node.get_logger(), "Hovering at current position");
                // Continue publishing the current setpoint to maintain position
                hover_timer_ = _node.create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        publishOffboardControlMode();
                        publishCurrentSetpoint();
                    });
                break;

            case State::Landing:
                RCLCPP_INFO(_node.get_logger(), "Initiating landing");
                // Implement landing logic here
                landing_timer_ = _node.create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() {
                        publishOffboardControlMode();
                        publishLandingSetpoint();

                        // Check if the vehicle has landed
                        if (hasLanded()) {
                            RCLCPP_INFO(_node.get_logger(), "Landing complete");
                            landing_timer_->cancel();
                            disarm();
                            runState(State::WaitUntilDisarmed);
                        }
                    });
                break;

            case State::WaitUntilDisarmed:
                RCLCPP_INFO(_node.get_logger(), "Waiting until vehicle is disarmed");
                wait_timer_ = _node.create_wall_timer(
                    std::chrono::milliseconds(500),
                    [this]() {
                        if (!isVehicleArmed()) {
                            RCLCPP_INFO(_node.get_logger(), "Vehicle is disarmed, operation complete");
                            wait_timer_->cancel();
                        }
                    });
                break;

            default:
                RCLCPP_WARN(_node.get_logger(), "Unknown state");
                break;
        }
    }

    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publishTrajectorySetpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {0.0f, 0.0f, 0.0f}; // Initial position
        msg.yaw = NAN;
        msg.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publishTakeoffSetpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {0.0f, 0.0f, takeoff_altitude_};
        msg.yaw = NAN;
        msg.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publishCurrentSetpoint() {
        // Continue to publish the current position to maintain hover
        // This function would get the current position from vehicle data
    }

    void publishLandingSetpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {0.0f, 0.0f, 0.0f}; // Landing at ground level
        msg.yaw = NAN;
        msg.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        static_cast<float>(VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED), // param1: base mode
        static_cast<float>(PX4_CUSTOM_MAIN_MODE_OFFBOARD));        // param2: custom main mode


    void arm() {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    }

    void disarm() {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
    }

    bool isVehicleArmed() {
        // Implement a method to check if the vehicle is armed
        // This might involve subscribing to the vehicle_status topic
        return vehicle_armed_;
    }

    bool hasReachedAltitude(float altitude) {
        // Implement logic to check if the vehicle has reached the desired altitude
        // This might involve subscribing to a position topic
        return true; // Placeholder
    }

    bool hasLanded() {
        // Implement logic to check if the vehicle has landed
        // This might involve checking altitude or landed state
        return true; // Placeholder
    }

    // Class members
    rclcpp::Node & _node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _signal_subscription;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscriber_;

    rclcpp::TimerBase::SharedPtr arming_timer_;
    rclcpp::TimerBase::SharedPtr takeoff_timer_;
    rclcpp::TimerBase::SharedPtr hover_timer_;
    rclcpp::TimerBase::SharedPtr landing_timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;

    int offboard_setpoint_counter_;
    float takeoff_altitude_;
    bool vehicle_armed_;

    bool vehicle_armed_;
    bool vehicle_landed_;
    float current_position_x_;
    float current_position_y_;
    float current_altitude_;
    float takeoff_altitude_;
};

int main(int argc, char ** argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the rclcpp node
    auto node = std::make_shared<rclcpp::Node>("drone_manager_node");

    // Create instances of your modes and the state manager
    SimpleModeBase simple_mode(*node);
    DroneStateManager state_manager(*node, simple_mode);

    // Optionally activate the mode and state manager
    simple_mode.activate();
    state_manager.activate();

    // Start the ROS 2 event loop
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}