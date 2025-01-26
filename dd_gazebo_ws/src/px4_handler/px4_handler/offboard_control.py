#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import String


# cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && colcon build --executor sequential --packages-select px4_handler && source install/setup.bash

# MicroXRCEAgent udp4 -p 8888

# cd ~/Dev/PX4-Autopilot && make px4_sitl gazebo-classic

# cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch px4_offboard offboard_velocity_control.launch.py
# cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 launch drone_nav navigation.launch.py

# cd ~/Dev && ./QGroundControl.AppImage

# cd ~/Dev/DD_Nav_WS/dd_gazebo_ws/ && source install/setup.bash && ros2 run px4_handler offboard_control 

# ros2 topic pub -1 /signal std_msgs/msg/String "{data: 'A'}"
# ros2 topic pub -1 /signal std_msgs/msg/String "{data: 'B'}"


class Goal:
    NONE = 0
    TAKEOFF = 1
    LAND = 2

    def __init__(self):
        self.data = 0
        self.state = Goal.NONE


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Handle the signal
        self.signal_subscriber = self.create_subscription(
            String, '/signal', self.signal_callback, qos_profile)

        self.goal = Goal()

        # Initialize variables
        # self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def signal_callback(self, msg):
        """Callback function for the signal topic subscriber."""
        if msg.data == 'A':
            self.get_logger().info("A: Takeoff signal received!!")
            self.goal.data = 0
            self.goal.state = Goal.TAKEOFF
        elif msg.data == 'B':
            self.get_logger().info("B: Land signal received!!")
            self.goal.data = 0
            self.goal.state = Goal.LAND
        else:
            self.get_logger().info("Invalid signal received")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.goal.state == Goal.TAKEOFF:
            if self.goal.data == 10:
                self.get_logger().info(f"Takeoff initiated at height {self.vehicle_local_position.z}")
                self.engage_offboard_mode()
                self.arm()
            if self.goal.data < 11: # 11 -> do once more
                self.goal.data += 1
        elif self.goal.state == Goal.LAND:
            if self.goal.data == 0:
                self.goal.data = 1
                # self.vehicle_local_position.z <= self.takeoff_height:
                self.get_logger().info(f"Land initiated at height {self.vehicle_local_position.z}")
                self.land()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)


def main(args=None) -> None:
    print('Starting offboard control node...')
    
    rclpy.init(args=args)
    
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
