"""
This is a program that implements a Closed-loop robot control (holonomic Pure Pursuit Algorithm). It ueses
a ROS2 node that commands the Rosmasterx3 using a holonomic approach of the Pure Pursuuit feedback controller.
The robot will follow a square path with a side length of 3 meters.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ClosedLoopControlNode(Node):
    # Initialize the node and set up the publisher and timer
    def __init__(self):
        super().__init__('closed_loop_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.path = [(0, 0), (3, 0), (3, 3), (0, 3)]
        self.current_target_index = 0
        self.kp = 1.0  # Proportional gain
        self.robot_position = (0, 0)  # Initial robot position
    
    # Function to update the robot's position
    def control_loop(self):
        if self.current_target_index >= len(self.path):
            self.get_logger().info('Path completed!')
            return
        
        target = self.path[self.current_target_index]
        error_x = target[0] - self.robot_position[0]
        error_y = target[1] - self.robot_position[1]
        
        # Simple proportional control
        control_signal_x = self.kp * error_x
        control_signal_y = self.kp * error_y
        
        # Create and publish the Twist message
        twist = Twist()
        twist.linear.x = control_signal_x
        twist.linear.y = control_signal_y
        self.publisher.publish(twist)
        
        # Check if the robot is close enough to the target
        if (error_x ** 2 + error_y ** 2) ** 0.5 < 0.1:  # Threshold distance
            self.current_target_index += 1