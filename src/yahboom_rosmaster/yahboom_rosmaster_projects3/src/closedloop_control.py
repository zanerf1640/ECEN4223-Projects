"""
This is a program that implements a Closed-loop robot control (holonomic Pure Pursuit Algorithm). It ueses
a ROS2 node that commands the Rosmasterx3 using a holonomic approach of the Pure Pursuuit feedback controller.
The robot will follow a square path with a side length of 3 meters.
"""
# Import necessary libraries and ROS2 message types
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class ClosedLoopControlNode(Node):
    # Initialize the node and set up the publisher and timer
    def __init__(self):
        super().__init__('closed_loop_control_node')
        # Subscribe to the odometry or pose topic to get the robot's current position and orientation
        self.declare_parameter('pose_source', 'odom')
        pose_source = self.get_parameter('pose_source').get_parameter_value().string_value
        if pose_source == 'odom':
            self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        else:
            self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        # Publisher to send velocity commands to the robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer to control the loop at a fixed rate (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        # Map the square with 3m sides
        self.path = [(0, 0), (3, 0), (3, 3), (0, 3)]

        # Initialize parameters
        self.current_target_index = 0       # Index of the current target waypoint
        self.kp = 1.0                       # Proportional gain
        self.robot_x = 0.0                  # Initial robot x position
        self.robot_y = 0.0                  # Initial robot y position
        self.robot_position = (0.0, 0.0)    # Initial robot position as a tuple
        self.robot_yaw = 0.0                # Initial robot orientation
        self.Ld = 0.5                       # Lookahead distance
        self.s_progress = 0.0               # Progress along the path
        self.cycle_count = 0                # Cycle count for debugging
        self.max_cycles = 3                 # Maximum number of cycles to prevent infinite loops


    # Callback function to update robot's state from Odometry message
    def odom_callback(self, msg):
        # Update robot's position and orientation from Odometry message
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_position = (self.robot_x, self.robot_y)
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    

    # Callback function to update robot's state from PoseStamped message
    def pose_callback(self, msg):
        # Update robot's position and orientation from PoseStamped message
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_position = (self.robot_x, self.robot_y)
        # Extract yaw from quaternion
        orientation_q = msg.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)


    # Update the progress of the robot along the path
    def update_progress(self):
        # Calculate the closest point and update progress
        p1 = self.path[self.current_target_index % len(self.path)]
        p2 = self.path[(self.current_target_index + 1) % len(self.path)]
        # Project robot onto segment
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        seg_len = math.dist(p1, p2)
        t = ((self.robot_x - p1[0]) * dx + (self.robot_y - p1[1]) * dy) / (seg_len ** 2)
        t = max(0, min(1, t))  # Clamp to segment
        return t, p1, p2, seg_len


    # Compute lookahead point for Pure Pursuit
    def compute_lookahead_point(self):
        t, p1, p2, seg_len = self.update_progress()
        remaining = (1 - t) * seg_len
        # If lookahead is within the current segment, calculate the lookahead point
        if remaining >= self.Ld:
            ratio = (t * seg_len + self.Ld) / seg_len
            lookahead_x = p1[0] + ratio * (p2[0] - p1[0])
            lookahead_y = p1[1] + ratio * (p2[1] - p1[1])
        else:
            # If lookahead goes beyond current segment, calculate the next point on the path
            next_index = (self.current_target_index + 1) % len(self.path)
            p3 = self.path[(next_index + 1) % len(self.path)]
            extra = self.Ld - remaining
            next_seg_len = math.dist(p2, p3)
            ratio = extra / next_seg_len
            lookahead_x = p2[0] + ratio * (p3[0] - p2[0])
            lookahead_y = p2[1] + ratio * (p3[1] - p2[1])
            # Switch to next segment if close to current target
            if remaining < 0.1:
                self.current_target_index += 1
        return lookahead_x, lookahead_y


    # Rotate would-frame error to robot frame
    def rotate_to_robot_frame(self, error_x, error_y):
        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)
        robot_error_x = error_x * cos_yaw - error_y * sin_yaw
        robot_error_y = error_x * sin_yaw + error_y * cos_yaw
        return robot_error_x, robot_error_y
    

    # Command vx, vy proportional to the error in the robot frame
    def compute_control_command(self, robot_error_x, robot_error_y):
        control_signal_x = self.kp * robot_error_x
        control_signal_y = self.kp * robot_error_y
        return control_signal_x, control_signal_y
    

    # Function to update the robot's position
    def control_loop(self):
        # Stop if all cycles are completed
        if self.cycle_count >= self.max_cycles:
            self.publisher.publish(Twist())  # Stop the robot
            return None
        # Compute the lookahead point and calculate control commands
        lookahead = self.compute_lookahead_point()
        # Calculate the error in the world frame and rotate it to the robot frame
        error_x = lookahead[0] - self.robot_position[0]
        error_y = lookahead[1] - self.robot_position[1]
        rx, ry = self.rotate_to_robot_frame(error_x, error_y)
        vx, vy = self.compute_control_command(rx, ry)
        # Publish the velocity command to the robot
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        self.publisher.publish(twist)

        # Waypoint switching logic
        if self.current_target_index >= len(self.path):
            self.cycle_count += 1
            if self.cycle_count >= self.max_cycles:
                self.get_logger().info('Completed maximum cycles, stopping control loop.')
                self.publisher.publish(Twist())  # Stop the robot
                return
            self.current_target_index = 0

# Main function    
def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()