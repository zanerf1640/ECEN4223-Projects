import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class OpenClawWatchdog(Node):
    def __init__(self):
        super().__init__('openclaw_watchdog_node')
        
        self.subscription = self.create_subscription(
            Twist,
            '/openclaw/cmd_vel',
            self.listener_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.last_msg_time = time.time()
        self.timeout_duration = 1.0 
        self.is_moving = False  # Flag to prevent spamming stop commands 
        
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info('Watchdog Active. Listening on /openclaw/cmd_vel...')

    def listener_callback(self, msg):
        self.last_msg_time = time.time()
        self.is_moving = True 
        self.publisher.publish(msg)

    def watchdog_callback(self):
        # Only trigger stop if we were moving and have now timed out [cite: 174, 184]
        if self.is_moving and (time.time() - self.last_msg_time) > self.timeout_duration:
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist() # Initializes all values to 0.0 [cite: 27]
        self.publisher.publish(stop_msg)
        self.is_moving = False 
        self.get_logger().info('Timeout reached: Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = OpenClawWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()