import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self) -> str:
        super().__init__(node_name="keyboard_control")
        self.publisher = self.create_publisher(Twist, "/obstacle/velocity", 10)
        self.get_logger().info("Use W/A/X/S/D to control the ball. Press Q to quit.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_setting = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_setting)
        return key
    
    def send_velocity(self) -> None:
        msg = Twist()
        key = self.get_key().upper()

        if key == 'W':
            msg.linear.y = 20.0
        elif key == 'X':
            msg.linear.y = -20.0
        elif key == 'A':
            msg.linear.x = -20.0
        elif key =='D':
            msg.linear.x = 20.0
        elif key == 'S':
            msg.linear.x = 0.0
            msg.linear.y = 0.0
        elif key == 'Q':
            self.get_logger().info("Quitting...")
            sys.exit(0)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardController()

    while rclpy.ok():
        node.send_velocity()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()