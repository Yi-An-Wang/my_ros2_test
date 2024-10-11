import rclpy
from rclpy.node import Node
from interface_package.msg import MotionCMD
import numpy as np
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self) -> str:
        super().__init__(node_name="GBM_keyboard")
        self.publisher = self.create_publisher(MotionCMD, "/cmd/GBM_motion", 10)
        self.get_logger().info("Use W/X/A/D/ to move the GBM robot, O/P to rotate the GBM, and E/Q/C/Z to turn orientation. Press T to quit.")

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
        msg = MotionCMD()
        msg.move_type = 2
        key = self.get_key().upper()

        if key == 'W':
            msg.vx_cmd= 100.0
        elif key == 'X':
            msg.vx_cmd = -100.0
        elif key == 'E':
            msg.vx_cmd = 100.0
            msg.omega_cmd = -np.pi/15
        elif key == 'Q':
            msg.vx_cmd = 100.0
            msg.omega_cmd = np.pi/15
        elif key == 'Z':
            msg.vx_cmd = -100.0
            msg.omega_cmd = -np.pi/15
        elif key == 'C':
            msg.vx_cmd = -100.0
            msg.omega_cmd = np.pi/15
        elif key == 'A':
            msg.vy_cmd = 100.0
        elif key =='D':
            msg.vy_cmd= -100.0
        elif key == 'O':
            msg.omega_cmd = np.pi/6
        elif key == 'P':
            msg.omega_cmd = -np.pi/6
        elif key == 'S':
            msg.vx_cmd = 0.0
            msg.vy_cmd = 0.0
            msg.omega_cmd = 0.0
        elif key == 'T':
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