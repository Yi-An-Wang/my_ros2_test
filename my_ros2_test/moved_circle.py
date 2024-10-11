import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Pose

class moved_circle(Node):

    def __init__(self, fig: plt.figure, ax: plt.axis, radius: float, position: list[float], color: list[float] = [0.1, 0.1, 0.1]) -> None:
        self.fig = fig
        self.ax = ax
        self.fig_background = fig.canvas.copy_from_bbox(ax.bbox)

        self.r = radius
        self.pose = position
        self.velocity_vector = [0, 0]
        
        self.time_ii = 0

        # ploting section
        self.Circle = plt.Circle(self.pose, self.r, edgecolor = 'black', facecolor = color)
        self.ax.add_patch(self.Circle)
        plt.draw()
        self.fig.canvas.flush_events()

        # Node section
        super().__init__(node_name="moved_circle")
        self.get_logger().info("I am a ball!!!")
        self.pose_pub = self.create_publisher(Pose, "/circle/pose", 10)
        self.timer = self.create_timer(0.01, self.move)

    def move(self) -> None:
        self.time_ii = self.time_ii + 1
        self.new_pose(v_cmd=[50, 50], dt=0.01)
        if self.time_ii % 10 == 0:
            self.draw_current_pose()
        msg = Pose()
        msg.position.x = self.pose[0]
        msg.position.y = self.pose[1]
        self.pose_pub.publish(msg=msg)

    def new_pose(self, v_cmd: list[float], dt: float) -> None:
        self.velocity_vector = v_cmd
        position = [self.pose[0] + v_cmd[0] * dt, self.pose[1] + v_cmd[1] * dt]
        self.pose = position

    def draw_current_pose(self) -> None:
        self.Circle.set_center(self.pose)
        # plt.draw()
        self.fig.canvas.restore_region(self.fig_background)
        self.ax.draw_artist(self.Circle)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()
        

def main(args=None):
    rclpy.init(args=args)

    plt.ion()

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.set_xlim(-300, 300)
    ax1.set_ylim(-300, 300)
    ax1.set_aspect('equal')
    ax1.grid(True)

    node = moved_circle(fig, ax1, 5, [0, 0])
    rclpy.spin(node)

    plt.ioff()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()