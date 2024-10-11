#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from interface_package.msg import PoseAndWheels

class GBM_animation(Node):

    def __init__(self, fig: plt.figure, ax: plt.axis, center_point: list[float], width: float,
                 height: float, L, init_angle: float = 0.0, facecolor: list[float] = [0.5, 0.5, 0.5]) -> None:
        '''
        please enter length unit in centimeter(cm)
        '''
        self.center = center_point
        self.angle = init_angle
        self.width = width
        self.height = height
        self.L = L
        self.wheel_angles = [0.0, 0.0]
        xy = [self.center[0]-1/2*self.width, self.center[1]-1/2*self.height]
        self.rec = Rectangle(xy, width, height, angle=self.angle, rotation_point='center', edgecolor='black', facecolor=facecolor)
        # print(self.center)
        # print(self.angle)
        # self.trajectory_x = [self.center[0]]
        # self.trajectory_y = [self.center[1]]

        wheel_length = 35
        wheel_width = 14
        self.wheel_init = np.array([[-wheel_length/2, 0-wheel_width/2], [-wheel_length/2, 0-wheel_width/2]]).T
        wheel_point = self.wheel_init + np.array([[self.L/2, -self.L/2],[0, 0]])
        th = self.angle/180*np.pi
        Rotate = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        wheel_point = Rotate @ wheel_point + np.array([[self.center[0], self.center[0]], [self.center[1], self.center[1]]])
        # print(wheel_point[:,0].flatten().tolist())
        self.wheel_f = Rectangle(wheel_point[:,0].flatten().tolist(), wheel_length, wheel_width, angle=self.angle,
                                 edgecolor='black', facecolor=[0.2, 0.2, 0.2])
        self.wheel_r = Rectangle(wheel_point[:,1].flatten().tolist(), wheel_length, wheel_width, angle=self.angle,
                                 edgecolor='black', facecolor=[0.2, 0.2, 0.2])
        
        self.fig = fig
        self.ax = ax
        self.fig_background = fig.canvas.copy_from_bbox(ax.bbox)
        
        self.ax.add_patch(self.rec)
        self.ax.add_patch(self.wheel_f)
        self.ax.add_patch(self.wheel_r)
        plt.draw()
        self.fig.canvas.flush_events()
        
        # Node section
        super().__init__(node_name="GBM_plot")
        self.get_logger().info(f"I am a GBM with width={self.width}, height={self.height}, wheel_distance={self.L} !!!")
        self.subscriber = self.create_subscription(PoseAndWheels, "/plot/GBM_state", self.subscriber_callback, 10)
        self.timer = self.create_timer(0.05, self.new_pose)

    def subscriber_callback(self, msg: PoseAndWheels) -> None:
        self.center = [msg.pose_x, msg.pose_y]
        self.angle = msg.pose_theta/np.pi*180
        self.wheel_angles = [msg.wheel_theta_f/np.pi*180, msg.wheel_theta_r/np.pi*180]
        # self.get_logger().info(f"wheel angles = {self.wheel_angles}")
        self.get_logger().info(f"X = {msg.pose_x}")
        self.get_logger().info(f"Y = {msg.pose_y}")
        self.get_logger().info(f"theta = {msg.pose_theta}")

    def translate(self) -> None:
        xy = [self.center[0]-1/2*self.width, self.center[1]-1/2*self.height]
        self.rec.set_xy(xy)

    def rotate(self) -> None:
        self.rec.set_angle(self.angle)

    def wheel_move(self) -> None:
        th1, th2 = self.wheel_angles[0]/180*np.pi, self.wheel_angles[1]/180*np.pi
        Rotate1 = np.array([[np.cos(th1), -np.sin(th1)], [np.sin(th1), np.cos(th1)]])
        Rotate2 = np.array([[np.cos(th2), -np.sin(th2)], [np.sin(th2), np.cos(th2)]])
        wheel_point = np.zeros(shape=(2,2))
        wheel_point[:,0:1] = Rotate1 @ self.wheel_init[:,0:1] + np.array([[self.L/2],[0]])
        wheel_point[:,1:2] = Rotate2 @ self.wheel_init[:,1:2] + np.array([[-self.L/2],[0]])
        th = self.angle/180*np.pi
        Rotate = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        wheel_point = Rotate @ wheel_point + np.array([[self.center[0], self.center[0]], [self.center[1], self.center[1]]])
        
        self.wheel_f.set_xy(wheel_point[:,0].flatten().tolist())
        self.wheel_r.set_xy(wheel_point[:,1].flatten().tolist())

        self.wheel_f.set_angle(self.angle+self.wheel_angles[0])
        self.wheel_r.set_angle(self.angle+self.wheel_angles[1])

    def new_pose(self) -> None:
        self.translate()
        self.rotate()
        self.wheel_move()
        self.fig.canvas.restore_region(self.fig_background)
        self.ax.draw_artist(self.rec)
        self.ax.draw_artist(self.wheel_f)
        self.ax.draw_artist(self.wheel_r)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)

    plt.ion()

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.set_xlim(-500, 500)
    ax1.set_ylim(-500, 500)
    ax1.set_aspect('equal')
    ax1.grid(True)

    node = GBM_animation(fig, ax1, [0,0], 150, 80, 80, 0)
    rclpy.spin(node)

    plt.ioff()

    node.destroy_node()
    rclpy.shutdown()

def test_main(args=None):
    rclpy.init(args=args)
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.set_xlim(-500, 500)
    ax1.set_ylim(-500, 500)
    ax1.set_aspect('equal')
    ax1.grid(True)
    node = GBM_animation(fig, ax1, [0,0], 150, 80, 80, 0)
    node.center = [5,10]
    node.new_pose()
    rclpy.spin(node)
    plt.ioff()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    test_main()