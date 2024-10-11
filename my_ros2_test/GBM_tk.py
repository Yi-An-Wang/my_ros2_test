import rclpy
from rclpy.node import Node
from interface_package.msg import PoseAndWheels
import math
import numpy as np
import tkinter as tk
import threading

def make_rec_points(pose: list[float], width: float, height: float) -> list[float]:
    P = np.array([[width/2, -width/2, -width/2, width/2],
                  [height/2, height/2, -height/2, -height/2]])
    rotate_M = np.array([[math.cos(pose[2]), -math.sin(pose[2])],
                         [math.sin(pose[2]), math.cos(pose[2])]])
    new_P = rotate_M @ P
    return new_P + np.array([[pose[0]], [pose[1]]])

def coordinate_change(canvas: tk.Canvas, points: list[float], proportion: float = 1) -> list[float]:
    """
    please enter the points as numpy.array(2*n) form
    """
    changed_ps = np.zeros(shape=np.shape(points))
    C_w = canvas.winfo_width()
    C_h = canvas.winfo_height()
    changed_ps[1:2,:] = points[1:2,:] * -proportion
    changed_ps[0:1,:] = points[0:1,:] * proportion
    changed_ps[0:1,:] = changed_ps[0:1,:] + C_w/2
    changed_ps[1:2,:] = changed_ps[1:2,:] + C_h/2
    return changed_ps.T.flatten().tolist()

class GBM_tk(Node):

    def __init__(self, GBM_pose: list[float], wheel_thetas: list[float], width: float, height: float, L: float) -> None:
        # system parameters.  do not chage them after init
        self.width = width
        self.height = height
        self.L = L
        self.wheel_width = 35
        self.wheel_height = 14

        # system state
        self.GBM_pose = GBM_pose #[x, y, psi]
        self.wheel_thetas = wheel_thetas #[theta_f, theta_r]
        # system points
        self.GBM_points, self.wheel_f_points, self.wheel_r_points = self.make_GBM_points()

        # Node section
        super().__init__(node_name="GBM_tk")
        self.get_logger().info("I don't know whether i can work or not.")
        self.subscriber = self.create_subscription(PoseAndWheels, "/plot/GBM_state", self.callback_change_points, 10)

    def make_GBM_points(self) -> list[float]:
        wheel_f_pose = [self.GBM_pose[0] + self.L/2*math.cos(self.GBM_pose[2]), self.GBM_pose[1] + self.L/2*math.sin(self.GBM_pose[2]), self.GBM_pose[2] + self.wheel_thetas[0]]
        wheel_r_pose = [self.GBM_pose[0] - self.L/2*math.cos(self.GBM_pose[2]), self.GBM_pose[1] - self.L/2*math.sin(self.GBM_pose[2]), self.GBM_pose[2] + self.wheel_thetas[1]]
        GBM_points = make_rec_points(self.GBM_pose, self.width, self.height)
        wheel_f_points = make_rec_points(wheel_f_pose, self.wheel_width, self.wheel_height)
        wheel_r_points = make_rec_points(wheel_r_pose, self.wheel_width, self.wheel_height)
        return GBM_points, wheel_f_points, wheel_r_points
    
    def update_GBM_tk(self,tk_canvas: tk.Canvas, GBM_rec, wheel_f_rec, wheel_r_rec, proportion) -> None:
        new_GBM_coord = coordinate_change(tk_canvas, self.GBM_points, proportion)
        print(new_GBM_coord)
        new_wheel_f_coord = coordinate_change(tk_canvas, self.wheel_f_points, proportion)
        new_wheel_r_coord = coordinate_change(tk_canvas, self.wheel_r_points, proportion)
        tk_canvas.coords(GBM_rec, *new_GBM_coord)
        tk_canvas.coords(wheel_f_rec, *new_wheel_f_coord)
        tk_canvas.coords(wheel_r_rec, *new_wheel_r_coord)

    def callback_change_points(self, msg: PoseAndWheels) -> None:
        self.get_logger().info(f"Pose = [{msg.pose_x}, {msg.pose_y}, {msg.pose_theta}]")
        # changes system state
        self.GBM_pose = [msg.pose_x, msg.pose_y, msg.pose_theta]
        self.wheel_thetas = [msg.wheel_theta_f, msg.wheel_theta_r]
        # change system points
        self.GBM_points, self.wheel_f_points, self.wheel_r_points = self.make_GBM_points()

def main(args=None):
    rclpy.init(args=args)

    node = GBM_tk([0, 0, 0], [0, 0], 150, 80, 80)
    
    def ros_loop():
        rclpy.spin(node)

    def gui_loop():
        root = tk.Tk()
        canvas = tk.Canvas(root, width=1000, height=1000, background="#fef5e7")
        canvas.pack()
        root.update()
        proportion = 0.5
        GBM_points = coordinate_change(canvas, node.GBM_points, proportion)
        wheel_f_points = coordinate_change(canvas, node.wheel_f_points, proportion)
        wheel_r_points = coordinate_change(canvas, node.wheel_r_points, proportion)
        GBM_rec = canvas.create_polygon(GBM_points, fill="#85c1e9", outline='#283747', width=0.5)
        wheel_f_rec = canvas.create_polygon(wheel_f_points, fill="#212121", outline='#283747', width=0.5)
        wheel_r_rec = canvas.create_polygon(wheel_r_points, fill="#212121", outline='#283747', width=0.5)

        line_points = np.array([[-500, 500, 0, 0], [0, 0, 500, -500]])
        ch_line_points = coordinate_change(canvas, line_points, 1)
        canvas.create_line(*ch_line_points[0:4], width=2, fill="#95a5a6")
        canvas.create_line(*ch_line_points[4:8], width=2, fill="#95a5a6")

        def update():
            node.update_GBM_tk(canvas, GBM_rec, wheel_f_rec, wheel_r_rec, proportion)
            root.after(10, update)
        update()
        root.mainloop()

    ros_thread = threading.Thread(target=ros_loop)
    gui_thread = threading.Thread(target=gui_loop)

    ros_thread.start()
    gui_thread.start()

    ros_thread.join()
    gui_thread.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()