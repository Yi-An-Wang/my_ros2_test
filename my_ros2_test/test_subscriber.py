import rclpy
from rclpy.node import Node
from interface_package.msg import MotionCMD
import tkinter as tk
import threading

class test_subscriber(Node):

    def __init__(self):
        super().__init__(node_name="test_subscriber")
        self.get_logger().info("I don't want to spin~~")
        self.subscriber = self.create_subscription(MotionCMD,"/cmd/GBM_motion",self.callback, 10)
        self.rec_points = [100, 100, 0, 100, 0, 0, 100, 0]

    def callback(self, msg:MotionCMD):
        self.get_logger().info("I still alive!")
        self.rec_points[0] += 10
        self.rec_points[2] += 10
        self.rec_points[4] += 10
        self.rec_points[6] += 10

# def ros_loop(args=None):
#     rclpy.init(args=args)
#     node = test_subscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# def gui_loop():
#     root = tk.Tk()
#     label = tk.Label(root, text='LA LA LA')
#     label.pack()
#     root.mainloop()

# def update(root: tk.Tk, canvas: tk.Canvas, rec,rec_points) -> None:
#     canvas.coords(rec, *rec_points)
#     root.after(10, lambda: update(root, canvas, rec, rec_points))

def main(args=None):
    rclpy.init(args=args)

    node = test_subscriber()
    rec_points = [100, 100, 0, 100, 0, 0, 100, 0]
    def ros_loop():
        rclpy.spin(node)

    def gui_loop():

        def update(root_: tk.Tk, canvas_: tk.Canvas, rec_) -> None:
            canvas_.coords(rec, *node.rec_points)
            root_.after(10,lambda: update(root_, canvas_, rec_))

        root = tk.Tk()
        canvas = tk.Canvas(root, width=600, height=600)
        canvas.pack()
        rec = canvas.create_polygon(rec_points, fill='blue')
        update(root, canvas, rec)
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