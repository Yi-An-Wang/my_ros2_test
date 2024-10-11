import rclpy 
from rclpy.node import Node
import numpy as np
import math
import scipy.integrate as integrate

from interface_package.msg import MotionCMD, PoseAndWheels

class wheel_system:
    def __init__(self, radius: float, robot_Mass: float, gear_ratio: float,
                T_motor_max: float, w_no_load: float, stearing_rate: float) -> None:
        self.r = radius
        self.M = robot_Mass
        self.n = gear_ratio
        self.T_max = T_motor_max
        self.w_no_load = w_no_load
        self.K_w = self.n/self.r
        # print(f"K_w: {K_w}")
        self.K_T = 2*self.n/(self.M*self.r)
        # print(f"K_T: {K_T}")
        self.a_p = -2 * self.T_max * self.n / (self.r * self.M)
        # print(f"a_p{self.a_p}")
        self.v_w = 0
        self.a_w = 0
        self.spin_degree = 0 

        self.theta_dot_max = stearing_rate
        self.theta_dot = 0
        self.theta = 0
    
    def wheel_actuate(self, v_goal: float, angle_goal: float, dt: float) -> float:
        # spining wheel
        a_max = self.a_max_TN_curve()
        # print(f"current_dv: {(v_goal - self.v_w) / dt}")
        # print(f"a_max: {a_max}")
        if (v_goal - self.v_w) / dt > a_max:
            a_i = a_max
            # print("foward chasing")
        elif (v_goal - self.v_w) / dt < self.a_p:
            a_i = self.a_p
            # print("backward chasing")
        else:
            a_i = (v_goal - self.v_w) / dt
            # print("arived !")
        self.a_w = a_i
        self.v_w = self.v_w + self.a_w * dt

        # rotating wheel angle
        if (angle_goal - self.theta) / dt > self.theta_dot_max:
            self.theta_dot = self.theta_dot_max
        elif (angle_goal - self.theta) / dt < -self.theta_dot_max:
            self.theta_dot = -self.theta_dot_max
        else:
            self.theta_dot = (angle_goal - self.theta) / dt
        self.theta = self.theta + self.theta_dot * dt

        return self.a_w * 100, self.v_w * 100, self.theta_dot , self.theta 

    def a_max_TN_curve(self) -> float:
        a_max = self.K_T*self.T_max*(1-self.K_w*self.v_w/self.w_no_load)
        return a_max

class GBM_robot(Node):

    def __init__(self, long: float, width: float, L: float, GBM_pose: list[float], color=[0.6, 0.7, 0.6]) -> None:
        self.D, self.W, self.L = long, width, L
        self.r = (1/2)*(self.D**2 + self.W**2)**0.5
        self.H_matrix = np.array([[1, 0, 0],
                                  [0, 1, L/2],
                                  [1, 0, 0],
                                  [0, 1, -L/2]])
        self.HTH_matrix = np.array([[1/2, 0, 1/2, 0],
                                    [0, 1/2, 0, 1/2],
                                    [0, 1/L, 0, -1/L]])
        self.pose = GBM_pose
        rotate_M = np.array([[np.cos(GBM_pose[2]), -np.sin(GBM_pose[2])],
                             [np.sin(GBM_pose[2]), np.cos(GBM_pose[2])]])
        self.wheel_vtheta_state = np.zeros(shape=(4,1)) # [v_f, theta_f, v_r, theta_r]^T
        self.wheel_xy_state = np.zeros(shape=(4,1)) # [v_xf, v_yf, v_xr, v_yr]^T
        self.local_motion = np.zeros(shape=(3,1)) # [v_x, v_y, omega]^T

        # system parameters 
        # all Units is in SI(meter, kilogram, Newton, ...)
        wheel_radius = 0.0508
        # motor_max_torque = 3280/1000*9.81/100
        # motor_w_no_load = 8500*2*np.pi/60
        motor_max_torque = 1800/1000*9.81/100
        motor_w_no_load = 5700*2*np.pi/60
        gear_ratio = 8.14 * 3.3
        robot_Mass = 10
        stearing_rate = 5700*2*np.pi/(12.8*10.9)

        self.front_wheel_sys = wheel_system(wheel_radius, robot_Mass, gear_ratio, motor_max_torque, motor_w_no_load, stearing_rate)
        self.rear_wheel_sys = wheel_system(wheel_radius, robot_Mass, gear_ratio, motor_max_torque, motor_w_no_load, stearing_rate)

        # points for detecting collision
        self.d_points = np.zeros(shape=(2,5))
        self.d_points[:,0:1] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[self.D/2], [self.W/2]])
        self.d_points[:,1:2] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[self.D/2], [-self.W/2]])
        self.d_points[:,2:3] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[-self.D/2], [-self.W/2]])
        self.d_points[:,3:4] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[-self.D/2], [self.W/2]])
        self.d_points[:,4:5] = np.array([[self.pose[0]], [self.pose[1]]])

        # Node section
        self.motion_cmd = np.zeros(shape=(3,1))
        super().__init__(node_name="GBMsim")
        self.get_logger().info(f"I am a GBM system with {self.D} * {self.W}, wheel_distance={self.L} !!!")
        self.motion_subscriber = self.create_subscription(MotionCMD, "/cmd/GBM_motion",self.cmd_subscribe_callback,10)
        self.pose_publisher = self.create_publisher(PoseAndWheels,"/plot/GBM_state",10)
        self.timer = self.create_timer(0.01, self.move_GBM)

    def global2local(self, global_motion: list[float]) -> list[float]:
        assert isinstance(global_motion, np.ndarray), "輸入必須是 numpy 陣列"
        T_matrix=np.array([[np.cos(self.pose[2]), np.sin(self.pose[2]), 0],
                           [-np.sin(self.pose[2]), np.cos(self.pose[2]), 0],
                           [0, 0, 1]])
        local_motion = T_matrix @ global_motion
        return local_motion
    
    def local2global(self) -> list[float]:
        T_matrix=np.array([[np.cos(self.pose[2]), -np.sin(self.pose[2]), 0],
                           [np.sin(self.pose[2]), np.cos(self.pose[2]), 0],
                           [0, 0, 1]])
        global_motion = T_matrix @ self.local_motion
        return global_motion

    def odometry_and_local_motion(self, dt, a_f, a_r, theta_dot_f, theta_dot_r) -> list[float]:
        # print(dt, a_f, a_r, theta_dot_f, theta_dot_r)
        def V_x(t: float) -> float:
            Vx = 1/2*((a_f*t+self.wheel_vtheta_state[0,0])*math.cos(self.wheel_vtheta_state[1,0]+theta_dot_f*t)+(a_r*t+self.wheel_vtheta_state[2,0])*math.cos(self.wheel_vtheta_state[3,0]+theta_dot_r*t))
            return Vx
        
        def V_y(t: float) -> float:
            Vy = 1/2*((a_f*t+self.wheel_vtheta_state[0,0])*math.sin(self.wheel_vtheta_state[1,0]+theta_dot_f*t)+(a_r*t+self.wheel_vtheta_state[2,0])*math.sin(self.wheel_vtheta_state[3,0]+theta_dot_r*t))
            return Vy
        
        def omega(t: float) -> float:
            spining_speed = 1/self.L*((a_f*t+self.wheel_vtheta_state[0,0])*math.sin(self.wheel_vtheta_state[1,0]+theta_dot_f*t)\
                                     -(a_r*t+self.wheel_vtheta_state[2,0])*math.sin(self.wheel_vtheta_state[3,0]+theta_dot_r*t))
            return spining_speed
        
        def orientation(t: float) -> float:
            
            def int_w_f(tau: float) -> float:
                return a_f*math.sin(self.wheel_vtheta_state[1,0]+theta_dot_f*tau)/(theta_dot_f**2*self.L)\
                        - (a_f*tau+self.wheel_vtheta_state[0,0])*math.cos(self.wheel_vtheta_state[1,0]+theta_dot_f*tau)/(theta_dot_f*self.L)
            
            def int_w_r(tau: float) -> float:
                return - a_r*math.sin(self.wheel_vtheta_state[3,0]+theta_dot_r*tau)/(theta_dot_r**2*self.L)\
                        + (a_r*tau+self.wheel_vtheta_state[2,0])*math.cos(self.wheel_vtheta_state[3,0]+theta_dot_r*tau)/(theta_dot_r*self.L)
            
            def zero_theta_dot_f(tau: float) ->  float:
                return (1/self.L)*(math.sin(self.wheel_vtheta_state[1,0])*(1/2*a_f*tau**2+self.wheel_vtheta_state[0,0]*tau))
            
            def zero_theta_dot_r(tau: float) -> float:
                return (-1/self.L)*(math.sin(self.wheel_vtheta_state[3,0])*(1/2*a_r*tau**2+self.wheel_vtheta_state[2,0]*tau))
            
            if theta_dot_f == 0.0 and theta_dot_r != 0.0:
                psi = zero_theta_dot_f(t) + (int_w_r(t) - int_w_r(0)) + self.pose[2]
            elif theta_dot_f != 0.0 and theta_dot_r == 0.0:
                psi = (int_w_f(t) - int_w_f(0)) + zero_theta_dot_r(t) + self.pose[2]
            elif theta_dot_f == 0.0 and theta_dot_r == 0.0:
                psi = zero_theta_dot_f(t) + zero_theta_dot_r(t) + self.pose[2]
            else:
                psi = (int_w_f(t) - int_w_f(0)) + (int_w_r(t) - int_w_r(0)) + self.pose[2]

            # psi = int_w(t) - int_w(0) + self.pose[2]
            return psi
        
        def int_x(t: float) -> float:
            # X = V_x(t) * math.cos(orientation(t)) - V_y(t) * math.sin(orientation(t))
            X = V_x(t) * math.cos(orientation(t))\
                - V_y(t) * math.sin(orientation(t))
            return X
        
        def int_y(t: float) -> float:
            # Y = V_x(t) * math.sin(orientation(t)) + V_y(t) * math.cos(orientation(t))
            Y = V_x(t) * math.sin(orientation(t))\
                + V_y(t) * math.cos(orientation(t))
            return Y
        
        # x_odometry = integrate.quad(lambda t: int_x(t), 0, dt,  full_output=1)
        x_odometry = integrate.quad(lambda t: int_x(t), 0, dt)
        # print(f"x_int: {x_odometry}")
        y_odometry = integrate.quad(lambda t: int_y(t), 0, dt)
        psi = orientation(dt)
        # print(f"psi: {psi}")
        # print(f"int_w: {self.pose[2] + (integrate.quad(lambda t: omega(t), 0, dt))[0]}")
        pose = [x_odometry[0] + self.pose[0], y_odometry[0] + self.pose[1], psi]

        local_motion = np.array([[V_x(dt)],
                                 [V_y(dt)],
                                 [omega(dt)]])
        
        return pose, local_motion

    def actuate(self, motion_cmd: list[float], dt: float) -> None:
        assert isinstance(motion_cmd, np.ndarray), "輸入必須是 numpy 陣列"
        goal_wheel_xy_state = (self.H_matrix @ motion_cmd) * 0.01 # unit change from cm to m
        goal_theta_f = np.arctan2(goal_wheel_xy_state[1,0], goal_wheel_xy_state[0,0])
        goal_theta_r = np.arctan2(goal_wheel_xy_state[3,0], goal_wheel_xy_state[2,0])
        goal_v_f = (goal_wheel_xy_state[0,0]**2+goal_wheel_xy_state[1,0]**2)**0.5
        goal_v_r = (goal_wheel_xy_state[2,0]**2+goal_wheel_xy_state[3,0]**2)**0.5

        a_f, v_f, theta_f_dot, theta_f = self.front_wheel_sys.wheel_actuate(goal_v_f, goal_theta_f, dt)
        a_r, v_r, theta_r_dot, theta_r = self.rear_wheel_sys.wheel_actuate(goal_v_r, goal_theta_r, dt)

        # change the GBM's pose 
        self.pose, self.local_motion = self.odometry_and_local_motion(dt, a_f, a_r, theta_f_dot, theta_r_dot)
        # print(f"GBM's position and orientation: {self.pose}")
        # recording robot's state
        self.wheel_vtheta_state = np.array([[v_f],
                                            [theta_f],
                                            [v_r],
                                            [theta_r]])
        self.wheel_xy_state = np.array([[v_f * math.cos(theta_f)],
                                        [v_f * math.sin(theta_f)],
                                        [v_r * math.cos(theta_r)],
                                        [v_r * math.sin(theta_r)]])
        
    def move_GBM(self) -> None:
        self.actuate(self.motion_cmd, dt = 0.01)
        msg = PoseAndWheels()
        msg.pose_x, msg.pose_y, msg.pose_theta = self.pose[0], self.pose[1], self.pose[2]
        msg.wheel_theta_f, msg.wheel_theta_r = self.wheel_vtheta_state[1,0], self.wheel_vtheta_state[3,0]
        self.pose_publisher.publish(msg=msg)
        rotate_M = np.array([[np.cos(self.pose[2]), -np.sin(self.pose[2])],
                             [np.sin(self.pose[2]), np.cos(self.pose[2])]])
        self.d_points[:,0:1] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[self.D/2], [self.W/2]])
        self.d_points[:,1:2] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[self.D/2], [-self.W/2]])
        self.d_points[:,2:3] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[-self.D/2], [-self.W/2]])
        self.d_points[:,3:4] = np.array([[self.pose[0]], [self.pose[1]]]) + rotate_M @ np.array([[-self.D/2], [self.W/2]])
        self.d_points[:,4:5] = np.array([[self.pose[0]], [self.pose[1]]])

    def cmd_subscribe_callback(self, msg: MotionCMD) -> None:
        motion_cmd = np.array([[msg.vx_cmd], [msg.vy_cmd], [msg.omega_cmd]]) 
        if msg.move_type == 1:
            self.get_logger().info(f"receive global motion command: [{msg.vx_cmd},{msg.vy_cmd},{msg.omega_cmd}]")
            self.motion_cmd = self.global2local(motion_cmd)
        else:
            self.get_logger().info(f"receive local motion command: [{msg.vx_cmd},{msg.vy_cmd},{msg.omega_cmd}]")
            self.motion_cmd = motion_cmd

def main(args=None):
    rclpy.init(args=args)

    node = GBM_robot(150, 80, 80, [0, 0, 0])
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
