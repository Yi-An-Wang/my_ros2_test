#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

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