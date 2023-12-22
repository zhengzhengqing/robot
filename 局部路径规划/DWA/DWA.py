#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera
import math

class Config:
    """
    simulation parameter class
    """

    def KinematicModel(state,control,dt):
        """机器人运动学模型

        Args:
            state (_type_): 状态量---x,y,yaw,v,w
            control (_type_): 控制量---v,w,线速度和角速度
            dt (_type_): 离散时间

        Returns:
            _type_: 下一步的状态
        """
        state[0] += control[0] * math.cos(state[2]) * dt
        state[1] += control[0] * math.sin(state[2]) * dt
        state[2] += control[1] * dt
        state[3] = control[0]
        state[4] = control[1]
        return state

    def __init__(self):
        # robot parameters

        #线速度边界
        self.v_max = 1.0
        self.v_min = -0.5

        # 角速度边界
        self.w_max = 40.0 * math.pi / 180.0  # [rad/s], 最大叫速度为 40 度/s
        self.w_max = -40.0 * math.pi / 180.0  # [rad/s], 最大叫速度为 40 度/s

        # 采样分辨率
        self.v_sample = 0.01 # m/s
        self.w_sample = 0.1 * math.pi / 180.0 #[rad/s]

        # 离散时间间隔
        self.dt = 0.1 #[s] Time tick for motion prediction

        # 轨迹推算时间长度
        self.predict_time = 3 # [s]

        # 轨迹评价系数
        self.alpha = 0.15
        self.beta = 1.0
        self.gamma = 1.0

        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0 # [m] for collision check

        self.judge_distance = 10 # 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值

        # 障碍物位置 [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [8.0, 10.0],
                    [9.0, 11.0],
                    [12.0, 13.0],
                    [12.0, 12.0],
                    [15.0, 15.0],
                    [13.0, 13.0]
                    ])
        
        # 目标点位置
        self.target = np.array([10,10])

class DWA:
    def __init__(self, config):
        self.dt=config.dt
        self.v_min=config.v_min
        self.w_min=config.w_min
        self.v_max=config.v_max
        self.w_max=config.w_max
        self.predict_time = config.predict_time
        self.a_vmax = config.a_vmax
        self.a_wmax = config.a_wmax
        self.v_sample = config.v_sample # 线速度采样分辨率
        self.w_sample = config.w_sample # 角速度采样分辨率
        self.alpha = config.alpha
        self.beta = config.beta
        self.gamma = config.gamma
        self.radius = config.robot_radius
        self.judge_distance = config.judge_distance

    def dwa_control(self, state, goal, obstacle):
        """滚动窗口算法入口

        Args:
            state (_type_): 机器人当前状态--[x,y,yaw,v,w]
            goal (_type_): 目标点位置，[x,y]

            obstacle (_type_): 障碍物位置，dim:[num_ob,2]

        Returns:
            _type_: 控制量、轨迹（便于绘画）
        """

        control, trajectory = self.trajectory_evaluation(state,goal,obstacle)
        return control, trajectory
    
    def cal_dynamic_window_vel(self, v, w, state, obstacle):
        """速度采样,得到速度空间窗口

        Args:
            v (_type_): 当前时刻线速度
            w (_type_): 当前时刻角速度
            state (_type_): 当前机器人状态
            obstacle (_type_): 障碍物位置
        Returns:
            [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
        """
        