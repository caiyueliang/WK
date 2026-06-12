#!/usr/bin/env python3
import sys

import numpy as np
import osqp
import rospy
import scipy.sparse as sp
from geometry_msgs.msg import Twist

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

class MPCController:
    def __init__(self, network_interface):
        rospy.loginfo("========================================")
        rospy.loginfo("Initializing MPC Controller (Locked Stop Mode)...")
        rospy.loginfo("========================================")

        # 1. 初始化 SDK
        try:
            ChannelFactoryInitialize(0, network_interface)
        except Exception as e:
            print(f"[ERROR] 网络初始化失败: {e}")
            sys.exit(-1)

        # 2. 初始化客户端
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        try:
            self.sport_client.Init()
        except Exception as e:
            print(f"[ERROR] 机器人连接失败: {e}")
            sys.exit(-1)

        self.control_freq = 50.0
        self.dt = 1.0 / self.control_freq
        self.cmd_timeout = 0.5
        
        # --- MPC 物理约束 ---
        self.max_vx = 1.0   # 最大前进线速度，单位：米/秒
        self.max_vy = 0.0   # 最大横向移动速度，设为0禁止横向移动
        self.max_wz = 0.6   # 最大绕z轴角速度，单位：弧度/秒
        
        # 加速度设置较大，解决起步慢的问题
        self.max_acc_v = 45.0   
        self.max_acc_w = 8.0   

        # --- MPC 权重参数 ---
        self.Q_v = 70.0
        self.R_v = 0.15
        
        # --- 状态变量 ---
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        
        # 真实反馈速度 (来自 DDS)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.last_cmd_time = rospy.Time.now()

        # --- 【移植自 V4】到位锁定机制参数 ---
        self.is_stopped = False
        self.stop_velocity_threshold = 0.01  # 判定停止的速度阈值
        
        # 订阅
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # DDS 订阅
        self.odom_dds_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_dds_sub.Init(self.dds_odom_callback)

        self.control_timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        
        rospy.loginfo("🚀 控制器已启动 (MPC + 锁定停止)")

    def dds_odom_callback(self, msg: SportModeState_):
        self.current_vx = msg.velocity[0]
        self.current_vy = msg.velocity[1]
        self.current_wz = msg.yaw_speed

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = rospy.Time.now()

        # 【移植自 V4】如果处于锁定状态，忽略微小指令，防止抖动
        if self.is_stopped:
            if abs(msg.linear.x) < 0.05 and abs(msg.linear.y) < 0.05 and abs(msg.angular.z) < 0.1:
                return # 保持锁定，不更新目标
            else:
                # 收到明显的运动指令，解锁
                self.is_stopped = False
                rospy.loginfo("🔓 解锁：开始新运动")

        vx = msg.linear.x
        vy = 0.0
        wz = msg.angular.z

        # 前进时禁止横向速度，G1 更容易走直
        if abs(vx) > 0.1:
            vy = 0.0

        # 前进时忽略小角速度，防止局部规划器一点点修正导致走弧线
        if abs(vx) > 0.15 and abs(wz) < 0.18:
            wz = 0.0

        # 前进时限制最大转向，不要边走边大幅转弯
        if abs(vx) > 0.15:
            wz = max(-0.35, min(0.35, wz))

        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz
        #self.target_vx = msg.linear.x
        #self.target_vy = msg.linear.y
        #self.target_wz = msg.angular.z

    def solve_mpc_step(self, v_current, v_target, v_last_cmd, max_v, max_acc):
        # 将当前实际速度限制在最大允许速度范围内，防止超出物理限制
        v_current = np.clip(v_current, -max_v, max_v)
        # 将目标速度同样限制在最大允许速度范围内，确保目标值合法
        v_target = np.clip(v_target, -max_v, max_v)
        
        P = sp.csc_matrix([[2 * (self.Q_v + self.R_v)]])
        q = np.array([-2 * (self.Q_v * v_target + self.R_v * v_last_cmd)])
        
        acc_limit = max_acc * self.dt
        # 融合当前反馈速度与上一周期指令，帮助角速度平滑突破底盘启动死区
        # 0.3权重给当前反馈值，保留一定的实际状态感知
        v_base = 0.3 * v_current + 0.7 * v_last_cmd
        # 计算本次指令的最小允许值：不低于最大反向速度，同时不超过加速度限制的最小步长
        lower_bound = np.array([max(-max_v, v_base - acc_limit)])
        # 计算本次指令的最大允许值：不高于最大正向速度，同时不超过加速度限制的最大步长
        upper_bound = np.array([min(max_v, v_base + acc_limit)])
        
        A_box = sp.csc_matrix([[1.0]])
        
        prob = osqp.OSQP()
        prob.setup(P, q, A_box, lower_bound, upper_bound, verbose=False, eps_abs=1e-3, eps_rel=1e-3)
        res = prob.solve()
        
        if res.info.status != 'solved':
            return 0.0
        return res.x[0]

    def control_loop(self, event):
        # --- 调试打印区 ---
        cmd_is_fresh = (rospy.Time.now() - self.last_cmd_time).to_sec() <= self.cmd_timeout
        if int(rospy.get_time() * 10) % 10 == 0:
            if self.is_stopped:
                state_str = "LOCKED"
            elif cmd_is_fresh:
                state_str = "ACTIVE"
            else:
                state_str = "IDLE"
            print(f"[State: {state_str}] "
                  f"Target: ({self.target_vx:.2f}, {self.target_wz:.2f}) | "
                  f"Current: ({self.current_vx:.2f}, {self.current_wz:.2f})")

        # 1. 指令超时保护。新导航脚本不再依赖 move_base 的全局路径，
        # 因此用 cmd_vel 心跳替代原来的 path 判定。
        if not cmd_is_fresh:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        # 2.【移植自 V4】到位检测与锁定逻辑
        # 条件：目标速度接近0 且 当前速度很小
        if (abs(self.target_vx) < 0.01 and 
            abs(self.target_vy) < 0.01 and 
            abs(self.target_wz) < 0.01 and
            abs(self.current_vx) < self.stop_velocity_threshold and
            abs(self.current_vy) < self.stop_velocity_threshold and
            abs(self.current_wz) < self.stop_velocity_threshold):
            
            if not self.is_stopped:
                rospy.loginfo("🔒 锁定：到达目标点，停止运动")
            self.is_stopped = True

        # 3. 根据锁定状态执行控制
        if self.is_stopped:
            # 【关键】锁定状态下，强制发送 0，跳过 MPC 计算
            self.sport_client.Move(0.0, 0.0, 0.0)
            # 重置指令记录，防止下次启动时突变
            self.last_cmd_vx = 0.0
            self.last_cmd_vy = 0.0
            self.last_cmd_wz = 0.0
        else:
            # 正常 MPC 计算流程
            cmd_vx = self.solve_mpc_step(self.current_vx, self.target_vx, self.last_cmd_vx, self.max_vx, self.max_acc_v)
            cmd_vy = self.solve_mpc_step(self.current_vy, self.target_vy, self.last_cmd_vy, self.max_vy, self.max_acc_v)
            cmd_wz = self.solve_mpc_step(self.current_wz, self.target_wz, self.last_cmd_wz, self.max_wz, self.max_acc_w)

            # 原地转向起步时，如果反馈角速度几乎为 0，给一个最小起转速度
            # 防止被底盘死区卡住，导致 yaw 长时间不变化。
            if (
                abs(self.target_vx) < 0.05      # 目标线速度接近0，处于原地转向场景
                and abs(self.target_wz) > 0.2   # 目标角速度足够大，确实需要转向
                and abs(self.current_wz) < 0.03 # 当前实际角速度接近0，被底盘死区卡住
                and abs(cmd_wz) < 0.3           # MPC计算出的指令角速度也偏小
            ):
                # 给一个最小的起转角速度，突破底盘死区，方向和目标角速度一致
                cmd_wz = 0.3 if self.target_wz > 0 else -0.3

            # 发送指令
            if abs(cmd_vx) < 0.01 and abs(cmd_vy) < 0.01 and abs(cmd_wz) < 0.01:
                self.sport_client.StopMove()
            else:
                self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)
            
            self.last_cmd_vx = cmd_vx
            self.last_cmd_vy = cmd_vy
            self.last_cmd_wz = cmd_wz

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 g1_control_mpc_final.py <network_interface>")
        sys.exit(-1)

    network_interface = sys.argv[1]
    rospy.init_node("unitree_mpc_controller", anonymous=False)
    
    try:
        controller = MPCController(network_interface)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
