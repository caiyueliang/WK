#!/usr/bin/env python3
import rospy
import actionlib
import threading
import time
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# 记录机器人当前在 map 坐标系下的位姿，供导航过程中的距离判断使用。
current_global_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
# TF 回调和主线程都会读写位姿，这里用锁避免并发访问问题。
pose_lock = threading.Lock()

class RobotController:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK 通道、语音、手臂动作和运动控制客户端。
        rospy.loginfo("正在初始化语音和动作系统...")
        ChannelFactoryInitialize(0, network_interface)
        self.audio_client = AudioClient()
        self.audio_client.Init()
        self.audio_client.SetTimeout(10.0)
        self.arm_client = G1ArmActionClient()
        self.arm_client.Init()
        self._wakeup_audio()
        self.global_plan_length = 0
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        # 订阅全局路径长度，用于粗略判断当前是否因为障碍物导致无有效路径。
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._path_callback)
        # 发布速度指令，主要用于原地旋转对齐航向和强制停车。
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("✅ 语音和动作系统初始化完成")

    def _path_callback(self, msg: Path):
        # 只保留路径点数量，作为“全局规划是否存在”的简单标记。
        self.global_plan_length = len(msg.poses)

    def _wakeup_audio(self):
        # 某些硬件在启动后需要先访问一次服务，避免首次播报失败。
        rospy.loginfo("🔊 正在唤醒音频硬件...")
        for i in range(10):
            try:
                self.audio_client.GetVolume()
                rospy.loginfo("✅ 音频服务已连接")
                break
            except:
                rospy.logwarn(f"⏳ 等待音频服务... ({i+1}/10)")
                time.sleep(1)
        self.audio_client.SetVolume(100)
        time.sleep(0.5)
        rospy.loginfo("✅ 音频硬件唤醒完成")

    def speak(self, text):
        # 播放语音，不阻塞到播报结束，只负责下发 TTS 指令。
        try:
            rospy.loginfo(f"🔊 说: {text}")
            self.audio_client.TtsMaker(text, 0)
        except Exception as e:
            rospy.logerr(f"语音播放失败: {e}")

    def perform_interaction(self, text, action_id):
        # 到站后的标准流程：先做动作，再播报，最后让手臂复位。
        rospy.loginfo(f"🤖 执行动作 ID: {action_id}")
        try:
            self.arm_client.ExecuteAction(action_id)
            time.sleep(2.0)
        except Exception as e:
            rospy.logerr(f"动作执行失败: {e}")

        rospy.loginfo(f"🎤 播放: {text}")
        self.speak(text)
        
        estimated_speech_time = len(text) * 0.195
        rospy.loginfo(f"⏳ 预计讲解时长: {estimated_speech_time:.1f} 秒，正在等待讲解结束...")
        time.sleep(estimated_speech_time)

        rospy.loginfo("🔄 讲解结束，正在复位手臂...")
        try:
            self.arm_client.ExecuteAction(99)
            time.sleep(3.0)
        except Exception as e:
            rospy.logerr(f"复位失败: {e}")

    # 原地旋转到目标朝向，弥补 move_base 在“到点即停”时朝向不够准的问题。
    def rotate_to_yaw(self, target_yaw, listener):
        rospy.loginfo(f"🔄 开始原地旋转修正航向至: {math.degrees(target_yaw):.1f}°")
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            try:
                # 通过 TF 获取机器人当前朝向。
                (trans, rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                current_yaw = euler_from_quaternion(rot)[2]
                
                # 使用 atan2 归一化角度差，避免跨越 pi/-pi 时出现跳变。
                yaw_diff = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
                
                # 如果角度误差足够小，就结束旋转。
                if abs(yaw_diff) < 0.1:
                    rospy.loginfo("✅ 航向对齐完成")
                    break
                
                # 使用简单的 P 控制器输出角速度，并限制最大旋转速度。
                cmd_wz = max(-0.6, min(0.6, yaw_diff * 1.5))
                
                # 只给角速度，不给线速度，实现原地转向。
                twist_msg = Twist()
                twist_msg.angular.z = cmd_wz
                self.cmd_vel_pub.publish(twist_msg)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # TF 短暂不可用时忽略本次循环，等待下一个周期继续。
                pass
            
            rate.sleep()
        
        # 发送零速度，确保原地旋转彻底停止。
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.1)


def set_fast_params():
    # 远距离巡航参数：更快的线速度，更激进的路径跟踪。
    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 0.8)
    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_theta', 0.8)
    rospy.set_param('/move_base/TebLocalPlannerROS/acc_lim_x', 0.8)
    rospy.set_param('/move_base/TebLocalPlannerROS/acc_lim_theta', 0.7)
    rospy.set_param('/move_base/TebLocalPlannerROS/path_distance_bias', 60.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/goal_distance_bias', 20.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/xy_goal_tolerance', 0.3)
    rospy.set_param('/move_base/TebLocalPlannerROS/yaw_goal_tolerance', 0.3)
    rospy.set_param('/move_base/TebLocalPlannerROS/min_turning_radius', 0.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/weight_optimaltime', 1.0)
    rospy.loginfo("🚀 切换到快速巡航模式")

def set_slow_params():
    # 近距离精调参数：降低线速度，允许更稳的靠近目标点。
    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 0.6)
    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_theta', 1.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/acc_lim_x', 0.5)
    rospy.set_param('/move_base/TebLocalPlannerROS/acc_lim_theta', 0.8)
    rospy.set_param('/move_base/TebLocalPlannerROS/min_turning_radius', 0.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x_backwards', 0.0)
    rospy.set_param('/move_base/TebLocalPlannerROS/xy_goal_tolerance', 0.2)
    rospy.set_param('/move_base/TebLocalPlannerROS/yaw_goal_tolerance', 0.2)
    rospy.loginfo("🐌 切换到精准调整模式")


def force_robot_stop(robot_controller_instance):
    # 双保险停车：同时调用 SDK 停止和 ROS 速度清零。
    rospy.logwarn("🛑 强制接管控制...")
    try:
        robot_controller_instance.loco_client.StopMove()
        stop_msg = Twist()
        for _ in range(10):
            robot_controller_instance.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.02)
        rospy.loginfo("🛑 SDK StopMove 指令已发送")
    except Exception as e:
        rospy.logerr(f"SDK 停止指令发送失败: {e}")
    time.sleep(0.1)

def navigate_to_waypoints(waypoints, robot_controller):
    # 连接 move_base，后续所有导航目标都通过 actionlib 发送。
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待move_base服务器启动...")
    client.wait_for_server()

    # 用于在“路线被障碍物卡住”时手动清一次代价地图。
    clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    # 这里设置的是 DWA 的一些公共参数，保障整个导航过程有一致的转向表现。
    rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 0.4)
    rospy.set_param('/move_base/DWAPlannerROS/max_rot_vel', 0.6)
    rospy.set_param('/move_base/DWAPlannerROS/min_rot_vel', 0.2)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_theta', 0.2)
    rospy.set_param('/move_base/DWAPlannerROS/path_distance_bias', 40.0)
    rospy.set_param('/move_base/DWAPlannerROS/goal_distance_bias', 15.0)
    rospy.set_param('/move_base/planner_patience', 18.0)
    rospy.set_param('/move_base/planner_frequency', 1.0)

    listener = tf.TransformListener()
    base_frame = "/base_link" 
    map_frame = "/map"
    
    # 本脚本不依赖 move_base 严格对齐朝向，而是“先到点，再单独修正角度”。
    STOP_YAW_TOLERANCE = 3.14  # 几乎任何角度都允许停止

    for idx, waypoint in enumerate(waypoints):
        # 每次发目标前先读取真实起点，便于计算本次导航距离和停车阈值。
        rospy.loginfo("⏳ 获取机器人在地图中的真实初始位置...")
        try:
            listener.waitForTransform(map_frame, base_frame, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
            with pose_lock:
                current_global_pose["x"] = trans[0]
                current_global_pose["y"] = trans[1]
            rospy.loginfo(f"📍 初始全局位置: ({current_global_pose['x']:.2f}, {current_global_pose['y']:.2f})")
        except Exception as e:
            rospy.logerr(f"❌ 无法获取 TF 变换: {e}")
            continue

        # 根据起点到目标点的距离，动态决定停车半径：
        # 近距离导航要求更精确，远距离导航允许稍微宽松一些。
        start_dist = math.sqrt(
            (waypoint["x"] - current_global_pose["x"]) ** 2 + 
            (waypoint["y"] - current_global_pose["y"]) ** 2
        )
        
        if start_dist < 1.0:
            STOP_DISTANCE = 0.4
        else:
            STOP_DISTANCE = 0.7
            
        # 提前减速，避免快到点时冲过头。
        SLOWDOWN_DISTANCE = 1.5

        # 构造 move_base 目标点，位置和目标朝向都使用 map 坐标系。
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint["x"]
        goal.target_pose.pose.position.y = waypoint["y"]
        goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, waypoint["yaw"])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"发送第{idx+1}个目标: ({waypoint['x']}, {waypoint['y']}, yaw: {waypoint['yaw']:.2f})")
        
        # 每次新目标都从快速巡航模式起步。
        set_fast_params()
        client.send_goal(goal)

        # 这些状态变量用于“避障播报”“清图”“减速切换”等运行时逻辑。
        start_time = time.time()
        last_spoke_time = 0.0
        speak_interval = 10.0
        obstacle_start_time = 0.0 
        map_cleared = False
        is_fast_mode = True
        slowdown_switched = False
        
        while not rospy.is_shutdown():
            # 持续监控 action 状态和机器人当前位置。
            state = client.get_state()
            elapsed = time.time() - start_time
            now = time.time()

            def handle_arrival(reason):
                # 统一封装“到站收尾流程”，避免多个到达出口重复代码。
                rospy.loginfo(f"✅ {reason}：触发第{idx+1}个点停止流程")
                client.cancel_goal()
                force_robot_stop(robot_controller)
                
                # 到点后单独修正朝向，让位姿朝向更稳定。
                try:
                    (trans, rot) = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
                    current_yaw = euler_from_quaternion(rot)[2]
                    yaw_diff = math.atan2(math.sin(waypoint["yaw"] - current_yaw), math.cos(waypoint["yaw"] - current_yaw))
                    
                    # 如果偏差较大，就执行原地旋转。
                    if abs(yaw_diff) > 0.15: # 约 8 度
                        robot_controller.rotate_to_yaw(waypoint["yaw"], listener)
                except:
                    # 航向修正失败不阻塞后续讲解流程。
                    pass
                
                # 到站后播报并执行手臂动作。
                say_text = waypoint.get("say_text", "你好")
                action_id = waypoint.get("action_id", 25)
                robot_controller.perform_interaction(say_text, action_id)
                
                rospy.loginfo("🎯 讲解与动作执行完毕，准备前往下一站")
                return True

            if state == actionlib.GoalStatus.ACTIVE:
                try:
                    # 实时通过 TF 获取当前位置和朝向。
                    (trans, rot) = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
                    cur_x = trans[0]
                    cur_y = trans[1]
                    current_yaw = euler_from_quaternion(rot)[2]
                    
                    with pose_lock:
                        current_global_pose["x"] = cur_x
                        current_global_pose["y"] = cur_y
                        current_global_pose["yaw"] = current_yaw

                    distance_to_goal = math.sqrt(
                        (waypoint["x"] - cur_x) ** 2 + 
                        (waypoint["y"] - cur_y) ** 2
                    )
                    
                    yaw_diff = math.atan2(
                        math.sin(waypoint["yaw"] - current_yaw), 
                        math.cos(waypoint["yaw"] - current_yaw)
                    )

                    # 快到目标时切换到慢速参数，让终点附近动作更稳。
                    if is_fast_mode and distance_to_goal < SLOWDOWN_DISTANCE and not slowdown_switched:
                        rospy.loginfo(f"📏 距离目标 {distance_to_goal:.2f}m，切换到慢速模式")
                        set_slow_params()
                        is_fast_mode = False
                        slowdown_switched = True
                    
                    # 只要物理距离足够近，就认为可以先停下，再单独修正朝向。
                    if elapsed > 0.8 and distance_to_goal < STOP_DISTANCE:
                        if handle_arrival(f"物理距离达标 (Dist: {distance_to_goal:.2f}m)"):
                            break

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    # TF 读取失败时，先跳过本轮位置判断。
                    pass

                if elapsed > 0.8: 
                    # 如果全局路径长度为 0，说明规划器暂时没有有效路径，
                    # 这里将其视为可能被障碍物阻挡。
                    if robot_controller.global_plan_length == 0:
                        if obstacle_start_time == 0.0:
                            obstacle_start_time = now
                        
                        if now - obstacle_start_time > 1.5: 
                            # 被挡住持续一段时间后，周期性语音提醒行人避让。
                            if now - last_spoke_time > speak_interval:
                                robot_controller.speak("请让一让我")
                                last_spoke_time = now
                            
                            # 只清一次代价地图，避免频繁清图造成规划抖动。
                            if (not map_cleared):
                                rospy.loginfo("🧹 尝试清除代价地图...")
                                try:
                                    clear_costmaps_service()
                                    map_cleared = True
                                except Exception as e:
                                    rospy.logerr(f"清除代价地图失败: {e}")
                    else:
                        # 规划恢复后，重置“被障碍阻挡”的计时状态。
                        if obstacle_start_time != 0.0:
                            rospy.loginfo("✅ 路径已恢复")
                            obstacle_start_time = 0.0
                            map_cleared = False

            # move_base 自己宣布成功时，也走同一套到站收尾逻辑。
            if state == actionlib.GoalStatus.SUCCEEDED:
                if handle_arrival("Move_base 判定到达"):
                    break

            # 如果规划失败，则播报后跳过当前点，继续后续任务。
            if state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn(f"⚠️ 第{idx+1}个目标导航失败，跳过")
                robot_controller.speak("无法到达该目标位置，即将前往下一位置")
                time.sleep(3.0)
                break

            # 小周期轮询，兼顾响应性和 CPU 占用。
            rospy.sleep(0.01) 

if __name__ == '__main__':
    try:
        # 初始化 ROS 节点。
        rospy.init_node('multi_waypoint_nav')

        import sys
        # 启动参数需要传入机器人网络接口，例如 eth0/wlan0。
        if len(sys.argv) < 2:
            rospy.logerr("使用方法: rosrun your_package multi_nav_action_audio.py network_interface")
            sys.exit(1)

        network_interface = sys.argv[1]
        # 创建统一的机器人控制器，封装语音、动作和底盘控制接口。
        robot_controller = RobotController(network_interface)

        rospy.sleep(1.0)
        robot_controller.speak("启动成功")

        # 预设多个讲解点：每个点包含位置、朝向、动作编号和播报文本。
        waypoints = [
            {
                "x": 5.33,          # map坐标系下的目标位置x坐标（单位：米）
                "y": -1.30,         # map坐标系下的目标位置y坐标（单位：米）
                "yaw": 1.09,        # 目标航向角（绕z轴的欧拉角，单位：弧度，对应约62.5度）
                "action_id": 31,    # 到站后执行的手臂动作编号
                "say_text": "3333"  # 到站后语音播报的文本内容
            },
            {"x": -0.15, "y":-1.62, "yaw": -0.83, "action_id":31,"say_text": "7777"},
            {"x": 5.13, "y": 7.04, "yaw": 2.77, "action_id": 31, "say_text": "8888"},
            {"x": -1.78, "y": 6.37, "yaw": -2.98, "action_id": 31, "say_text": "2222"},
        ]

        # 依次执行多点导航、到站讲解和动作互动。
        navigate_to_waypoints(waypoints, robot_controller)

    except rospy.ROSInterruptException:
        rospy.loginfo("导航脚本被中断")
