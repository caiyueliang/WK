#!/usr/bin/env python3
import math
import time

import rospy
import tf
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RobotController:
    def __init__(self, network_interface):
        rospy.loginfo("正在初始化语音和动作系统...")
        ChannelFactoryInitialize(0, network_interface)

        self.audio_client = AudioClient()
        self.audio_client.Init()
        self.audio_client.SetTimeout(10.0)

        self.arm_client = G1ArmActionClient()
        self.arm_client.Init()

        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self._wakeup_audio()
        rospy.loginfo("✅ 语音和动作系统初始化完成")

    def _wakeup_audio(self):
        rospy.loginfo("🔊 正在唤醒音频硬件...")
        for i in range(10):
            try:
                self.audio_client.GetVolume()
                rospy.loginfo("✅ 音频服务已连接")
                break
            except Exception:
                rospy.logwarn(f"⏳ 等待音频服务... ({i + 1}/10)")
                time.sleep(1)
        self.audio_client.SetVolume(100)
        rospy.loginfo("✅ 已经设置音量为：100")
        time.sleep(0.5)
        rospy.loginfo("✅ 音频硬件唤醒完成")

    def publish_cmd_vel(self, vx=0.0, wz=0.0):
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.angular.z = wz
        self.cmd_vel_pub.publish(twist_msg)

    def stop_motion(self):
        rospy.logwarn("🛑 停止当前运动...")
        try:
            self.loco_client.StopMove()
        except Exception as exc:
            rospy.logwarn(f"StopMove 调用失败，继续发送零速: {exc}")

        for _ in range(10):
            self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.02)

    def speak(self, text):
        try:
            rospy.loginfo(f"🔊 说: {text}")
            self.audio_client.TtsMaker(text, 0)
        except Exception as e:
            rospy.logerr(f"语音播放失败: {e}")

    def perform_interaction(self, text, action_id):
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

    def rotate_to_yaw(self, target_yaw, listener, timeout=8.0):
        rospy.loginfo(f"开始原地旋转修正航向至: {math.degrees(target_yaw):.1f}度")
        rate = rospy.Rate(20)
        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logwarn("航向修正超时，跳过本次原地旋转")
                break

            try:
                _, _, current_yaw = get_current_pose(listener)
                yaw_diff = normalize_angle(target_yaw - current_yaw)

                rospy.loginfo_throttle(
                    0.5,
                    f"当前yaw: {math.degrees(current_yaw):.1f}度, "
                    f"目标yaw: {math.degrees(target_yaw):.1f}度, "
                    f"误差: {math.degrees(yaw_diff):.1f}度",
                )

                if abs(yaw_diff) < 0.15:
                    rospy.loginfo("航向对齐完成")
                    break

                cmd_wz = max(-0.6, min(0.6, yaw_diff * 1.5))
                if abs(cmd_wz) < 0.25:
                    cmd_wz = 0.25 if cmd_wz > 0 else -0.25

                self.publish_cmd_vel(0.0, cmd_wz)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, f"获取TF失败: {e}")

            rate.sleep()

        self.stop_motion()
        time.sleep(0.1)


def get_current_pose(listener):
    trans, rot = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
    current_yaw = euler_from_quaternion(rot)[2]
    return trans[0], trans[1], current_yaw


def drive_straight_to_point(target_x, target_y, listener, robot_controller, timeout=40.0):
    rospy.loginfo(f"🚶 开始直线前往目标点: ({target_x:.2f}, {target_y:.2f})")
    rate = rospy.Rate(20)
    start_time = time.time()
    stop_distance = 0.15

    while not rospy.is_shutdown():
        if time.time() - start_time > timeout:
            rospy.logwarn("直线移动超时，停止本次前往")
            robot_controller.stop_motion()
            return False

        try:
            cur_x, cur_y, cur_yaw = get_current_pose(listener)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, f"获取TF失败: {exc}")
            rate.sleep()
            continue

        dx = target_x - cur_x
        dy = target_y - cur_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < stop_distance:
            rospy.loginfo(f"✅ 到达目标点附近，当前位置距目标 {distance:.2f}m")
            robot_controller.stop_motion()
            return True

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - cur_yaw)

        if distance > 1.5:
            cmd_vx = 0.45
        elif distance > 0.8:
            cmd_vx = 0.30
        elif distance > 0.35:
            cmd_vx = 0.18
        else:
            cmd_vx = 0.10

        cmd_wz = max(-0.25, min(0.25, heading_error * 1.2))

        # 如果朝向误差较大，优先先摆正再走，保证整体更接近直线。
        if abs(heading_error) > 0.35:
            cmd_vx = 0.0
            cmd_wz = max(-0.35, min(0.35, heading_error * 1.5))
        elif abs(heading_error) > 0.15:
            cmd_vx *= 0.5

        rospy.loginfo_throttle(
            0.5,
            f"目标点距离: {distance:.2f}m, heading误差: {math.degrees(heading_error):.1f}度, "
            f"发送速度: vx={cmd_vx:.2f}, wz={cmd_wz:.2f}",
        )
        robot_controller.publish_cmd_vel(cmd_vx, cmd_wz)
        rate.sleep()

    robot_controller.stop_motion()
    return False


def move_to_waypoint_direct(waypoint, listener, robot_controller):
    target_x = waypoint["x"]
    target_y = waypoint["y"]
    target_yaw = waypoint["yaw"]

    rospy.loginfo(
        f"📍 前往第一个目标流程" if waypoint.get("_index", 0) == 0 else
        f"📍 前往第{waypoint['_index'] + 1}个目标流程"
    )

    cur_x, cur_y, _ = get_current_pose(listener)
    dx = target_x - cur_x
    dy = target_y - cur_y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance > 0.08:
        travel_yaw = math.atan2(dy, dx)
        robot_controller.rotate_to_yaw(travel_yaw, listener, timeout=6.0)
        if not drive_straight_to_point(target_x, target_y, listener, robot_controller):
            return False
    else:
        rospy.loginfo("目标点距离很近，跳过直线移动阶段")

    robot_controller.rotate_to_yaw(target_yaw, listener, timeout=6.0)
    return True


def navigate_to_waypoints(waypoints, robot_controller):
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(5.0))

    for idx, waypoint in enumerate(waypoints):
        waypoint["_index"] = idx
        rospy.loginfo(
            f"🚩 开始处理第{idx + 1}个目标点: "
            f"({waypoint['x']:.2f}, {waypoint['y']:.2f}, yaw={waypoint['yaw']:.2f})"
        )

        try:
            success = move_to_waypoint_direct(waypoint, listener, robot_controller)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logerr(f"❌ 第{idx + 1}个目标点获取TF失败: {exc}")
            robot_controller.stop_motion()
            continue

        if not success:
            rospy.logwarn(f"⚠️ 第{idx + 1}个目标点移动失败，跳过讲解")
            continue

        say_text = waypoint.get("say_text", "你好")
        action_id = waypoint.get("action_id", 25)
        robot_controller.perform_interaction(say_text, action_id)
        rospy.loginfo("🎯 讲解与动作执行完毕，准备前往下一站")


if __name__ == "__main__":
    try:
        rospy.init_node("multi_waypoint_nav_direct")

        import sys

        if len(sys.argv) < 2:
            rospy.logerr("使用方法: python3 multi_nav.py network_interface")
            sys.exit(1)

        network_interface = sys.argv[1]
        robot_controller = RobotController(network_interface)

        rospy.sleep(1.0)
        robot_controller.speak("启动成功")

        waypoints = [
            {
                "x": -0.3,
                "y": 0.03,
                "yaw": 1.157,
                "action_id": 25,
                "say_text": (
                    "各位来宾，大家好！欢迎来到武汉人工智能研究院科技展厅。"
                    "首先，我为大家简要介绍研究院和紫东太初大模型的基本情况。"
                ),
            },
            {
                "x": -4,
                "y": 1.25,
                "yaw": 0.109,
                "action_id": 25,
                "say_text": (
                    "从初创到引领，紫东太初的成长路径上，有着一系列关键的里程碑节点。"
                    "早在2020年以前，中科院自动化所的紫东太初研发团队就启动了跨模态通用人工智能开放平台这一创新任务，"
                    "并列入中科院十四五规划重点方向，从一开始就承载着国家使命。"
                    "标志着模型在深度推理与工程化落地方面迈入了新阶段。"
                ),
            },
        ]
        navigate_to_waypoints(waypoints, robot_controller)
    except rospy.ROSInterruptException:
        rospy.loginfo("导航脚本被中断")
