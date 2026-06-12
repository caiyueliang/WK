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
        # 创建Twist速度消息实例，用于封装机器人运动指令
        twist_msg = Twist()
        # 设置线速度的x分量（机器人前后移动速度，单位：m/s）
        twist_msg.linear.x = vx
        # 设置角速度的z分量（机器人原地旋转速度，单位：rad/s）
        twist_msg.angular.z = wz
        # 将速度消息发布到ROS的/cmd_vel话题，驱动机器人运动
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
                    "武汉人工智能研究院是由武汉东湖新技术开发区设立的新型研发机构，"
                    "依托中国科学院自动化研究所在人工智能领域的深厚积累，以及武汉市优越的区位、科教与产业优势，"
                    "聚焦跨模态智能这一国际前沿研究方向。我们的核心目标，是构建全栈国产化的人工智能重大基础设施平台，"
                    "推动人工智能成果从实验室走向规模化应用。"
                    "研究院秉持“立足武汉、辐射中部、服务全国”的发展方针。"
                    "目前，我们正围绕具身智能、科学智能、低空经济等前沿方向，持续开展核心技术攻关，"
                    "积极推动“人工智能+”与各行业的深度融合，致力于为湖北加快建成中部地区崛起的重要战略支点提供科技支撑。"
                    "研究院的重要支撑——中国科学院自动化研究所，成立于1956年，是我国最早开展类脑智能研究的国立研究机构，"
                    "也是国内首个“人工智能学院”的牵头承办单位，在智能科学与技术领域形成了鲜明的学科优势和技术特色。"
                    "接下来，让我们一起沿着时间脉络，走进“紫东太初”大模型的创新发展之路。"
                ),
            },
            {
                "x": -4,
                "y": 1.25,
                # "yaw": 0.109,
                "yaw": 1.089,
                "action_id":25,
                "say_text": (
                    "从初创到引领，“紫东太初”的成长路径上，有着一系列关键的里程碑节点。"
                    "早在2020年以前，中科院自动化所的紫东太初研发团队就启动了“跨模态通用人工智能开放平台”这一创新任务，"
                    "并列入中科院“十四五”规划重点方向，从一开始就承载着国家使命。"
                    "标志着模型在深度推理与工程化落地方面迈入了新阶段。"
                ),
            },
            {
                "x": -3.53,
                "y": 4.91,
                "yaw": -0.316,
                "action_id": 25,
                "say_text": (
                    "为了让模型能力更好地服务千行百业，我们基于“紫东太初”构建了一套完整产品与平台体系。"
                    "我们的核心平台产品“紫东太初云”，是国内首个全栈国产的万卡智算云平台，"
                    "能够为企业提供从底层算力到顶层应用的全链路一站式支持。"
                    "算力服务平台，实现了“一云多芯”，覆盖全国18座城市，可调度超过10000P的弹性算力，"
                    "适配10多种国产芯片，真正做到开放兼容。"
                    "大模型训推平台，管理大模型开发的全生命周期，支持主流模型训练与推理。"
                    "目前已有超过10万家企业用户使用，开放了超过5000个服务接口，覆盖100多种垂直行业算法。"
                    "应用开发平台，提供零代码或低代码开发模式，用户无需编程即可通过“搭积木”的方式快速构建AI应用，"
                    "开发周期可缩短60%以上。"
                    "该平台已统一接入超过200个主流模型，极大降低了AI应用门槛。"
                ),
            },
            {
                "x": -2.8,
                "y": 8,
                "yaw": -2.-1.500,
                "action_id": 25,
                "say_text": (
                    "未来，武汉人工智能研究院将继续秉持“立足武汉、辐射中部、服务全国”的发展方针，"
                    "打造新一代人工智能技术创新策源地和产业发展高地，与各界伙伴携手，"
                    "共同推动人工智能与经济社会深度融合，为数字中国建设贡献坚实的科技力量。"
                    "以上是展厅的全部介绍，谢谢大家！"
                ),
            },
        ]
        navigate_to_waypoints(waypoints, robot_controller)
    except rospy.ROSInterruptException:
        rospy.loginfo("导航脚本被中断")
