from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

ChannelFactoryInitialize(0, "enx6c1ff7bccc28")  # 改成你的网卡
client = G1ArmActionClient()
client.SetTimeout(10.0)
client.Init()

code, data = client.GetActionList()
print("code =", code)
print("data =", data)