from dobot_api import DobotApiDashboard, DobotApi
from time import sleep
import re

IP = "192.168.5.1"
LOCAT_IP="192.168.5.175"
MUDBUS_PORT = 60000
DASHBOARD_PORT = 29999
MOVE_PORT = 30003
FEED_PORT = 30004

# 机械臂位置参数（根据实际场景校准）
DEFAULT_POS = (150, 17.5, 126, 39, 81, 132)  # 默认位置
PRE_GRAB_POS = (183, 17.5, 126, 39, 81, 132)  # 
TARGET_POS = (183, 17.5, 116, 39, 81, 132)  # 抓取位置"""  """


class Connect_Dobot:

    def __init__(self, ip: str = IP, dashboard_port: int = DASHBOARD_PORT, feed_port: int = FEED_PORT):
        # 初始化连接
        try:
            self.__dobot_dashboard = DobotApiDashboard(ip, dashboard_port)
            #self.__dobot_move = DobotApiMove(ip, move_port)
            self.__dobot_feed = DobotApi(ip, feed_port)

            self.Set_Speed(5)
        except Exception as e:
            print(f"Error initializing Dobot API: {e}")
            raise
        error = self.__dobot_dashboard.ClearError()
        print(f"清除错误返回: {error}")  # 若返回"ok"，说明无错误或已清除
        sleep(1)
        # print(self.__dobot_dashboard.RequestControl())
        # sleep(1)
        # # 执行在TCP模式
        # 机械臂上电
        print(self.__dobot_dashboard.PowerOn())
        print("Dobot Power On")
        sleep(11)
        # 使能机械臂
        print(self.__dobot_dashboard.EnableRobot(0.75, 0.0, 0.0, 0.085))
        # print(self.__dobot_dashboard.DisableRobot())
        print("Dobot Enabled")
        sleep(1)

    def __del__(self):
        # 机械臂下电
        # self.__dobot_dashboard.DisableRobot()
        pass
    
    def Set_Speed(self, speed: float = 10):
        print(self.__dobot_dashboard.SpeedFactor(speed))
        print(self.__dobot_dashboard.SpeedJ(speed))
        print(self.__dobot_dashboard.SpeedL(speed))
        print(f"Set Speed Factor to {speed}%")
        sleep(1)

    def Get_Controller(self):
        return self.__dobot_dashboard, self.__dobot_feed
    

def extract_number(text):
    """使用正则表达式提取花括号中的数字"""
    match = re.search(r'\{(\d+)\}', text)
    if match:
        return int(match.group(1))
    return None

def extract_number_before_brace(text):
    """提取花括号前面的数字"""
    match = re.search(r'(\d+),\{', text)
    if match:
        return int(match.group(1))
    return None
    

class Ohand_Control:
    
    def __init__(self, dobot_dashboard: DobotApiDashboard):
        self.__dobot_dashboard = dobot_dashboard

        result = self.__dobot_dashboard.ModbusCreate(IP, MUDBUS_PORT, 2, 1)
        # result = self.__dobot_dashboard.ModbusRTUCreate(2, 115200)

        # print(self.__dobot_dashboard.SetToolPower(0))
        # sleep(10)
        # print(self.__dobot_dashboard.SetToolPower(1))
        # sleep(40)
        
        # 获取485终端口信息
        # self.mudbus_index = self.__dobot_dashboard.ModbusRTUCreate(2, 115200, "N", 8, 1)
        print("Succeed init 485:", result)
        self.mudbus_index = extract_number(result)
        print(self.mudbus_index, type(self.mudbus_index))
        if extract_number_before_brace(result) != 0:
            exit(-1)
        # sleep(1)

    def __del__(self):
        if self.mudbus_index is not None:
            self.__dobot_dashboard.ModbusClose(self.mudbus_index)
            # sleep(1)
        else:
            pass

    def Ohand_Close(self):
        #self.__dobot_dashboard.sendRecvMsg(f"SetTool485({115200})")
        result = self.__dobot_dashboard.RobotMode()
        print("RobotMode指令输出日志:", result)

        #self.__dobot_dashboard.GetHoldRegs(self.mudbus_index, 1135, count=1)

        # 写入寄存器，打开手掌
        # self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1135, count=1, valTab="{0}", valType="U16")
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1136, count=1, valTab="{65535}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1137, count=1, valTab="{65535}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1138, count=1, valTab="{65535}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1139, count=1, valTab="{65535}", valType="U16")
        # sleep(1)
        #self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1140, count=4, valTab='{65535,65535,65535,0}', valType="U16")
        
    def Ohand_capture(self):
        #self.__dobot_dashboard.sendRecvMsg(f"SetTool485({115200})")
        result = self.__dobot_dashboard.RobotMode()
        print("抓取:", result)

        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1135, count=1, valTab="{13759}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1136, count=1, valTab="{17742}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1137, count=1, valTab="{13034}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1138, count=1, valTab="{16293}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1139, count=1, valTab="{14482}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1140, count=1, valTab="{65535}", valType="U16")

    def Ohand_Open(self):
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1135, count=1, valTab="{0}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1136, count=1, valTab="{0}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1137, count=1, valTab="{0}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1138, count=1, valTab="{0}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1139, count=1, valTab="{0}", valType="U16")
        sleep(0.1)  # 避免指令发送过快
        self.__dobot_dashboard.SetHoldRegs(self.mudbus_index, 1140, count=1, valTab="{0}", valType="U16")
        


class Dobot_Arm_Control:

    def __init__(self, dobot_dashboard: DobotApiDashboard):
        self.__dobot_dashboard = dobot_dashboard

    def Move_Joints(self, joints: tuple=(0, 0, 0, 0, 0, 0), is_blocking: bool = True):
        #self.__dobot_dashboard.ServoJ(*joints, is_blocking)
        self.__dobot_dashboard.MovL(*joints, is_blocking)
        
        sleep(1)


if __name__ == "__main__":
    # 连接机械臂
    dobot = Connect_Dobot(ip=IP, dashboard_port=DASHBOARD_PORT, feed_port=FEED_PORT)
    dobot_dashboard, dobot_feed = dobot.Get_Controller()
    # 初始化灵巧手
    ohand = Ohand_Control(dobot_dashboard)
    # 初始化机械臂
    arm = Dobot_Arm_Control(dobot_dashboard)

    # 1灵巧手张开
    ohand.Ohand_Open()
    sleep(3)

    # 设置机械臂运动速度
    dobot.Set_Speed(speed=20)

    # 2运动到默认位置
    print("运动到默认位置...")
    arm.Move_Joints(joints=DEFAULT_POS)
    sleep(3)

    # 3运动到抓取前位置
    print("运动到抓取前位置...")
    arm.Move_Joints(joints=PRE_GRAB_POS)
    sleep(3)

    # 4灵巧手抓取
    ohand.Ohand_capture()
    sleep(4)

    # 5运动到目标位置
    print("运动到目标位置...")
    arm.Move_Joints(joints=TARGET_POS)
    sleep(3)

    # sleep(4)
    # 6机械臂运动到默认位置
    arm.Move_Joints(joints=DEFAULT_POS)
    sleep(3)
    # # 机械臂运动到指定位置
    # arm.Move_Joints(joints=(0, -30, 30, 0, 0, 0), is_blocking=True)
    # sleep(1)
    # # 机械臂运动到指定位置
    # arm.Move_Joints(joints=(0, -10, 10, 0, 0, 0), is_blocking=True)
    # sleep(1)
    # 灵巧手张开
    # ohand.Ohand_Open()
    # sleep(2)