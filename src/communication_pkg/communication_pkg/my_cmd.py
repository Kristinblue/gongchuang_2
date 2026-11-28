"""
应用层再封装
内部是[MyUART]

把原本的cmd等封装起来，对外暴露动作接口

ROS 版的 MyCMD 发话题，由 SerialNode 调用 MyUART。
"""

from robot_interfaces.msg import Command, Feedback

# 分层说明：
#   MyCMD（本文件）只负责“要做什么动作”，把动作翻译成命令位和数值，
#   通过 /robot/command 话题发出去；真正的 6 字节打包、串口读写、
#   看门狗都在 C++ 的 serial_node 里。这样改动作逻辑不用碰串口，
#   改协议也不用碰任务代码。
#   命令编号和设计注释见 robot_interfaces/msg/Command.msg 与 Feedback.msg。


class MyCMD:
    """动作接口通过 ROS 发布，串口由 communication_pkg 持有。"""

    def __init__(self, publisher):
        self.publisher = publisher
        # 相对旋转是否完成，由 read_feedback() 按连续角度回包判断。
        self.turn_done = False
        self.angle = None
        self.straight_done = False
        self._straight_pending = False
        self._turn_angle = 0
        self._turned = 0

    def _send(self, command: int, value: int):
        """发布 ROS2 命令消息，具体串口打包交给 SerialNode。"""
        # Command 中的具名常量本身就是整数，可以直接填入 command。
        # 协议里 value 是 int16，先夹到 -32768~32767，避免越界。
        if command in (Command.STRAIGHT, Command.TURN, Command.TURN_ANGLE, Command.STRAIGHT_DIS, Command.STOP):
            self.straight_done = False
            self._straight_pending = command == Command.STRAIGHT_DIS
        self.publisher.publish(Command(command=command, value=max(-32768, min(32767, int(value)))))

    def move_straight(self, error: int):
        """前进/后退，正数前进，负数后退
        发送的是像素误差，送入下位机的PID
        """
        self._send(Command.STRAIGHT, error)

    def move_straight_dis(self, dis: int):
        """按厘米直行，正数前进，负数后退，由 STM32 执行。"""
        self._send(Command.STRAIGHT_DIS, dis)

    def move_turn(self, error: int):
        """转向，正右负左"""
        # 和前进一样，单位是像素误差，正右负左。
        self._send(Command.TURN, error)

    def move_turn_angle(self, angle: int):
        """直接发送相对转角（度），返回是否已接受请求。"""
        self._turn_angle = max(-32768, min(32767, int(angle)))
        self._turned = 0
        self.turn_done = self._turn_angle == 0
        if self._turn_angle:
            self._send(Command.TURN_ANGLE, self._turn_angle)
        return True

    def read_feedback(self, feedback):
        """处理定距到位回包，或用 01 航向累计转角。"""
        if feedback.command == Feedback.STRAIGHT_DONE:
            if self._straight_pending:
                self._straight_pending = False
                self.straight_done = True
            return
        if feedback.command != Feedback.ANGLE or not 0 <= feedback.value <= 360:
            return
        angle = feedback.value % 360
        if self._turn_angle and self.angle is not None:
            # 例如从 2° 顺时针转到 358°，差值是 -4°。命令正数表示顺时针。
            delta = (angle - self.angle + 180) % 360 - 180
            self._turned += -delta if self._turn_angle > 0 else delta
            if self._turned > 0 and self._turned >= abs(self._turn_angle) - 1:
                self.turn_done = True
                self._turn_angle = 0
        self.angle = angle

    def angle_calib(self, angle:int):
        """绝对角度校准，利用安全区的，之后具体规定"""
        self._send(Command.ANGLE_CALIB, angle)

    def move_stop(self):
        """08停车，并清除旧的前进和转向误差，不改变夹爪。"""
        # 停车同时清掉 turn_done，避免旧的“转完了”状态影响下一次动作。
        self.turn_done = False
        self._turn_angle = 0
        self._send(Command.STOP, 0)

    def grab(self, angle:int):
        """夹爪，0为开，1为关"""
        # 当前协议只有开/关两个值，抓取角度后面再补。
        self._send(Command.GRAB, angle)

    def camera(self, angle:int):
        """相机俯角，单位度；0水平，向下为正。"""
        # 0~80 度，serial_node 还会按 max_pitch_deg 再检查一次。
        self._send(Command.CAMERA, angle)
