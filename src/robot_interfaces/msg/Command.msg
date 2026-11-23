# command：动作类型，使用下面定义的具名常量
# value：动作参数，不同命令的单位不同
# STRAIGHT、TURN 为像素误差
# TURN_ANGLE、CAMERA 为角度
# STRAIGHT_DIS 为厘米

uint8 command
int16 value #最大32767

# 动作命令位的取值，和 communication_pkg/my_cmd.py 对应。
# 这些是消息常量，只参与编译，不会进串口帧，下位机协议不变。最终发送的是规定的字节流
# 电脑设置红蓝方吧，协议里先留空
uint8 RESERVED=0

# 正数前进，负数后退。香橙派算完误差之后取负数再发过去。正数往前更符合直觉
uint8 STRAIGHT=1

# 正右负左
uint8 TURN=2

# 爪子有限位，需映射角度，暂且先弄最大和最小吧
# 之后应该还要在单片机里面加对角度的判断，有些角度抓取物块不能完全闭合，应该是检测堵转或者实际角度？
uint8 GRAB=3 # 0为开，1为关

# 摄像头角度：0度水平，向下为正；当前限位下整数指令范围0–80度。
uint8 CAMERA=4

# 角度旋转，以当前方向为基准，正数为右转角度，顺时针，负数左转
uint8 TURN_ANGLE=5

# 绝对角度校准，利用安全区的，之后具体规定
uint8 ANGLE_CALIB=6

# 直线行驶，单位cm，正数前进，负数后退。
# 可能是需要具体距离的，因为我想通过这个快速调整接近安全区的姿态
uint8 STRAIGHT_DIS=7
uint8 STOP=8
