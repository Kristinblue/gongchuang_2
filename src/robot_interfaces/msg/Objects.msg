#一组物块的描述
#带时间戳，就能知道是哪一帧的了
std_msgs/Header header
robot_interfaces/Object[] items

#Header 本身也是一个消息结构，主要有两个字段：
#builtin_interfaces/Time stamp
#string frame_id
