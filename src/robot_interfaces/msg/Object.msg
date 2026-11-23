#对单个目标的描述
string name
int32 id
float32 score

float32 x1
float32 y1
float32 x2
float32 y2
bool observed
bool in_safe_zone

#我们需要减少跳变，能稳定追踪一个特定物块，虽然YOLO有，但还是想自己定义这些，会更清晰可控一些
#所以我们要对每个物体标id