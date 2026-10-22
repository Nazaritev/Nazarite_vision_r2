# 本帧采集时间；与 session_id 一起避免旧检测结果影响新一轮控制。
builtin_interfaces/Time stamp
uint64 session_id
# 只有成功读取并推理的图像才会发布本消息。
bool detected
# 检测框中心的横、纵像素坐标，不是相机焦距；无目标时为 -1。
float32 fx
float32 fy
# 原图中的轴对齐检测框；无目标时四项均为 -1。
float32 x1
float32 y1
float32 x2
float32 y2
uint32 image_width
uint32 image_height
