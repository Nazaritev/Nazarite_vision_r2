# yolo_detect_box

ROS 2 Humble 独立功能包，直接使用 `cv2.VideoCapture(camera_index)` 读取普通摄像头，摄像头编号默认 `0`；使用 Ultralytics `.pt` 检测模型，固定 CPU 推理。

## 模型和依赖

将训练得到的单类别检测模型命名为 `best.pt`，放在本包的 `model/` 目录，**然后编译**。
CMake 会把整个 `model/` 安装到 `share/yolo_detect_box/model/`，节点通过 ROS 包索引查找模型，不写死电脑路径。
模型更换后需要重新编译。启动时加载模型，收到服务请求前不进行推理。

目标电脑需要 ROS 2 Humble、OpenCV、PyTorch 和 Ultralytics。
先在准备启动 launch 的 Python 环境中检查依赖，已有可用环境可以直接复用：

```bash
source /opt/ros/humble/setup.bash
python3 -c "import sys, rclpy, cv2, torch, ultralytics; print(sys.executable); print(ultralytics.__file__)"
```

如果该环境缺少 Ultralytics，再安装：

```bash
python3 -m pip install -r src/yolo_detect_box/requirements.txt
```

有 PyTorch 即可运行 CPU 推理，不需要 CUDA。使用显示窗口，运行环境需要图形桌面。

## 当前电脑的实测结果

2026-10-08 已完成真实摄像头和模型的服务流程测试：

- Python 环境：`/home/tomato/miniconda3/envs/yolo-dev`，Python `3.10.21`。环境名是 `yolo-dev`（中间为短横线）。
- Ultralytics：`8.3.161`，源码位置为 `/home/tomato/code/yolo_official/ultralytics_src`。
- PyTorch：`2.7.1+cu128`；节点指定 `device='cpu'`，实际使用 CPU 推理。
- 模型来源：`/home/tomato/code/box_seg_annotation/runs/blue_kfs_detect_20261008_150ep_p50/weights/best.pt`。
- 模型已复制到本包 `model/best.pt`，类别为 `blue_kfs`；编译后已安装，源文件、包内副本和安装副本的 SHA-256 一致。
- 摄像头 0 读取正常，CPU 推理正常，launch 能启动并待机。
- 实际检测到目标后，连续超过 1 秒无目标，窗口关闭，服务返回完成状态（当前接口为整数 `success=1`）。

## 编译

在 ROS 工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_detect_box \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

编译时指定系统 Python 来生成 ROS 接口；运行检测节点时使用下面激活的 YOLO 环境。

## 当前电脑的启动命令

在任意终端执行：

```bash
source /home/tomato/miniconda3/etc/profile.d/conda.sh
conda activate yolo-dev
source /opt/ros/humble/setup.bash
source /home/tomato/code/Nazarite_vision_r2/install/yolo_detect_box/share/yolo_detect_box/local_setup.bash

ros2 launch yolo_detect_box yolo_detect.launch.py
```

节点首行是 `#!/usr/bin/env python3`，使用启动终端中 `PATH` 找到的 `python3`；因此必须先激活上述环境。
这里只加载本功能包的安装环境，避免整个工作空间内其他旧包的失效安装路径影响启动。

launch 只启动检测节点，节点直接打开指定摄像头，不需要另外启动相机 ROS 驱动。
不传 `camera_index` 时默认使用摄像头 `0`；例如使用摄像头 `1`：

```bash
ros2 launch yolo_detect_box yolo_detect.launch.py camera_index:=1
```

摄像头在节点退出时释放。默认检测置信度阈值为 `0.6`，每帧最多保留置信度最高的一个框。
显示和目标出现/丢失判断都使用过滤后的结果；仅限制框数量不能保证消除高置信度误检。

如果仍有误检，可以在启动时进一步提高阈值（提高后也可能漏掉真实目标）：

```bash
ros2 launch yolo_detect_box yolo_detect.launch.py conf:=0.75
```

两个参数可以一起传入：

```bash
ros2 launch yolo_detect_box yolo_detect.launch.py camera_index:=1 conf:=0.75
```

修改脚本或 launch 后需重新编译，并重启节点才能生效。

## 服务调用

接口：`yolo_detect_box/srv/CameraUse`，服务名：`/camara_use`。

```srv
int32 use
uint64 session_id
---
int32 success
float32 fx
float32 fy
```

当前电脑另开终端执行（服务客户端使用系统 ROS Python 即可）：

```bash
source /opt/ros/humble/setup.bash
source /home/tomato/code/Nazarite_vision_r2/install/yolo_detect_box/share/yolo_detect_box/local_setup.bash
ros2 service call /camara_use yolo_detect_box/srv/CameraUse "{use: 1}"
```

1. launch 启动后打开摄像头并待机，不读取画面，不显示窗口。
2. 收到 `use=1` 后开始检测并显示检测框；客户端继续等待响应。
3. 首次检测到目标前，无论等待多久都不结束。
4. 首次检测到目标后，从第一张无目标图像开始计时。再次检测到目标则清除计时。
5. 后续无目标图像确认连续丢失超过 1 秒，关闭窗口、停止检测，服务返回 `success=1`。
6. 回到待机，可以再次发送 `use=1` 开始下一轮。

计时使用单调时钟；读取摄像头失败时继续等待，不把失败当作无目标图像。
其他 `use` 值以及检测期间的重复请求返回 `success=0`。
此服务没有初始等待超时，客户端应使用异步调用并持续 spin，避免阻塞客户端的其他回调。

`success` 是整数状态：`1` 表示本轮正常完成，`0` 表示请求被拒绝或检测失败。
控制节点明确判断 `success == 1`，其他状态同样停车，但不视为正常完成。
接口类型由布尔值改为 `int32` 后，服务端和客户端都需要重新编译并重启。
`session_id` 是本轮编号；上面的手动调用不填时默认为 `0`。服务响应的 `fx`、`fy`
是最后一次有效检测框中心的原图像素坐标，未识别到目标时为 `-1`，不是相机焦距。
实时坐标另外发布到 `/yolo/box_detection`（`yolo_detect_box/msg/BoxDetection`），
包含本轮编号、采集时间、是否检测到目标及图像宽高；无目标帧的坐标为 `-1`。

## /slope 触发的串口横移控制

新增的控制节点位于 `run_keyboard_test/run_keyboard_test/yolo_control.py`，
串口协议位于同目录的 `serial_velocity.py`。默认直接写 `/dev/ttyUSB0`，
波特率沿用现有 `nav_to_pose.run` 的 `961200`。

流程：

1. 等待外部向 `/slope`（`tur3_run_interfaces/msg/Slope`）发送 `slope=3`。
   `1` 和 `2` 保留原有上坡含义，不触发 YOLO。
2. 打开串口并先发零速度，异步调用 `/camara_use`，发送 `use=1` 和本轮编号。
   待机不打开串口；运行中重复收到 `3` 会被忽略。
3. 首次识别前保持零速度。识别后只调节横移 `Vy`，`Vx`、`Vw` 始终为 `0`。
   使用图像实际宽度，不写死分辨率。
4. 框中心满足 `0.4 * image_width <= fx <= 0.6 * image_width` 即到位，含边界。
   例如宽度 `1080`，范围是 `[432, 648]` 像素。
5. **每帧实时判断中心位置**：在范围内以默认 `20 Hz` 持续发送零速度；
   一旦移出范围，就继续按照偏差横移追踪，再次进入范围时重新停车。
   整个过程持续到方块被拿走，不需要重新发送 `slope=3`。
6. 首次识别后，连续 `missing_timeout` 秒无目标，服务返回 `success=1`。
   控制节点收到响应后继续发送 `stop_duration` 秒零速度，再关闭串口、回到待机。
   单纯进入中心范围不会返回成功。

检测短暂丢失、结果过期时先发零速度；摄像头读图失败不算方块被拿走，
连续失败达到 `camera_timeout` 时服务返回 `success=0`，控制节点同样停车。
服务和话题使用相同的 `session_id`，其他客户端的检测结果不会驱动本轮控制。

速度协议为 12 字节小端帧：

```text
0xA5 | Vx(int16) | Vy(int16) | Vw(int16) | Flag | Reserved | Mode | CRC8 | 0xFF
```

`Vy` 的单位转换为 `int(Vy * 1000)`；三个标志字节都为 `0`，
CRC8 使用多项式 `0x07`、初值 `0`，校验帧头之后的 9 字节。
控制逻辑和串口编码都限制 `|Vy| <= 0.3 m/s`，参数调大也不能突破。
只保留一个速度上限参数 `max_lateral_speed`，默认 `0.3 m/s`。
范围外的速度大小为 `min(lateral_gain * abs((fx - 目标中心) / image_width), max_lateral_speed)`，
默认 `lateral_gain=1.0`：偏差较大时能达到 `0.3 m/s`，偏差变小时速度随之降低。
例如宽度 `1080` 时，`fx=100` 对应 `Vy=+0.3 m/s`，
`fx=350` 对应约 `+0.176 m/s`，`fx=540` 在停车范围内、速度为 `0`。

### 编译和启动

接口已经改变，需要重新编译并重启服务端、客户端。控制节点使用 pyserial；
若系统 Python 没有安装，可安装发行版的 `python3-serial`，或为实际运行解释器安装 `pyserial`。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tur3_run_interfaces yolo_detect_box run_keyboard_test \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

在拥有 Ultralytics 的环境中运行：

```bash
source /home/tomato/miniconda3/etc/profile.d/conda.sh
conda activate yolo-dev
source /opt/ros/humble/setup.bash
source /home/tomato/code/Nazarite_vision_r2/install/setup.bash
ros2 launch run_keyboard_test yolo_control.launch.py camera_index:=0 max_lateral_speed:=0.3
```

这个 launch 会同时启动检测和控制，不需要再单独启动 YOLO。
请确保 `get_and_pub`、`nav_to_pose`、串口启动器或其他程序没有同时向该串口发送指令。
`exclusive=True` 用于协作进程间的串口占用检查，但无法替代对旧节点的统一管理。

另开终端触发：

```bash
source /opt/ros/humble/setup.bash
source /home/tomato/code/Nazarite_vision_r2/install/setup.bash
ros2 topic pub --once /slope tur3_run_interfaces/msg/Slope "{slope: 3}"
```

参数均可通过 `名称:=值` 传入：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `serial_port` / `baudrate` | `/dev/ttyUSB0` / `961200` | 串口及波特率 |
| `max_lateral_speed` | `0.3` | 唯一的最大横移速度参数，m/s，不能突破 `0.3` |
| `lateral_gain` | `1.0` | 归一化水平偏差对应的速度增益 |
| `center_left_ratio` / `center_right_ratio` | `0.4` / `0.6` | 到位范围占图像宽度的比例 |
| `lateral_direction` | `-1.0` | 图像偏差到 Vy 的符号；镜像安装可改为 `1.0` |
| `control_rate` | `20.0` | 包含保持停车时的发帧频率，Hz |
| `detection_timeout` | `0.5` | 过期检测结果的停车阈值，秒 |
| `missing_timeout` | `1.0` | 首次识别后连续丢失的完成阈值，秒 |
| `camera_timeout` | `2.0` | 连续读图失败的失败阈值，秒 |
| `service_wait_timeout` | `5.0` | 等待服务上线的时长，秒 |
| `stop_duration` | `0.5` | 服务结束后继续发零速度的时长，秒 |
| `camera_index` / `conf` | `0` / `0.6` | 摄像头编号及检测阈值 |
| `show_window` | `true` | 是否显示窗口；无桌面时设为 `false` |
| `slope_topic` / `trigger_value` | `/slope` / `3` | 外部启动条件 |
| `detection_topic` | `/yolo/box_detection` | 实时检测结果话题 |

### 不连接实车的测试

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/run_keyboard_test:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q src/run_keyboard_test/test/test_yolo_control.py
```

测试替换串口和模型，验证中心范围内停车、移出范围恢复追踪、丢失判定、重复触发、
服务成功/失败、结果过期、串口编码与速度硬限幅，不打开真实串口。

## 换电脑使用

复制整个功能包（包括 `model/best.pt`）到目标工作空间的 `src/`，在目标电脑重新编译。
运行时激活目标电脑已有的兼容 ROS 和 YOLO 的 Python 环境，加载 ROS 和目标工作空间的安装环境，再运行 launch。
环境名不必叫 `yolo-dev`，上面的绝对路径需替换为目标电脑的实际路径。

先确认该环境能导入 `rclpy`、`cv2`、`torch`、`ultralytics`，然后测试指定摄像头、CPU 推理和完整服务返回。
使用相同模型和 Ultralytics 源码版本更便于复现；若模型训练涉及自定义源码，也需携带相同源码并安装到目标环境。
`requirements.txt` 当前只声明 Ultralytics，未固定版本，不会自动创建或激活 Python 环境。
