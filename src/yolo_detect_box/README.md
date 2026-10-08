# yolo_detect_box

ROS 2 Humble 独立功能包，直接使用 `cv2.VideoCapture(0)` 读取普通摄像头，使用 Ultralytics `.pt` 检测模型，固定 CPU 推理。

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
- 实际检测到目标后，连续超过 1 秒无目标，窗口关闭，服务返回 `CameraUse_Response(success=True)`。

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

launch 只启动检测节点，节点直接打开摄像头 0，不需要另外启动相机 ROS 驱动。
摄像头在节点退出时释放。默认检测置信度阈值为 `0.25`。

## 服务调用

接口：`yolo_detect_box/srv/CameraUse`，服务名：`/camara_use`。

```srv
int32 use
---
bool success
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
5. 后续无目标图像确认连续丢失超过 1 秒，关闭窗口、停止检测，服务返回 `success=true`。
6. 回到待机，可以再次发送 `use=1` 开始下一轮。

计时使用单调时钟；读取摄像头失败时继续等待，不把失败当作无目标图像。
其他 `use` 值以及检测期间的重复请求返回 `success=false`。
此服务没有初始等待超时，客户端应使用异步调用并持续 spin，避免阻塞客户端的其他回调。

## 换电脑使用

复制整个功能包（包括 `model/best.pt`）到目标工作空间的 `src/`，在目标电脑重新编译。
运行时激活目标电脑已有的兼容 ROS 和 YOLO 的 Python 环境，加载 ROS 和目标工作空间的安装环境，再运行 launch。
环境名不必叫 `yolo-dev`，上面的绝对路径需替换为目标电脑的实际路径。

先确认该环境能导入 `rclpy`、`cv2`、`torch`、`ultralytics`，然后测试摄像头 0、CPU 推理和完整服务返回。
使用相同模型和 Ultralytics 源码版本更便于复现；若模型训练涉及自定义源码，也需携带相同源码并安装到目标环境。
`requirements.txt` 当前只声明 Ultralytics，未固定版本，不会自动创建或激活 Python 环境。
