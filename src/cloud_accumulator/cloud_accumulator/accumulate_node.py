#!/usr/bin/env python3
"""在 map 系下按 ROI 累积点云，并保证内存有界。

数据流::

    /cloud_registered (odom 系)
        --TF map<-odom-->        map 系
        --多个 ROI 盒子筛选-->   只保留关心的区域
        --体素键去重-->          每个体素只存一个点
        --定时发布-->            /cloud_accumulated (map 系)

为什么内存有界
--------------
1. ROI 是有限的，体素数量正比于 ROI 内可见表面面积 / voxel_size^2；
2. 同一体素被反复观测只更新 last_seen，不会新增条目；
3. 因此体素数**不随运行时间增长**，即使 decay_time=0（不淘汰）也不会爆内存。
   max_voxels 只是最后一道保险，正常配置下不应该触发。

“有出”的两种方式
----------------
1. decay_time > 0：超过该时间没有被再次观测到的体素会被删除，用来清掉
   行人、移动物体留下的残影；静态场地可以设 0 关掉。
2. 运行中把 ROI 改小：参数回调里会立即删掉新 ROI 之外的点。

坐标系
------
输出话题的 frame_id 固定是 ``target_frame``（默认 map）。输入点云必须先通过
TF 变换到该坐标系再判断 ROI；**拿不到 TF 就整帧丢弃**，绝不能用 odom 坐标
冒充 map 坐标，否则会安静地把错误数据写进累积图。

注意：map<-odom 由 AMCL 提供。AMCL 重定位时可能一次跳变几十厘米到几米，
这种情况下旧体素和新点云会错位、出现重影，所以节点会检测跳变并清空累积图
（阈值见 jump_translation / jump_yaw_deg）。
"""

import heapq
import math
import threading

import numpy as np

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener


# ---------------------------------------------------------------------------
# 纯函数工具
# ---------------------------------------------------------------------------

def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 3) float32 数组，自动去掉 NaN / Inf。

    按 point_step 和字段 offset 建视图，不做逐点 Python 循环，40 Hz 下开销很小。
    不要换成 sensor_msgs_py.read_points：它是生成器，逐点处理会拖慢整个回调。
    """
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)

    offsets = {f.name: f.offset for f in msg.fields}
    for name in ('x', 'y', 'z'):
        if name not in offsets:
            raise ValueError(f'PointCloud2 缺少字段 {name}')

    # 大端数据要显式声明，否则读出来是乱码（x86 雷达通常是小端）。
    dtype = np.dtype('>f4') if msg.is_bigendian else np.dtype('<f4')
    cols = [
        np.ndarray(n, dtype=dtype, buffer=msg.data,
                   offset=offsets[name], strides=msg.point_step)
        for name in ('x', 'y', 'z')
    ]
    pts = np.column_stack(cols).astype(np.float32, copy=False)
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def quat_to_rot(q) -> np.ndarray:
    """TF 四元数 -> 3x3 旋转矩阵。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('TF 里的四元数长度为 0')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_of(rot: np.ndarray) -> float:
    """旋转矩阵的偏航角（绕 z 轴），用于跳变检测。"""
    return math.atan2(rot[1, 0], rot[0, 0])


def wrap_angle(angle: float) -> float:
    """把角度折回 (-pi, pi]，用于比较两个 yaw 的差值。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def voxel_keys(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """把点量化成体素键。

    每轴 21 位，拼进 int64：x 占高 21 位，y 占中 21 位，z 占低 21 位。
    21 位配合 0.05 m 体素可以覆盖 ±52 km，完全够用。

    注意：先转 int64 再按位与 0x1FFFFF。直接用 int32 左移会把符号位移进高位，
    产生错误的键。
    """
    ijk = np.floor(pts / voxel_size).astype(np.int64) & 0x1FFFFF
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------

class CloudAccumulator(Node):

    def __init__(self):
        super().__init__('cloud_accumulator')

        # ---------------- 参数 ----------------
        # 话题
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/cloud_accumulated')
        # 累积/发布所在的坐标系。输入点云会先变换到这里，ROI 也按这个系解释。
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('tf_timeout', 0.2)

        # 体素/内存
        self.declare_parameter('voxel_size', 0.05)      # 体素边长 (m)
        self.declare_parameter('decay_time', 0.0)       # 0 = 不按时间淘汰
        self.declare_parameter('max_voxels', 100000)    # 兜底上限
        self.declare_parameter('publish_rate', 2.0)     # 输出频率 (Hz)
        self.declare_parameter('process_every_n', 1)    # 每 N 帧处理一次，省 CPU

        # ROI：两种写法，任选其一
        #   1) roi_boxes = [xmin, xmax, ymin, ymax, zmin, zmax, ...]（每 6 个数一个盒子）
        #   2) roi_centers = [x1, y1, x2, y2, ...] + roi_half_size + roi_z_min/z_max
        # 都为空 => 不筛选（不推荐，内存就不再被 ROI 约束了）。
        #
        # 注意：这里默认值必须写 None，不能写空列表 []。
        # rclpy 会把 [] 推断成 BYTE_ARRAY，之后用 double 数组覆盖会报类型错误。
        # 写 None + 显式 descriptor 时，未传参数的节点里该参数是“未初始化”，
        # 读取时用 _get_roi_value() 兜底成空列表。
        double_array = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('roi_boxes', None, double_array)
        self.declare_parameter('roi_centers', None, double_array)
        self.declare_parameter('roi_half_size', 0.5)
        self.declare_parameter('roi_z_min', -0.75)
        self.declare_parameter('roi_z_max', 1.5)

        # AMCL 跳变检测：平移/偏航超过阈值就清空累积图
        self.declare_parameter('jump_translation', 0.5)  # m
        self.declare_parameter('jump_yaw_deg', 10.0)     # deg

        # ---------------- 内部状态 ----------------
        self._lock = threading.Lock()          # 保护 _voxels 和下面的缓存参数
        # key -> (x, y, z, last_seen_sec)。用 tuple 省内存；更新时整体替换。
        self._voxels = {}
        self._last_tf = None                   # 上一帧 (tx, ty, yaw)，用于跳变检测
        self._output_frame = str(self.get_parameter('target_frame').value)
        self._frame_counter = 0
        self._tf_skips = 0

        self._refresh_cached_params()
        self._roi_boxes = self._build_roi_boxes()

        # ---------------- 通信 ----------------
        # 订阅端和发布定时器放在同一个互斥回调组里：两者不会并发碰字典，
        # 但 TF 监听器在另一个回调组，MultiThreadedExecutor 仍能及时处理 /tf。
        # 如果以后想让发布不阻塞接收，可以换成 Reentrant + 已存在的锁。
        self._cb_group = MutuallyExclusiveCallbackGroup()

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        # 输入 QoS 与 fast_lio 的默认可靠发布兼容；depth 小一点，堵住时丢旧帧。
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self.on_cloud, sub_qos,
            callback_group=self._cb_group)

        # 发布 QoS：可靠发布能同时兼容可靠/尽力而为的订阅者。
        # depth=1：下游慢时只要最新的一帧，避免队列堆积（"有出"的一部分）。
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(PointCloud2, output_topic, pub_qos)

        # TF：缓存 10 秒，足够覆盖消息时间戳和 AMCL 的发布延迟
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # TransformListener 会在本节点上创建 /tf 订阅，由 MultiThreadedExecutor 驱动
        self._tf_listener = TransformListener(self._tf_buffer, self)

        publish_rate = max(0.1, float(self.get_parameter('publish_rate').value))
        self._timer = self.create_timer(
            1.0 / publish_rate, self.on_publish_timer,
            callback_group=self._cb_group)

        # 运行时改参数时重新加载 ROI、体素尺寸等
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'cloud_accumulator 启动: {input_topic} -> {output_topic}, '
            f'frame={self._target_frame}, voxel={self._voxel_size} m, '
            f'ROI 盒子数={len(self._roi_boxes)}, decay={self._decay_time} s')

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------

    def _refresh_cached_params(self) -> None:
        """把热路径要用的参数缓存成属性，避免每帧查参数表。"""
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._voxel_size = max(1e-3, float(self.get_parameter('voxel_size').value))
        self._decay_time = max(0.0, float(self.get_parameter('decay_time').value))
        self._max_voxels = max(1, int(self.get_parameter('max_voxels').value))
        self._process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self._tf_timeout = Duration(seconds=max(0.0, float(self.get_parameter('tf_timeout').value)))
        self._jump_translation = max(0.0, float(self.get_parameter('jump_translation').value))
        self._jump_yaw = math.radians(max(0.0, float(self.get_parameter('jump_yaw_deg').value)))

    def _build_roi_boxes(self, proposed=None) -> np.ndarray:
        """读参数，返回 (K, 6) 的 ROI 数组；每行是 xmin,xmax,ymin,ymax,zmin,zmax。

        proposed 是参数回调里尚未生效的新值（name -> value），用于合并预览。
        如果 roi_boxes 非空优先用它，否则用 roi_centers 展开成盒子。
        """
        def get(name):
            if proposed is not None and name in proposed:
                return proposed[name]
            try:
                value = self.get_parameter(name).value
            except ParameterUninitializedException:
                # 没在 launch / 命令行里传这个参数：当作空数组
                return []
            return value if value is not None else []

        raw = list(get('roi_boxes') or [])
        if raw:
            if len(raw) % 6 != 0:
                raise ValueError('roi_boxes 的长度必须是 6 的倍数')
            return np.asarray(raw, dtype=np.float64).reshape(-1, 6)

        centers = list(get('roi_centers') or [])
        if not centers:
            return np.empty((0, 6), dtype=np.float64)

        if len(centers) % 2 != 0:
            raise ValueError('roi_centers 必须是偶数个 [x1,y1,x2,y2,...]')
        half = float(get('roi_half_size'))
        z_min = float(get('roi_z_min'))
        z_max = float(get('roi_z_max'))
        if half <= 0.0:
            raise ValueError('roi_half_size 必须大于 0')
        if z_min >= z_max:
            raise ValueError('roi_z_min 必须小于 roi_z_max')

        c = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        return np.column_stack([
            c[:, 0] - half, c[:, 0] + half,
            c[:, 1] - half, c[:, 1] + half,
            np.full(len(c), z_min), np.full(len(c), z_max),
        ])

    def _on_set_parameters(self, params):
        """运行时修改参数：校验 -> 应用到缓存 -> 必要时清理已有体素。"""
        proposed = {p.name: p.value for p in params}

        # 先校验，失败就整批拒绝，不留半生效状态
        try:
            if 'voxel_size' in proposed and float(proposed['voxel_size']) <= 0.0:
                raise ValueError('voxel_size 必须大于 0')
            if 'decay_time' in proposed and float(proposed['decay_time']) < 0.0:
                raise ValueError('decay_time 不能为负')
            if 'max_voxels' in proposed and int(proposed['max_voxels']) <= 0:
                raise ValueError('max_voxels 必须大于 0')
            if 'publish_rate' in proposed and float(proposed['publish_rate']) <= 0.0:
                raise ValueError('publish_rate 必须大于 0')
            if 'process_every_n' in proposed and int(proposed['process_every_n']) <= 0:
                raise ValueError('process_every_n 必须大于 0')
            new_boxes = self._build_roi_boxes(proposed)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        roi_keys = ('roi_boxes', 'roi_centers', 'roi_half_size', 'roi_z_min', 'roi_z_max')

        with self._lock:
            self._roi_boxes = new_boxes
            # ROI 变了：把已经不在新 ROI 里的点立刻删掉
            if any(k in proposed for k in roi_keys):
                self._purge_outside_locked()

            # 体素键是按旧尺寸算的，改尺寸后旧键全部失效，只能清空重来
            if 'voxel_size' in proposed:
                self._voxel_size = float(proposed['voxel_size'])
                self._voxels.clear()

            # 换坐标系后旧点也失效
            if 'target_frame' in proposed and str(proposed['target_frame']) != self._target_frame:
                self._target_frame = str(proposed['target_frame'])
                self._output_frame = self._target_frame
                self._last_tf = None
                self._voxels.clear()

            if 'decay_time' in proposed:
                self._decay_time = float(proposed['decay_time'])
            if 'max_voxels' in proposed:
                self._max_voxels = int(proposed['max_voxels'])
            if 'process_every_n' in proposed:
                self._process_every_n = int(proposed['process_every_n'])

        # 发布频率变了要重建定时器（放在锁外面，避免在持锁时做通信操作）
        if 'publish_rate' in proposed:
            rate = float(proposed['publish_rate'])
            self.destroy_timer(self._timer)
            self._timer = self.create_timer(
                1.0 / rate, self.on_publish_timer, callback_group=self._cb_group)

        self.get_logger().info(
            f'参数已更新: ROI 盒子数={len(self._roi_boxes)}, 体素={self._voxel_size} m')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # ROI
    # ------------------------------------------------------------------

    def _mask_in_rois(self, pts: np.ndarray) -> np.ndarray:
        """返回每个点是否落在任意一个 ROI 盒子内（左闭右开）。

        ROI 为空时返回全 True。这里用向量化比较，几个盒子的开销可以忽略。
        """
        if len(self._roi_boxes) == 0:
            return np.ones(len(pts), dtype=bool)

        mask = np.zeros(len(pts), dtype=bool)
        for xmin, xmax, ymin, ymax, zmin, zmax in self._roi_boxes:
            mask |= ((pts[:, 0] >= xmin) & (pts[:, 0] < xmax) &
                     (pts[:, 1] >= ymin) & (pts[:, 1] < ymax) &
                     (pts[:, 2] >= zmin) & (pts[:, 2] < zmax))
        return mask

    def _purge_outside_locked(self) -> None:
        """删掉当前不在 ROI 内的体素。调用方必须持有 self._lock。"""
        if len(self._roi_boxes) == 0 or not self._voxels:
            return
        items = list(self._voxels.items())
        pts = np.array([[v[0], v[1], v[2]] for _, v in items], dtype=np.float64)
        keep = self._mask_in_rois(pts)
        for (key, _), is_inside in zip(items, keep):
            if not is_inside:
                del self._voxels[key]

    # ------------------------------------------------------------------
    # 输入：累积
    # ------------------------------------------------------------------

    def _lookup_tf(self, target_frame: str, source_frame: str, msg: PointCloud2):
        """查 target <- source 的 TF，优先用消息时间戳。"""
        has_stamp = msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
        requested = Time.from_msg(msg.header.stamp) if has_stamp else Time()
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, requested, timeout=self._tf_timeout)
        except ExtrapolationException:
            # 消息时间戳偶尔超前于 TF 缓存时，退回“最新可用”再试一次。
            if has_stamp:
                return self._tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(), timeout=self._tf_timeout)
            raise

    def on_cloud(self, msg: PointCloud2) -> None:
        self._frame_counter += 1
        # 每 N 帧处理一次，用来降 CPU；累积效果几乎不变
        if self._process_every_n > 1 and (self._frame_counter % self._process_every_n) != 0:
            return

        try:
            pts = cloud_to_xyz(msg)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'点云解析失败: {exc}', throttle_duration_sec=5.0)
            return
        if len(pts) == 0:
            return

        # 用消息时间戳（秒）作为“最后一次看到”的时间。stamp 为 0 时退回节点时钟。
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        src_frame = msg.header.frame_id
        target = self._target_frame

        if target and target != src_frame:
            # ---- 1) 变换到目标坐标系（map） ----
            try:
                tf = self._lookup_tf(target, src_frame, msg)
            except TransformException:
                # 拿不到 TF 就丢掉整帧，绝不能拿 odom 坐标当 map 坐标存
                self._tf_skips += 1
                self.get_logger().warn(
                    f'缺少 TF {target} <- {src_frame}，跳过该帧 '
                    f'(累计 {self._tf_skips})',
                    throttle_duration_sec=5.0)
                return

            try:
                rot = quat_to_rot(tf.transform.rotation)
                trans = tf.transform.translation
            except ValueError as exc:
                self.get_logger().error(f'TF 无效: {exc}', throttle_duration_sec=5.0)
                return

            # ---- 2) 跳变检测（AMCL 重定位） ----
            yaw = yaw_of(rot)
            if self._last_tf is not None:
                dx = trans.x - self._last_tf[0]
                dy = trans.y - self._last_tf[1]
                dyaw = wrap_angle(yaw - self._last_tf[2])
                if (math.hypot(dx, dy) > self._jump_translation or
                        abs(dyaw) > self._jump_yaw):
                    with self._lock:
                        self._voxels.clear()
                    self.get_logger().warn(
                        f'检测到 {target}<-{src_frame} 跳变 '
                        f'(平移 {math.hypot(dx, dy):.2f} m, '
                        f'yaw {math.degrees(dyaw):.1f} deg)，清空累积图',
                        throttle_duration_sec=2.0)
            self._last_tf = (trans.x, trans.y, yaw)

            # ---- 3) 点云 -> map 系 ----
            pts = pts @ rot.T + np.array([trans.x, trans.y, trans.z],
                                         dtype=np.float64)
            self._output_frame = target
        else:
            # 输入本来就在目标系（或 target_frame 为空）：不做变换
            pts = pts.astype(np.float64, copy=False)
            self._output_frame = src_frame

        # ---- 4) ROI 筛选：必须在算体素键和入库之前做，否则内存省不下来 ----
        pts = pts[self._mask_in_rois(pts)]
        if len(pts) == 0:
            return

        # ---- 5) 体素去重 + 入库 ----
        keys = voxel_keys(pts, self._voxel_size)
        # 同一帧内落到同一体素的点只保留一个，减少字典操作
        unique_keys, first_index = np.unique(keys, return_index=True)

        with self._lock:
            voxels = self._voxels
            for key, idx in zip(unique_keys.tolist(), first_index.tolist()):
                x, y, z = pts[idx]
                # 已存在的体素只“刷新”位置和时间，不会增加条目
                voxels[int(key)] = (float(x), float(y), float(z), stamp_sec)

    # ------------------------------------------------------------------
    # 输出：淘汰 + 发布
    # ------------------------------------------------------------------

    def on_publish_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        with self._lock:
            # 1) 时间淘汰：超过 decay_time 没再被看到的体素删掉
            if self._decay_time > 0.0:
                deadline = now_sec - self._decay_time
                expired = [k for k, v in self._voxels.items() if v[3] < deadline]
                for key in expired:
                    del self._voxels[key]

            # 2) 兜底上限：只有在参数配得不合理时才会触发
            if len(self._voxels) > self._max_voxels:
                excess = len(self._voxels) - self._max_voxels
                oldest = heapq.nsmallest(
                    excess, self._voxels.items(), key=lambda kv: kv[1][3])
                for key, _ in oldest:
                    del self._voxels[key]
                self.get_logger().warn(
                    f'体素数超过 max_voxels={self._max_voxels}，'
                    f'已删除最旧的 {excess} 个（检查 voxel_size / ROI / decay_time）',
                    throttle_duration_sec=5.0)

            # 3) 在锁内做快照，锁外再序列化发布，减少阻塞入库
            values = list(self._voxels.values())

        if values:
            # values 是 (x, y, z, t) 的列表，转成 (N, 4) float32
            pts = np.asarray(values, dtype=np.float32)[:, :3]
        else:
            pts = np.empty((0, 3), dtype=np.float32)

        self._publish(pts)

        self.get_logger().info(
            f'发布 {len(pts)} 点, 体素={len(values)}, TF 跳过={self._tf_skips}',
            throttle_duration_sec=10.0)

    def _publish(self, pts: np.ndarray) -> None:
        """把 (N, 3) 点数组组装成 PointCloud2 并发布。

        手动组装而不用点云工具库：numpy 的 tobytes() 是一次内存拷贝，
        比把 30 万个 Python 元组交给 create_cloud 快得多。

        point_step 用 16 而不是 12：多留 4 字节 padding，PCL 和大多数消费端
        都按 16 字节对齐处理，兼容性最好。
        """
        n = len(pts)
        packed = np.zeros((n, 4), dtype=np.float32)  # xyz + 4 字节 padding
        packed[:, :3] = pts

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._output_frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = packed.tobytes()

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulator()
    # 多线程：TF 订阅和我们的点云回调可以并行处理
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
