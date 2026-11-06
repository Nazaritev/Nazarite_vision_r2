#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ============================================================
// 堆叠检测（map 坐标系查询版）
//
//   订阅 /cloud_registered（odom 系点云），先用 TF 变到 map 系，
//   然后在 map 系下若干个查询点（points 参数，二维 [x,y] 对）处，
//   各取一个竖直方柱 ROI，统计堆叠高度和层数。
//
//   雷达物理倒装（URDF 里 base→imu pitch=π），但 laserMapping 发布 odom 时
//   已把它对齐到启动时的 base_link（见 laserMapping.cpp 的
//   published_odom_alignment），/cloud_registered 和 map 的 z 轴都是朝上的：
//   地面在 z = ground_z（2026-11-04 实测约 -0.73m），高度 h = z - ground_z。
//   ★ 换场地 / 换车必须重新标定 ground_z（RViz 里 Publish Point 点地面）。
//
//   层数规则：按每层中线判决，
//   layers = floor((h + layer_height/2) / layer_height)，
//   即 h≈一个块高时算 1 层、≈两个块高时算 2 层。
//
//   可视化（全部在 map 系）：
//     /block_stacking/markers       查询点 ROI 框 + 堆叠顶面 + 层数文字
//     /block_stacking/corner_cloud  ROI 内点云（绿=有堆叠，蓝=没有）
//     /block_stacking/layers        各查询点层数 Int32MultiArray
// ============================================================

// 一个查询点（map 坐标系下的 xy，竖直方向由 ground_z 决定）
struct QueryPoint {
    std::string name;
    float x, y;
};

// 单个查询点的检测结果
struct StackResult {
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
    bool has_object = false;
    int layers = 0;
    float height = 0.0f;    // 堆叠高度（米，地面以上）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

// 计算分位数（0~1），输入已排序
float percentile(const std::vector<float>& sorted_vals, float p)
{
    if (sorted_vals.empty()) return 0.0f;
    float idx = p * (sorted_vals.size() - 1);
    int lo = static_cast<int>(std::floor(idx));
    int hi = static_cast<int>(std::ceil(idx));
    if (lo == hi) return sorted_vals[lo];
    float w = idx - lo;
    return sorted_vals[lo] * (1.0f - w) + sorted_vals[hi] * w;
}

class BlockStackingDetection : public rclcpp::Node
{
public:
    BlockStackingDetection()
    : Node("block_stacking_detection"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // ---------- 参数 ----------
        const std::string topic =
            declare_parameter<std::string>("cloud_topic", "/cloud_registered");

        // 查询用的目标坐标系：点云先变到这里，points 也按这个系给
        target_frame_ = declare_parameter<std::string>("target_frame", "map");
        tf_timeout_   = declare_parameter<double>("tf_timeout", 0.2);

        // 地面高度（目标坐标系下）。2026-11-04 用 bag 实测 map 系地面约 -0.73m
        // ★ 换场地 / 换车必须重新标定：RViz 里 Publish Point 点一下地面读 z
        ground_z_ = declare_parameter<double>("ground_z", -0.73);
        // false: z 轴朝上（当前 fast_lio 的发布方式），高度 h = z - ground_z
        // true : z 轴朝下（原始 LIO 世界系），高度 h = ground_z - z
        inverted_ = declare_parameter<bool>("inverted", false);

        // 单个地块的高度（米）。层数按中线判决，见 analyze()
        layer_height_ = declare_parameter<double>("layer_height", 0.35);
        // 查询点周围的方柱半径（米），ROI 为 [x±r, y±r]
        roi_half_size_ = declare_parameter<double>("roi_half_size", 0.30);
        // 竖直方向最多往上看多高（米）
        max_stack_height_ = declare_parameter<double>("max_stack_height", 2.0);
        // 取多少分位作为堆叠顶面。单帧点云稀疏，0.95 会系统性偏低，默认 0.99
        top_percentile_ = declare_parameter<double>("top_percentile", 0.99);
        // ROI 内点数少于该值视为没有物体
        min_points_ = declare_parameter<int>("min_points", 10);
        // 处理节流（秒）
        process_period_ = declare_parameter<double>("process_period", 0.2);
        // 多帧累积窗口（秒）：MID360 单帧在 0.6x0.6m 的 ROI 里只有几十个点，
        // 顶面经常漏扫。先攒 window 秒的点云再统计，高度/层数稳定很多。
        // <= 0 表示不累积，退回单帧
        accumulate_window_ = declare_parameter<double>("accumulate_window", 0.2);
        // 统计离群点去除：ROI 点数少时 meanK 不能大；<= 1 表示跳过
        outlier_mean_k_ = declare_parameter<int>("outlier_mean_k", 8);
        outlier_stddev_ = declare_parameter<double>("outlier_stddev", 1.0);

        // ---------- 查询点：扁平二维数组 [x1,y1, x2,y2, ...] ----------
        const auto flat = declare_parameter<std::vector<double>>(
            "points", std::vector<double>{});
        const auto names = declare_parameter<std::vector<std::string>>(
            "point_names", std::vector<std::string>{});

        if (flat.size() % 2 != 0) {
            RCLCPP_ERROR(get_logger(),
                         "参数 points 需要成对的 [x,y]，收到 %zu 个数，已忽略",
                         flat.size());
        } else {
            for (size_t i = 0; i + 1 < flat.size(); i += 2) {
                const size_t idx = i / 2;
                QueryPoint q;
                q.name = (idx < names.size()) ? names[idx]
                                              : "point_" + std::to_string(idx + 1);
                q.x = static_cast<float>(flat[i]);
                q.y = static_cast<float>(flat[i + 1]);
                points_.push_back(q);
            }
        }
        if (points_.empty()) {
            RCLCPP_WARN(get_logger(),
                        "没有配置查询点，请用 -p points:=\"[x1,y1,x2,y2,...]\" 指定"
                        "（%s 系坐标）",
                        target_frame_.empty() ? "输入点云" : target_frame_.c_str());
        }

        // ---------- 订阅 / 发布 ----------
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&BlockStackingDetection::cloudCallback, this,
                      std::placeholders::_1));

        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/block_stacking/markers", 10);
        pub_roi_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/block_stacking/corner_cloud", 10);
        pub_layers_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/block_stacking/layers", 10);

        RCLCPP_INFO(get_logger(),
                    "BlockStackingDetection 启动：订阅 %s → 目标系 %s，"
                    "地面 z=%.2f（%s），单层 %.2fm，查询点 %zu 个",
                    topic.c_str(),
                    target_frame_.empty() ? "输入点云坐标系" : target_frame_.c_str(),
                    ground_z_, inverted_ ? "z 轴朝下" : "z 轴朝上",
                    layer_height_, points_.size());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string frame_id = msg->header.frame_id;
        if (!target_frame_.empty() && frame_id != target_frame_) {
            std::string tf_error;
            if (!tf_buffer_.canTransform(
                    target_frame_, frame_id, rclcpp::Time(msg->header.stamp),
                    rclcpp::Duration::from_seconds(tf_timeout_), &tf_error)) {
                const double waiting_seconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - tf_wait_start_).count();
                if (!tf_ready_ && waiting_seconds < 5.0) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "TF 缓存预热：%s -> %s 尚不能转换当前点云时间，跳过本帧，等待后续有效帧。",
                        frame_id.c_str(), target_frame_.c_str());
                } else {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "点云时刻的 TF %s -> %s 不可用（%s）；本帧跳过。"
                        "请检查定位 TF 和传感器时间戳。",
                        frame_id.c_str(), target_frame_.c_str(), tf_error.c_str());
                }
                return;
            }
        }

        // ---------- 1. 转 PCL ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到空点云");
            return;
        }

        // ---------- 2. 变换到 target_frame（默认 map） ----------
        if (!target_frame_.empty() && frame_id != target_frame_) {
            try {
                const auto ts = tf_buffer_.lookupTransform(
                    target_frame_, frame_id, rclcpp::Time(msg->header.stamp));

                const auto& tr = ts.transform.translation;
                const auto& rot = ts.transform.rotation;
                const tf2::Quaternion q(rot.x, rot.y, rot.z, rot.w);

                Eigen::Affine3f T = Eigen::Affine3f::Identity();
                T.translation() << static_cast<float>(tr.x),
                                   static_cast<float>(tr.y),
                                   static_cast<float>(tr.z);
                T.linear() = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z())
                                 .toRotationMatrix();
                pcl::transformPointCloud(*cloud, *cloud, T);
                frame_id = target_frame_;
                if (!tf_ready_) {
                    tf_ready_ = true;
                    RCLCPP_INFO(
                        get_logger(), "TF 缓存已就绪，开始按点云原始时间戳进行堆叠检测。");
                }
            } catch (const tf2::TransformException& ex) {
                // 没有定位时没有 map->odom，本帧跳过，避免发出错坐标系的框
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "等不到 TF %s -> %s（%s）；本帧跳过。"
                    "纯建图无定位时可 -p target_frame:=odom",
                    frame_id.c_str(), target_frame_.c_str(), ex.what());
                return;
            }
        }

        // ---------- 3. 累积最近 accumulate_window 秒的点云 ----------
        //   单帧 ROI 只有几十个点，顶面经常漏扫；先攒几帧再统计。
        const rclcpp::Time stamp(msg->header.stamp);
        if (accumulate_window_ > 0.0) {
            cloud_buffer_.emplace_back(stamp, cloud);
            while (!cloud_buffer_.empty() &&
                   (stamp - cloud_buffer_.front().first).seconds() >
                       accumulate_window_) {
                cloud_buffer_.pop_front();
            }
        }

        // ---------- 4. 节流：process_period 到了才计算一次 ----------
        const rclcpp::Time now = this->now();
        if (last_process_time_.nanoseconds() != 0 &&
            (now - last_process_time_).seconds() < process_period_) {
            return;
        }
        last_process_time_ = now;

        pcl::PointCloud<pcl::PointXYZ>::Ptr merged = cloud;
        size_t used_frames = 1;
        if (accumulate_window_ > 0.0 && cloud_buffer_.size() > 1) {
            size_t total = 0;
            for (const auto& entry : cloud_buffer_) {
                total += entry.second->size();
            }
            merged.reset(new pcl::PointCloud<pcl::PointXYZ>);
            merged->reserve(total);
            for (const auto& entry : cloud_buffer_) {
                *merged += *entry.second;
            }
            used_frames = cloud_buffer_.size();
        }

        // ---------- 5. 逐个查询点统计 ----------
        std::vector<StackResult> results;
        results.reserve(points_.size());
        for (const auto& qp : points_) {
            results.push_back(analyze(qp, merged, used_frames));
        }

        // ---------- 6. 发布 ----------
        std_msgs::msg::Header out_header = msg->header;
        out_header.frame_id = frame_id;
        publishVisualization(out_header, results);
        publishLayers(results);
    }

    StackResult analyze(const QueryPoint& qp,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        size_t used_frames)
    {
        StackResult res;
        res.name = qp.name;
        res.x = qp.x;
        res.y = qp.y;

        // 竖直 ROI：从地面往上 max_stack_height
        float z_low, z_high;
        if (inverted_) {
            z_low  = static_cast<float>(ground_z_ - max_stack_height_);
            z_high = static_cast<float>(ground_z_);
        } else {
            z_low  = static_cast<float>(ground_z_);
            z_high = static_cast<float>(ground_z_ + max_stack_height_);
        }

        const float r = static_cast<float>(roi_half_size_);
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(qp.x - r, qp.y - r, z_low, 1.0f));
        crop.setMax(Eigen::Vector4f(qp.x + r, qp.y + r, z_high, 1.0f));

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*roi_cloud);

        if (static_cast<int>(roi_cloud->size()) < min_points_) {
            res.cloud = roi_cloud;
            return res;
        }

        // 去噪：ROI 点数本来就不多，meanK 不能大；点数不足 meanK 时直接跳过，
        // 否则会把稀疏的顶面点当成离群点删掉，高度反而变低。
        pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(
            new pcl::PointCloud<pcl::PointXYZ>(*roi_cloud));
        if (outlier_mean_k_ > 1 &&
            static_cast<int>(roi_cloud->size()) > outlier_mean_k_) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(roi_cloud);
            sor.setMeanK(outlier_mean_k_);
            sor.setStddevMulThresh(outlier_stddev_);
            sor.filter(*denoised);
        }
        res.cloud = denoised;

        if (static_cast<int>(denoised->size()) < min_points_) {
            return res;
        }

        // 每个点相对地面的高度
        std::vector<float> heights;
        heights.reserve(denoised->size());
        for (const auto& pt : denoised->points) {
            const float h = inverted_
                                ? static_cast<float>(ground_z_) - pt.z
                                : pt.z - static_cast<float>(ground_z_);
            if (h >= 0.0f) {
                heights.push_back(h);
            }
        }
        if (heights.empty()) return res;

        std::sort(heights.begin(), heights.end());
        res.height = percentile(heights, static_cast<float>(top_percentile_));
        // 按每层中线判决：单块≈0.345m / layer_height 0.35 = 0.99，
        // 直接 floor 会判成 0 层；加半层再 floor 才能得到 1 层。
        res.layers = static_cast<int>(std::floor(
            (res.height + 0.5f * static_cast<float>(layer_height_)) /
            static_cast<float>(layer_height_)));
        res.has_object = res.layers > 0;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "%s (%.2f, %.2f) 累积%zu帧 点数=%zu 堆叠高度=%.3fm 层数=%d",
            res.name.c_str(), res.x, res.y, used_frames, denoised->size(),
            res.height, res.layers);
        return res;
    }

    void publishVisualization(const std_msgs::msg::Header& header,
                              const std::vector<StackResult>& results)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < results.size(); ++i) {
            const auto& res = results[i];
            const float r = static_cast<float>(roi_half_size_);

            float z_low, z_high;
            if (inverted_) {
                z_low  = static_cast<float>(ground_z_ - max_stack_height_);
                z_high = static_cast<float>(ground_z_);
            } else {
                z_low  = static_cast<float>(ground_z_);
                z_high = static_cast<float>(ground_z_ + max_stack_height_);
            }
            // 检测到的堆叠顶面
            const float z_top = inverted_
                                    ? static_cast<float>(ground_z_ - res.height)
                                    : static_cast<float>(ground_z_ + res.height);

            // 3.1 ROI 方柱：绿=有堆叠，蓝=没有
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "block_stacking_roi";
            box.id = static_cast<int>(i);
            box.type = visualization_msgs::msg::Marker::LINE_LIST;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.scale.x = 0.02;
            box.color.r = 0.0f;
            box.color.g = res.has_object ? 1.0f : 0.5f;
            box.color.b = res.has_object ? 0.0f : 1.0f;
            box.color.a = 0.9f;
            box.pose.orientation.w = 1.0;

            const pcl::PointXYZ c[8] = {
                {res.x - r, res.y - r, z_low},  {res.x + r, res.y - r, z_low},
                {res.x + r, res.y + r, z_low},  {res.x - r, res.y + r, z_low},
                {res.x - r, res.y - r, z_high}, {res.x + r, res.y - r, z_high},
                {res.x + r, res.y + r, z_high}, {res.x - r, res.y + r, z_high}};
            const int edges[12][2] = {{0,1},{1,2},{2,3},{3,0},
                                      {4,5},{5,6},{6,7},{7,4},
                                      {0,4},{1,5},{2,6},{3,7}};
            for (const auto& e : edges) {
                geometry_msgs::msg::Point p0, p1;
                p0.x = c[e[0]].x; p0.y = c[e[0]].y; p0.z = c[e[0]].z;
                p1.x = c[e[1]].x; p1.y = c[e[1]].y; p1.z = c[e[1]].z;
                box.points.push_back(p0);
                box.points.push_back(p1);
            }
            marker_array.markers.push_back(box);

            // 3.2 检测到的顶面：黄色方框
            if (res.has_object) {
                visualization_msgs::msg::Marker top;
                top.header = header;
                top.ns = "block_stacking_top";
                top.id = static_cast<int>(100 + i);
                top.type = visualization_msgs::msg::Marker::LINE_STRIP;
                top.action = visualization_msgs::msg::Marker::ADD;
                top.scale.x = 0.03;
                top.color.r = 1.0f;
                top.color.g = 1.0f;
                top.color.b = 0.0f;
                top.color.a = 1.0f;
                top.pose.orientation.w = 1.0;
                const pcl::PointXYZ corners[4] = {
                    {res.x - r, res.y - r, z_top}, {res.x + r, res.y - r, z_top},
                    {res.x + r, res.y + r, z_top}, {res.x - r, res.y + r, z_top}};
                for (int k = 0; k < 5; ++k) {
                    const auto& p = corners[k % 4];
                    geometry_msgs::msg::Point gp;
                    gp.x = p.x; gp.y = p.y; gp.z = p.z;
                    top.points.push_back(gp);
                }
                marker_array.markers.push_back(top);
            }

            // 3.3 层数文字
            visualization_msgs::msg::Marker text;
            text.header = header;
            text.ns = "block_stacking_text";
            text.id = static_cast<int>(200 + i);
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.scale.z = 0.2;
            text.color.r = 1.0f;
            text.color.g = 1.0f;
            text.color.b = 1.0f;
            text.color.a = 1.0f;
            text.pose.position.x = res.x;
            text.pose.position.y = res.y;
            // 放在堆叠顶面之上（倒装时"上"是 -z）
            text.pose.position.z = inverted_ ? z_top - 0.2f : z_top + 0.2f;
            text.pose.orientation.w = 1.0;
            text.text = res.has_object
                            ? (res.name + "\n" + std::to_string(res.layers) + " 层 / " +
                               std::to_string(static_cast<int>(res.height * 100)) + "cm")
                            : (res.name + "\n无");
            marker_array.markers.push_back(text);
        }

        pub_markers_->publish(marker_array);

        // 3.4 ROI 内点云合并发布：有堆叠=绿，没有=蓝
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        for (const auto& res : results) {
            if (!res.cloud) continue;
            for (const auto& p : res.cloud->points) {
                pcl::PointXYZRGB c;
                c.x = p.x; c.y = p.y; c.z = p.z;
                c.r = 0;
                c.g = res.has_object ? 255 : 80;
                c.b = res.has_object ? 0 : 200;
                colored.push_back(c);
            }
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(colored, out);
        out.header = header;
        pub_roi_cloud_->publish(out);
    }

    void publishLayers(const std::vector<StackResult>& results)
    {
        std_msgs::msg::Int32MultiArray arr;
        arr.layout.data_offset = 0;
        for (const auto& res : results) {
            arr.data.push_back(res.layers);
        }
        pub_layers_->publish(arr);
    }

    // 参数
    std::string target_frame_;
    double tf_timeout_;
    double ground_z_;
    bool   inverted_;
    double layer_height_;
    double roi_half_size_;
    double max_stack_height_;
    double top_percentile_;
    int    min_points_;
    double process_period_;
    double accumulate_window_;
    int    outlier_mean_k_;
    double outlier_stddev_;
    bool tf_ready_ = false;
    std::chrono::steady_clock::time_point tf_wait_start_{std::chrono::steady_clock::now()};

    std::vector<QueryPoint> points_;

    // 多帧累积缓冲：(点云时间戳, map 系点云)
    std::deque<std::pair<rclcpp::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>>
        cloud_buffer_;

    rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

    // 注意声明顺序：buffer 必须在 listener 之前构造
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_roi_cloud_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_layers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlockStackingDetection>());
    rclcpp::shutdown();
    return 0;
}
