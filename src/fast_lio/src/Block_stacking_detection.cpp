#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// ============================================================
// 四角堆叠检测（在线版）
//
//   离线版原来是读 ../scans.pcd，现在改成订阅 FAST-LIO 的
//   /cloud_registered（里程计坐标系 odom 下的世界系点云，
//   带 z，单位米），每个角做：
//     裁剪 -> z 分位数 -> 高度差 -> 层数
//
//   可视化不再用 PCLVisualizer，改成发 RViz marker：
//     /block_stacking/markers        四个 ROI 框 + 层数文字
//     /block_stacking/corner_cloud   框内点云（绿=有立方体，蓝=没有）
//     /block_stacking/layers         四角层数 Int32MultiArray
//
//   注意：/cloud_registered 是单帧点云（fast_lio 里
//   scan_pub_en 控制），框内点会随车/雷达移动变化。
//   如果想用累积地图，把 cloud_topic 参数改成 /Laser_map。
// ============================================================

// 一个角区域的定义（长方体裁剪框），坐标在 odom 系
struct CornerROI {
    std::string name;
    float x_min, x_max;
    float y_min, y_max;
    float z_min, z_max;
};

// 单个角的检测结果
struct CornerResult {
    std::string name;
    bool has_cube = false;
    int layers = 0;
    float ground_z = 0.0f;
    float top_z = 0.0f;
    float height_diff = 0.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

// 计算分位数（0~1），输入已排序
//   p = 0.05 → 低 5% 分位，用作地面高度，避免单个低噪声点拉低估计
//   p = 0.95 → 高 95% 分位，用作顶部高度，避免单个离群点拉高层数
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
    BlockStackingDetection() : Node("block_stacking_detection")
    {
        // ---------- 参数（可以在 launch/yaml 里覆盖） ----------
        // CUBE_HEIGHT：单层立方体高度，layer = round(高度差 / CUBE_HEIGHT)
        cube_height_        = declare_parameter<double>("cube_height", 0.5);
        // 只保留 z 低于该值的点，滤掉高处异常点
        max_height_         = declare_parameter<double>("max_height", 3.0);
        // 高度差超过该值才判“有物体”，默认 0.25m
        min_height_for_cube_ = declare_parameter<double>("min_height_for_cube",
                                                         cube_height_ * 0.5);
        ground_percentile_  = declare_parameter<double>("ground_percentile", 0.05);
        top_percentile_     = declare_parameter<double>("top_percentile", 0.95);
        // 框内点数少于该值直接跳过
        min_points_         = declare_parameter<int>("min_points", 20);
        // 处理节流（秒）：/cloud_registered 约 10Hz，没必要每帧都算
        process_period_     = declare_parameter<double>("process_period", 0.2);
        const std::string topic =
            declare_parameter<std::string>("cloud_topic", "/cloud_registered");

        // ---------- 四个角的 ROI：每项 6 个数 ----------
        //   [x_min, x_max, y_min, y_max, z_min, z_max]
        //   ★ 必须按实际地图坐标系改，确保立方体完整落在框内
        std::vector<std::string> names = declare_parameter<std::vector<std::string>>(
            "corner_names", {"corner_1", "corner_2", "corner_3", "corner_4"});
        const std::vector<std::vector<double>> defaults = {
            {-1.0, 1.0, -1.0, 1.0, -0.5, 1.8},
            {-1.0, 1.0, -1.0, 1.0, -0.5, 1.8},
            {-1.0, 1.0, -1.0, 1.0, -0.5, 1.8},
            {-1.0, 1.0, -1.0, 1.0, -0.5, 1.8},
        };
        for (size_t i = 0; i < defaults.size(); ++i) {
            const std::string pname = "corner_" + std::to_string(i + 1);
            auto v = declare_parameter<std::vector<double>>(pname, defaults[i]);
            if (v.size() != 6) {
                RCLCPP_ERROR(get_logger(), "参数 %s 需要 6 个数，收到 %zu 个，使用默认值",
                             pname.c_str(), v.size());
                v = defaults[i];
            }
            CornerROI roi;
            roi.name  = (i < names.size()) ? names[i] : pname;
            roi.x_min = static_cast<float>(v[0]);
            roi.x_max = static_cast<float>(v[1]);
            roi.y_min = static_cast<float>(v[2]);
            roi.y_max = static_cast<float>(v[3]);
            roi.z_min = static_cast<float>(v[4]);
            roi.z_max = static_cast<float>(v[5]);
            corners_.push_back(roi);
        }

        // ---------- 订阅 / 发布 ----------
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&BlockStackingDetection::cloudCallback, this, std::placeholders::_1));

        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/block_stacking/markers", 10);
        // 四个角的点云合并成一份发布，RViz 里一个显示项就能看全部区域
        pub_corner_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/block_stacking/corner_cloud", 10);
        pub_layers_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/block_stacking/layers", 10);

        RCLCPP_INFO(get_logger(),
                    "BlockStackingDetection 已启动，订阅 %s，立方体高度 %.2fm",
                    topic.c_str(), cube_height_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ---------- 0. 节流 ----------
        const rclcpp::Time now = this->now();
        if (last_process_time_.nanoseconds() != 0 &&
            (now - last_process_time_).seconds() < process_period_) {
            return;
        }
        last_process_time_ = now;

        // ---------- 1. 转 PCL ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "收到空点云");
            return;
        }

        // ---------- 1.5 高度筛选：只保留 z <= max_height 的点 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::infinity(),
                             static_cast<float>(max_height_));
        pass.filter(*height_filtered);

        if (height_filtered->empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "z 筛选后为空（max_height=%.2f）", max_height_);
            return;
        }

        // ---------- 2. 逐角裁剪、高度判断 ----------
        std::vector<CornerResult> results;
        results.reserve(corners_.size());

        for (const auto& roi : corners_) {
            CornerResult res;
            res.name = roi.name;

            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(height_filtered);
            crop.setMin(Eigen::Vector4f(roi.x_min, roi.y_min, roi.z_min, 1.0f));
            crop.setMax(Eigen::Vector4f(roi.x_max, roi.y_max, roi.z_max, 1.0f));

            pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
            crop.filter(*corner_cloud);

            if (static_cast<int>(corner_cloud->size()) < min_points_) {
                RCLCPP_DEBUG(get_logger(), "%s: 点云太少 (%zu)，跳过",
                             roi.name.c_str(), corner_cloud->size());
                res.cloud = corner_cloud;
                results.push_back(res);
                continue;
            }

            res.cloud = corner_cloud;

            // 收集 Z 值并排序
            std::vector<float> z_vals;
            z_vals.reserve(corner_cloud->size());
            for (const auto& pt : corner_cloud->points) {
                z_vals.push_back(pt.z);
            }
            std::sort(z_vals.begin(), z_vals.end());

            const float z_ground = percentile(z_vals, static_cast<float>(ground_percentile_));
            const float z_top    = percentile(z_vals, static_cast<float>(top_percentile_));
            const float diff     = z_top - z_ground;

            res.ground_z    = z_ground;
            res.top_z       = z_top;
            res.height_diff = diff;

            if (diff > min_height_for_cube_) {
                res.has_cube = true;
                // layers = round(高度差 / CUBE_HEIGHT)，至少 1 层
                int layers = static_cast<int>(std::round(diff / cube_height_));
                res.layers = std::max(1, layers);
            } else {
                res.has_cube = false;
                res.layers = 0;
            }

            RCLCPP_INFO(get_logger(),
                        "%s 点数=%zu 地面Z=%.3f 顶部Z=%.3f 高度差=%.3f 有立方体=%s 层数=%d",
                        roi.name.c_str(), corner_cloud->size(), z_ground, z_top, diff,
                        res.has_cube ? "是" : "否", res.layers);

            results.push_back(res);
        }

        // ---------- 3. 发布 ----------
        publishVisualization(msg->header, results);
        publishLayers(msg->header, results);
    }

    void publishVisualization(const std_msgs::msg::Header& header,
                              const std::vector<CornerResult>& results)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // 3.1 ROI 框：绿=有立方体，蓝=没有
        for (size_t i = 0; i < corners_.size(); ++i) {
            const auto& roi = corners_[i];
            const bool has = (i < results.size()) ? results[i].has_cube : false;

            visualization_msgs::msg::Marker m;
            m.header = header;
            m.ns = "block_stacking_roi";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::LINE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.02;
            m.color.r = 0.0f;
            m.color.g = has ? 1.0f : 0.5f;
            m.color.b = has ? 0.0f : 1.0f;
            m.color.a = 0.9f;
            m.pose.orientation.w = 1.0;

            // 12 条棱
            const pcl::PointXYZ c[8] = {
                {roi.x_min, roi.y_min, roi.z_min}, {roi.x_max, roi.y_min, roi.z_min},
                {roi.x_max, roi.y_max, roi.z_min}, {roi.x_min, roi.y_max, roi.z_min},
                {roi.x_min, roi.y_min, roi.z_max}, {roi.x_max, roi.y_min, roi.z_max},
                {roi.x_max, roi.y_max, roi.z_max}, {roi.x_min, roi.y_max, roi.z_max}};
            const int edges[12][2] = {{0,1},{1,2},{2,3},{3,0},
                                      {4,5},{5,6},{6,7},{7,4},
                                      {0,4},{1,5},{2,6},{3,7}};
            for (const auto& e : edges) {
                geometry_msgs::msg::Point p0, p1;
                p0.x = c[e[0]].x; p0.y = c[e[0]].y; p0.z = c[e[0]].z;
                p1.x = c[e[1]].x; p1.y = c[e[1]].y; p1.z = c[e[1]].z;
                m.points.push_back(p0);
                m.points.push_back(p1);
            }
            marker_array.markers.push_back(m);
        }

        // 3.2 层数文字
        for (size_t i = 0; i < corners_.size() && i < results.size(); ++i) {
            const auto& roi = corners_[i];
            const auto& res = results[i];

            visualization_msgs::msg::Marker t;
            t.header = header;
            t.ns = "block_stacking_text";
            t.id = static_cast<int>(100 + i);
            t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::msg::Marker::ADD;
            t.scale.z = 0.2;
            t.color.r = 1.0f;
            t.color.g = 1.0f;
            t.color.b = 1.0f;
            t.color.a = 1.0f;
            t.pose.position.x = (roi.x_min + roi.x_max) * 0.5;
            t.pose.position.y = (roi.y_min + roi.y_max) * 0.5;
            t.pose.position.z = roi.z_max + 0.15;
            t.pose.orientation.w = 1.0;
            t.text = res.has_cube
                         ? (res.name + "\n" + std::to_string(res.layers) + " 层 / " +
                            std::to_string(static_cast<int>(res.height_diff * 100)) + "cm")
                         : (res.name + "\n无");
            marker_array.markers.push_back(t);
        }

        pub_markers_->publish(marker_array);

        // 3.3 框内点云：四个角拼成一份，有立方体 = 绿，没有 = 蓝
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        for (const auto& res : results) {
            if (!res.cloud) continue;
            for (const auto& p : res.cloud->points) {
                pcl::PointXYZRGB c;
                c.x = p.x; c.y = p.y; c.z = p.z;
                c.r = 0;
                c.g = res.has_cube ? 255 : 80;
                c.b = res.has_cube ? 0 : 200;
                colored.push_back(c);
            }
        }
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(colored, out);   // 空点云也发，用来清掉 RViz 里的旧显示
        out.header = header;
        pub_corner_cloud_->publish(out);
    }

    void publishLayers(const std_msgs::msg::Header& header,
                       const std::vector<CornerResult>& results)
    {
        std_msgs::msg::Int32MultiArray arr;
        arr.layout.data_offset = 0;
        for (const auto& res : results) {
            arr.data.push_back(res.layers);
        }
        pub_layers_->publish(arr);
    }

    // 参数
    double cube_height_;
    double max_height_;
    double min_height_for_cube_;
    double ground_percentile_;
    double top_percentile_;
    int    min_points_;
    double process_period_;

    std::vector<CornerROI> corners_;

    rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_cloud_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_layers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlockStackingDetection>());
    rclcpp::shutdown();
    return 0;
}
