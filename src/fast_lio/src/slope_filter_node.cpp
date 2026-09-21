#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <limits>
#include <vector>

using std::placeholders::_1;

class SlopeFilter : public rclcpp::Node
{
public:
    SlopeFilter() : Node("slope_filter_node")
    {
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&SlopeFilter::topic_callback, this, _1));
        // sub_global_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/Laser_map", 10, std::bind(&SlopeFilter::global_topic_callback, this, _1));
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_obstacles", 10);

        // global_pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     "/global_cloud_obstacles", 10);
        RCLCPP_INFO(this->get_logger(), "Slope Filter Node Started.");
    }

private:
    static bool is_finite_normal(const pcl::Normal& normal)
    {
        return std::isfinite(normal.normal_x) &&
               std::isfinite(normal.normal_y) &&
               std::isfinite(normal.normal_z);
    }

    static bool is_wall_normal(const pcl::Normal& normal)
    {
        constexpr float kWallNormalZMax = 0.45f;
        return is_finite_normal(normal) && std::abs(normal.normal_z) <= kWallNormalZMax;
    }

    static bool has_right_angle_neighbor(
        size_t index,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
    {
        constexpr float kCornerSearchRadius = 0.35f;
        constexpr float kRightAngleDotMax = 0.17f;   // cos(80 deg): 80-100 deg is treated as near-right-angle.
        constexpr float kMinNeighborDistance = 0.12f;
        constexpr int kMinPerpendicularNeighbors = 8;

        const auto& base_normal = normals->points[index];
        if (!is_wall_normal(base_normal)) return false;

        const float base_xy_norm = std::hypot(base_normal.normal_x, base_normal.normal_y);
        if (base_xy_norm < std::numeric_limits<float>::epsilon()) return false;

        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        if (tree->radiusSearch(cloud->points[index], kCornerSearchRadius, neighbor_indices, neighbor_distances) <= 0) {
            return false;
        }

        int perpendicular_neighbors = 0;
        for (size_t i = 0; i < neighbor_indices.size(); ++i) {
            const int neighbor_index = neighbor_indices[i];
            if (neighbor_index == static_cast<int>(index)) continue;
            if (neighbor_distances[i] < kMinNeighborDistance * kMinNeighborDistance) continue;

            const auto& neighbor_normal = normals->points[neighbor_index];
            if (!is_wall_normal(neighbor_normal)) continue;

            const float neighbor_xy_norm = std::hypot(neighbor_normal.normal_x, neighbor_normal.normal_y);
            if (neighbor_xy_norm < std::numeric_limits<float>::epsilon()) continue;

            const float dot_xy = std::abs(
                (base_normal.normal_x * neighbor_normal.normal_x +
                 base_normal.normal_y * neighbor_normal.normal_y) /
                (base_xy_norm * neighbor_xy_norm));

            if (dot_xy <= kRightAngleDotMax && ++perpendicular_neighbors >= kMinPerpendicularNeighbors) {
                return true;
            }
        }

        return false;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换 ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm 一个格子
        sor.filter(*cloud_filtered);

        //  法向量估算 
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_filtered);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_filtered);
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        
        // 搜索半径 0.3米 
        ne.setRadiusSearch(0.3); 
        ne.compute(*cloud_normals);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (size_t i = 0; i < cloud_filtered->size(); ++i)
        {
            auto& pt = cloud_filtered->points[i];
            auto& nm = cloud_normals->points[i];

            // A. 高度绝对过滤 (保留 < 2.2m) 
            if (pt.z > 2.2) continue;

            // 保留竖直障碍物，过滤地面/斜坡
            if (!is_wall_normal(nm)) continue;

            // 只保留局部邻域里存在近似 90 度法向量关系的障碍物角边
            if (!has_right_angle_neighbor(i, cloud_filtered, cloud_normals, tree)) continue;

            cloud_obstacles->push_back(pt);
        }

        if (!cloud_obstacles->empty())
        {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_obstacles, output_msg);
            output_msg.header = msg->header; // 继承原始的 timestamp 和 frame_id (body)
            pub_cloud_->publish(output_msg);
        }
    }
    // void global_topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    // {
    //     // 转换 ROS -> PCL
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::fromROSMsg(*msg, *cloud);

    //     if (cloud->empty()) return;
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::VoxelGrid<pcl::PointXYZ> sor;
    //     sor.setInputCloud(cloud);
    //     sor.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm 一个格子
    //     sor.filter(*cloud_filtered);

    //     
    //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //     ne.setInputCloud(cloud_filtered);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    //     ne.setSearchMethod(tree);
    //     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        
    //    
    //     ne.setRadiusSearch(0.5); 
    //     ne.compute(*cloud_normals);

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        
    //     for (size_t i = 0; i < cloud_filtered->size(); ++i)
    //     {
    //         auto& pt = cloud_filtered->points[i];
    //         auto& nm = cloud_normals->points[i];

    //         
    //         if (pt.z > 2.2) continue;

    //         
    //         if (std::abs(nm.normal_z) > 0.73) continue; 

    //         
    //         cloud_obstacles->push_back(pt);
    //     }

    //     if (!cloud_obstacles->empty())
    //     {
    //         sensor_msgs::msg::PointCloud2 output_msg;
    //         pcl::toROSMsg(*cloud_obstacles, output_msg);
    //         output_msg.header = msg->header; // 继承原始的 timestamp 和 frame_id (body)
    //         global_pub_cloud_->publish(output_msg);
    //     }
    // }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_global_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pub_cloud_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlopeFilter>());
    rclcpp::shutdown();
    return 0;
}
