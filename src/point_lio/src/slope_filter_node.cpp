#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

using std::placeholders::_1;

class SlopeFilter : public rclcpp::Node
{
public:
    SlopeFilter() : Node("slope_filter_node")
    {
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&SlopeFilter::topic_callback, this, _1));
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cloud_obstacles", 10);
        RCLCPP_INFO(this->get_logger(), "Slope Filter Node Started.");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*cloud_filtered);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_filtered);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.3);
        ne.compute(*cloud_normals);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < cloud_filtered->size(); ++i) {
            const auto & pt = cloud_filtered->points[i];
            const auto & nm = cloud_normals->points[i];

            if (pt.z > 2.2) {
                continue;
            }
            if (std::abs(nm.normal_z) > 0.73) {
                continue;
            }
            cloud_obstacles->push_back(pt);
        }

        if (!cloud_obstacles->empty()) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_obstacles, output_msg);
            output_msg.header = msg->header;
            pub_cloud_->publish(output_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlopeFilter>());
    rclcpp::shutdown();
    return 0;
}
