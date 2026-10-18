#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class SlopeFilter : public rclcpp::Node
{
public:
    SlopeFilter() : Node("slope_filter_node")
    {
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&SlopeFilter::topic_callback, this, _1));

        // 发布：配对成功的盒子面点云
        pub_box_faces_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/box_faces", 10);

        // 发布：所有区域生长平面（方便调试）
        pub_planes_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/region_planes", 10);

        RCLCPP_INFO(this->get_logger(), "SlopeFilter 节点已启动，订阅 /cloud_registered");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // ---------- 1. 转 PCL ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空点云");
            return;
        }

        // ---------- 2. Z 高度筛选 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0f, 1.0f);
        pass.filter(*cloud_z);
        if (cloud_z->empty()) {
            RCLCPP_WARN(this->get_logger(), "Z 筛选后为空");
            return;
        }

        // ---------- 3. 降采样 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_z);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_down);
        if (cloud_down->empty()) {
            RCLCPP_WARN(this->get_logger(), "降采样后为空");
            return;
        }

        // ---------- 4. 去噪声 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_denoised(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_down);
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_denoised);
        if (cloud_denoised->empty()) {
            RCLCPP_WARN(this->get_logger(), "去噪后为空");
            return;
        }

        // ---------- 5. 法向量 ----------
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_denoised);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_denoised);
        ne.setSearchMethod(tree);
        ne.setKSearch(30);
        ne.compute(*normals);

        // ---------- 6. 区域生长 ----------
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(100);
        reg.setMaxClusterSize(100000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud_denoised);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1.0);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // ---------- 7. 逐平面算 OBB、法向量，筛选尺寸 ----------
        const float target_size = 0.5f;
        const float size_tolerance = 0.10f;

        struct PlaneInfo {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            Eigen::Vector3f normal;
            Eigen::Vector3f centroid;
            bool size_ok;
        };
        std::vector<PlaneInfo> planes;

        for (const auto& indices : clusters) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices)
                cluster_cloud->push_back((*cloud_denoised)[idx]);

            pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud(cluster_cloud);
            feature_extractor.compute();

            pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;
            feature_extractor.getOBB(min_point_OBB, max_point_OBB,
                                     position_OBB, rotational_matrix_OBB);

            float size_x = max_point_OBB.x - min_point_OBB.x;
            float size_y = max_point_OBB.y - min_point_OBB.y;
            float size_z = max_point_OBB.z - min_point_OBB.z;

            std::vector<float> sizes = {size_x, size_y, size_z};
            std::sort(sizes.begin(), sizes.end());
            float s_mid   = sizes[1];
            float s_large = sizes[2];

            Eigen::Vector4f centroid4;
            pcl::compute3DCentroid(*cluster_cloud, centroid4);
            Eigen::Vector3f centroid = centroid4.head<3>();

            Eigen::Vector4f pca_centroid;
            Eigen::Matrix3f pca_evecs;
            pcl::compute3DCentroid(*cluster_cloud, pca_centroid);
            pcl::computeCovarianceMatrixNormalized(*cluster_cloud, pca_centroid, pca_evecs);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(pca_evecs);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);
            normal.normalize();

            bool size_ok =
                std::fabs(s_mid   - target_size) < size_tolerance &&
                std::fabs(s_large - target_size) < size_tolerance;

            PlaneInfo info;
            info.cloud = cluster_cloud;
            info.normal = normal;
            info.centroid = centroid;
            info.size_ok = size_ok;
            planes.push_back(info);
        }

        // ---------- 8. 两两配对垂直平面，合并盒子面 ----------
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_faces(new pcl::PointCloud<pcl::PointXYZ>);
        int box_count = 0;

        for (size_t a = 0; a < planes.size(); ++a) {
            if (!planes[a].size_ok) continue;
            for (size_t b = a + 1; b < planes.size(); ++b) {
                if (!planes[b].size_ok) continue;

                float dot = std::fabs(planes[a].normal.dot(planes[b].normal));
                if (dot < 0.2f) {
                    float center_dist = (planes[a].centroid - planes[b].centroid).norm();
                    if (center_dist < 1.0f) {
                        *box_faces += *planes[a].cloud;
                        *box_faces += *planes[b].cloud;

                        pcl::PointXYZ min_pt, max_pt;
                        pcl::getMinMax3D(*box_faces, min_pt, max_pt);
                        float cx = (min_pt.x + max_pt.x) / 2.0f;
                        float cy = (min_pt.y + max_pt.y) / 2.0f;
                        float cz = (min_pt.z + max_pt.z) / 2.0f;

                        float distance = std::sqrt(cx * cx + cy * cy);
                        RCLCPP_INFO(this->get_logger(),
                                    "盒子 %d: 中心(%.2f, %.2f, %.2f) 距离=%.2fm 高度=%.2fm",
                                    box_count, cx, cy, cz, distance, cz);

                        ++box_count;
                    }
                }
            }
        }

        // ---------- 9. 发布 ----------
        if (!box_faces->empty()) {
            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(*box_faces, out_msg);
            out_msg.header = msg->header;
            pub_box_faces_->publish(out_msg);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : planes) {
            *all_planes += *p.cloud;
        }
        if (!all_planes->empty()) {
            sensor_msgs::msg::PointCloud2 out_msg;
            pcl::toROSMsg(*all_planes, out_msg);
            out_msg.header = msg->header;
            pub_planes_->publish(out_msg);
        }

        RCLCPP_INFO(this->get_logger(), "处理完成: 平面 %zu 个, 盒子 %d 个",
                    planes.size(), box_count);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_box_faces_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_planes_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlopeFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}