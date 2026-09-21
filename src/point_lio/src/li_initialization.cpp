#include "li_initialization.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;
// sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

rclcpp::Time latest_time;
V3D latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0, acc_0, gyr_0;
M3D latest_Q;
bool init = false;

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
    // mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    // printf("check lidar time %f\n", last_timestamp_lidar);
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
        }
    }
    else
    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //rclcpp::Time(msg->header.stamp).seconds();
        }
        if (frame_ct < 10)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
    }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg)
{
    // mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
        }
    }
    else
    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
    p_pre->process(msg, ptr);
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //rclcpp::Time(msg->header.stamp).seconds();
        }
        if (frame_ct < 10)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
    }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void updateLatestStates() {
  init = true;
  latest_time = rclcpp::Time(lidar_end_time * 1e9, RCL_ROS_TIME);
  if (use_imu_as_input) {
    latest_P = kf_input.x_.pos;
    latest_Q = kf_input.x_.rot;
    latest_V = kf_input.x_.vel;
    latest_Ba = kf_input.x_.ba;
    latest_Bg = kf_input.x_.bg;
  } else {
    latest_P = kf_output.x_.pos;
    latest_Q = kf_output.x_.rot;
    latest_V = kf_output.x_.vel;
    latest_Ba = kf_output.x_.ba;
    latest_Bg = kf_output.x_.bg;
  }
}

void fastPredictIMU(const rclcpp::Node& node, const rclcpp::Time &t,const V3D &acc,const V3D &gyr, const tf2_ros::Buffer & tf_buffer, tf2_ros::TransformBroadcaster &tf_broadcaster)
{
  // IMU前向预测
  double dt = (t - latest_time).seconds();
  latest_time = t;
  V3D un_acc_0;
  if (use_imu_as_input) {
    un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) + kf_input.x_.gravity;
  } else {
    un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) + kf_output.x_.gravity;
  }
  V3D un_gyr = 0.5 * (latest_gyr_0 + gyr) - latest_Bg;
  latest_Q = latest_Q * Exp(un_gyr, dt);
  V3D un_acc_1;
  if (use_imu_as_input) {
    un_acc_1 = latest_Q * (acc - latest_Ba) + kf_input.x_.gravity;
  } else {
    un_acc_1 = latest_Q * (acc - latest_Ba) + kf_output.x_.gravity;
  }
  V3D un_acc = 0.5 * (un_acc_0 + un_acc_1);
  latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
  latest_V = latest_V + dt * un_acc;
  latest_acc_0 = acc;
  latest_gyr_0 = gyr;

  // 根据运动学模型前向预测
  rclcpp::Time now = node.now();
  dt = (now - t).seconds();
  V3D now_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
  M3D now_Q = latest_Q * Exp(un_gyr, dt);

  geometry_msgs::msg::TransformStamped base_link_to_livox_frame_transform;
  try {
    base_link_to_livox_frame_transform =
     tf_buffer.lookupTransform("livox_frame", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "Failed to lookup transform from base_link to livox_frame: %s", ex.what());
    return;
  }

  // Create a TransformStamped message for lidar_odom to base_link
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = now;
  transform_stamped.header.frame_id = "odom";    // Source frame
  transform_stamped.child_frame_id = "real_time_base_link";// Target frame

  // Calculate the transform from lidar_odom to base_link by multiplying the transforms
  geometry_msgs::msg::Pose pose;
  pose.position.x = now_P.x();
  pose.position.y = now_P.y();
  pose.position.z = now_P.z();
  Eigen::Quaterniond quadrotor_Q = Eigen::Quaterniond(now_Q);
  pose.orientation.x = quadrotor_Q.x();
  pose.orientation.y = quadrotor_Q.y();
  pose.orientation.z = quadrotor_Q.z();
  pose.orientation.w = quadrotor_Q.w();
  tf2::Transform tf_lidar_odom_to_livox_frame;
  tf2::fromMsg(pose, tf_lidar_odom_to_livox_frame);
  tf2::Transform tf_base_link_to_livox_frame;
  tf2::fromMsg(base_link_to_livox_frame_transform.transform, tf_base_link_to_livox_frame);
  tf2::Transform tf_odom_to_base_link = tf_base_link_to_livox_frame.inverse() * tf_lidar_odom_to_livox_frame * tf_base_link_to_livox_frame;

  // Convert the resulting transform back to geometry_msgs::TransformStamped
  transform_stamped.transform = tf2::toMsg(tf_odom_to_base_link);

  // Publish the tf
  tf_broadcaster.sendTransform(transform_stamped);
}

void imu_cbk(const rclcpp::Node &node, const sensor_msgs::msg::Imu &msg_in, const tf2_ros::Buffer & tf_buffer, tf2_ros::TransformBroadcaster &tf_broadcaster)
{
    // mtx_buffer.lock();

    // publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(msg_in);

    if(init)
    {
      fastPredictIMU(node, msg_in.header.stamp,
                     V3D(msg_in.linear_acceleration.x,msg_in.linear_acceleration.y,msg_in.linear_acceleration.z),
                     V3D(msg_in.angular_velocity.x,msg_in.angular_velocity.y,msg_in.angular_velocity.z), tf_buffer, tf_broadcaster);
    }

    msg->header.stamp = get_ros_time(get_time_sec(msg_in.header.stamp) - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = get_time_sec(msg->header.stamp);
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    {
    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                meas.lidar = lidar_buffer.front();
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;
                if(meas.lidar->points.size() < 1)
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    double end_time = meas.lidar->points.back().curvature;
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }

            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if(meas.lidar->points.size() < 1)
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time))
    {
        return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte)
    {
        return false;
    }

    if (!lose_lid && !imu_pushed)
    {
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = get_time_sec(imu_deque.front()->header.stamp);
            imu_next = *(imu_deque.front());
            meas.imu.shrink_to_fit();
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = get_time_sec(imu_deque.front()->header.stamp); // can be changed
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    if (lose_lid && !imu_pushed)
    {
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            double imu_time = get_time_sec(imu_deque.front()->header.stamp);
            meas.imu.shrink_to_fit();

            imu_next = *(imu_deque.front());
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = get_time_sec(imu_deque.front()->header.stamp); // can be changed
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
    }
}
