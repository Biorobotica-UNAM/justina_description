// virtual_pointcloud.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

class DepthToPointCloud : public rclcpp::Node
{
public:
    DepthToPointCloud()
    : Node("depth_to_pointcloud")
    {
        // Parameters
        this->declare_parameter("max_frequency", 15.0);
        this->declare_parameter("min_depth", 0.1);
        this->declare_parameter("max_depth", 8.0);
        this->declare_parameter("downsample_step", 2);

        max_freq_ = this->get_parameter("max_frequency").as_double();
        min_depth_ = this->get_parameter("min_depth").as_double();
        max_depth_ = this->get_parameter("max_depth").as_double();
        downsample_step_ = this->get_parameter("downsample_step").as_int();

        // Subscribers
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hardware/camera/depth/image_raw", 1,
            std::bind(&DepthToPointCloud::depthCallback, this, std::placeholders::_1));

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/hardware/camera/depth/camera_info", 1,
            std::bind(&DepthToPointCloud::infoCallback, this, std::placeholders::_1));

        // Publisher (topic expected by navigation)
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/hardware/base/depth_registered/rectified_points", 1);

        // TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Throttling: last publish time
        last_pub_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "DepthToPointCloud node started (downsample=%d, max_freq=%.1f)", 
                    downsample_step_, max_freq_);
    }

private:
    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_)
        {
            camera_info_ = *msg;
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received");
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_)
            return;

        // Simple throttling: skip frames if we are publishing faster than max_freq_
        rclcpp::Time now = this->now();
        if ((now - last_pub_time_).seconds() < 1.0 / max_freq_)
            return;
        last_pub_time_ = now;

        // Convert depth image to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        const cv::Mat& depth = cv_ptr->image;
        if (depth.empty())
            return;

        // Get camera intrinsics
        double fx = camera_info_.k[0];
        double fy = camera_info_.k[4];
        double cx = camera_info_.k[2];
        double cy = camera_info_.k[5];

        // Look up transform from camera_depth_frame to base_link at this timestamp
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(
                "base_link",            // target frame
                "camera_depth_frame",   // source frame
                msg->header.stamp,
                tf2::durationFromSec(0.05));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Convert to tf2::Transform for efficient point transformation
        tf2::Transform tf_cam_to_base;
        tf2::fromMsg(transform_stamped.transform, tf_cam_to_base);

        // Prepare point cloud message
        sensor_msgs::msg::PointCloud2 pc_msg;
        pc_msg.header.stamp = msg->header.stamp;
        pc_msg.header.frame_id = "base_link";   // publish in base_link frame

        // Define fields: x, y, z
        pc_msg.fields.resize(3);
        pc_msg.fields[0].name = "x";
        pc_msg.fields[0].offset = 0;
        pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc_msg.fields[0].count = 1;

        pc_msg.fields[1].name = "y";
        pc_msg.fields[1].offset = 4;
        pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc_msg.fields[1].count = 1;

        pc_msg.fields[2].name = "z";
        pc_msg.fields[2].offset = 8;
        pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc_msg.fields[2].count = 1;

        pc_msg.point_step = 12;  // 3 * 4 bytes
        pc_msg.width = 0;        // will be set later
        pc_msg.height = 1;
        pc_msg.is_bigendian = false;
        pc_msg.is_dense = false;

        // Allocate temporary buffer
        std::vector<float> points;
        points.reserve(depth.rows * depth.cols * 3 / (downsample_step_ * downsample_step_));

        // Iterate over depth image with downsampling
        for (int v = 0; v < depth.rows; v += downsample_step_)
        {
            const float* row = depth.ptr<float>(v);
            for (int u = 0; u < depth.cols; u += downsample_step_)
            {
                float d = row[u];
                if (std::isnan(d) || std::isinf(d) || d < min_depth_ || d > max_depth_)
                    continue;

                // Point in camera frame
                float x_cam = (u - cx) * d / fx;
                float y_cam = (v - cy) * d / fy;
                float z_cam = d;

                // Transform to base_link frame
                tf2::Vector3 p_cam(x_cam, y_cam, z_cam);
                tf2::Vector3 p_base = tf_cam_to_base * p_cam;

                points.push_back(p_base.x());
                points.push_back(p_base.y());
                points.push_back(p_base.z());
            }
        }

        pc_msg.width = points.size() / 3;
        if (pc_msg.width == 0)
            return;

        pc_msg.row_step = pc_msg.width * pc_msg.point_step;
        pc_msg.data.resize(pc_msg.row_step);
        memcpy(pc_msg.data.data(), points.data(), pc_msg.data.size());

        pc_pub_->publish(pc_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::CameraInfo camera_info_;
    bool camera_info_received_ = false;
    rclcpp::Time last_pub_time_;

    double max_freq_;
    double min_depth_;
    double max_depth_;
    int downsample_step_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
