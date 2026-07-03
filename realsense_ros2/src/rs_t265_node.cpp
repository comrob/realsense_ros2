/*
* ROS wrapper for Realsense t265 camera
* By: Juan Galvis
* https://github.com/jdgalviss
*
* This code is free software: you can redistribute it and/or modify
* it under the terms of the MIT License.
*
* This code is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
*/
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/convert.h>

using namespace std::chrono_literals;

/*! T265 Node class */
class T265Node : public rclcpp::Node
{
public:
  T265Node()
      : Node("t265_node"), tf_broadcaster_(this), static_tf_broadcaster_(this)
  {
    publish_fisheye_ = this->declare_parameter<bool>("publish_fisheye", false);
    enable_pose_jumping_ = this->declare_parameter<bool>("enable_pose_jumping", false);
    enable_relocalization_ = this->declare_parameter<bool>("enable_relocalization", false);

    //begin_ = std::chrono::steady_clock::now();
    // Define configuration to start stream from t265 camera
    cfg_.enable_stream(RS2_STREAM_ACCEL);
    cfg_.enable_stream(RS2_STREAM_GYRO);
    cfg_.enable_stream(RS2_STREAM_POSE);
    if (publish_fisheye_)
    {
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, 848, 800, RS2_FORMAT_Y8, 30);
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, 848, 800, RS2_FORMAT_Y8, 30);
    }
    // Start pipeline with chosen configuration
    pipe_.start(cfg_);
    ConfigurePoseSensorOptions();

    // Publishers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("rs_t265/odom", 10);
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("rs_t265/imu", 10);
    if (publish_fisheye_)
    {
      fisheye1_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rs_t265/fisheye1/image_raw", 10);
      fisheye2_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rs_t265/fisheye2/image_raw", 10);
      fisheye1_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rs_t265/fisheye1/camera_info", 10);
      fisheye2_camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rs_t265/fisheye2/camera_info", 10);

      PublishFisheyeStaticTransforms();
    }
    // Timer used to publish camera's odometry periodically
    timer_ = this->create_wall_timer(
        10ms, std::bind(&T265Node::TimerCallback, this)); //30ms
  }

private:
  void ConfigurePoseSensorOptions()
  {
    try
    {
      auto active_profile = pipe_.get_active_profile();
      auto device = active_profile.get_device();

      for (auto &&sensor : device.query_sensors())
      {
        if (sensor.supports(RS2_OPTION_ENABLE_POSE_JUMPING))
        {
          sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, enable_pose_jumping_ ? 1.0f : 0.0f);
        }

        if (sensor.supports(RS2_OPTION_ENABLE_RELOCALIZATION))
        {
          sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, enable_relocalization_ ? 1.0f : 0.0f);
        }
      }
    }
    catch (const rs2::error &e)
    {
      RCLCPP_WARN(logger_, "Could not configure T265 pose options: %s", e.what());
    }
  }

  geometry_msgs::msg::Transform ToRosTransform(const rs2_extrinsics &extrinsics)
  {
    geometry_msgs::msg::Transform transform;

    // Basis change used in this node for pose stream: [x y z]_ros = B * [x y z]_rs.
    const tf2::Matrix3x3 B(
      0.0, 0.0, -1.0,
      -1.0, 0.0, 0.0,
      0.0, 1.0, 0.0);

    // librealsense stores rs2_extrinsics::rotation in column-major order.
    tf2::Matrix3x3 rs_R(
      extrinsics.rotation[0], extrinsics.rotation[3], extrinsics.rotation[6],
      extrinsics.rotation[1], extrinsics.rotation[4], extrinsics.rotation[7],
      extrinsics.rotation[2], extrinsics.rotation[5], extrinsics.rotation[8]);

    // Proper rotation conversion between coordinate conventions.
    const tf2::Matrix3x3 ros_R = B * rs_R * B.transpose();

    const tf2::Vector3 rs_t(
      extrinsics.translation[0],
      extrinsics.translation[1],
      extrinsics.translation[2]);
    const tf2::Vector3 ros_t = B * rs_t;

    transform.translation.x = ros_t.x();
    transform.translation.y = ros_t.y();
    transform.translation.z = ros_t.z();

    tf2::Quaternion ros_q;
    ros_R.getRotation(ros_q);

    transform.rotation.x = ros_q.x();
    transform.rotation.y = ros_q.y();
    transform.rotation.z = ros_q.z();
    transform.rotation.w = ros_q.w();
    return transform;
  }

  void PublishFisheyeStaticTransforms()
  {
    try
    {
      auto active_profile = pipe_.get_active_profile();
      auto pose_profile = active_profile.get_stream(RS2_STREAM_POSE).as<rs2::stream_profile>();
      auto fisheye1_profile = active_profile.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::stream_profile>();
      auto fisheye2_profile = active_profile.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::stream_profile>();

      auto extr_pose_to_fisheye1 = pose_profile.get_extrinsics_to(fisheye1_profile);
      auto extr_pose_to_fisheye2 = pose_profile.get_extrinsics_to(fisheye2_profile);

      geometry_msgs::msg::TransformStamped tf_fisheye1;
      tf_fisheye1.header.stamp = rclcpp::Clock().now();
      tf_fisheye1.header.frame_id = "t265_frame";
      tf_fisheye1.child_frame_id = "t265_fisheye1_frame";
      tf_fisheye1.transform = ToRosTransform(extr_pose_to_fisheye1);
      ApplyFisheyeRotationCorrection(tf_fisheye1.transform);

      geometry_msgs::msg::TransformStamped tf_fisheye2;
      tf_fisheye2.header.stamp = rclcpp::Clock().now();
      tf_fisheye2.header.frame_id = "t265_frame";
      tf_fisheye2.child_frame_id = "t265_fisheye2_frame";
      tf_fisheye2.transform = ToRosTransform(extr_pose_to_fisheye2);
      ApplyFisheyeRotationCorrection(tf_fisheye2.transform);

      static_tf_broadcaster_.sendTransform(tf_fisheye1);
      static_tf_broadcaster_.sendTransform(tf_fisheye2);
    }
    catch (const rs2::error &e)
    {
      RCLCPP_WARN(logger_, "Could not publish fisheye static TF from RealSense extrinsics: %s", e.what());
    }
  }

  void ApplyFisheyeRotationCorrection(geometry_msgs::msg::Transform &transform)
  {
    tf2::Quaternion q_current(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w);

    tf2::Quaternion q_correction;
    const double roll_rad = 90.0 * M_PI / 180.0;
    const double yaw_rad = -90.0 * M_PI / 180.0;
    q_correction.setRPY(roll_rad, 0.0, yaw_rad);

    tf2::Quaternion q_corrected = q_current * q_correction;
    q_corrected.normalize();

    transform.rotation.x = q_corrected.x();
    transform.rotation.y = q_corrected.y();
    transform.rotation.z = q_corrected.z();
    transform.rotation.w = q_corrected.w();
  }

  std::string DistortionModelToRos(rs2_distortion model)
  {
    if (model == RS2_DISTORTION_FTHETA || model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
      return "equidistant";
    }
    return "plumb_bob";
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfo(const rs2::video_stream_profile &profile, const std::string &frame_id, const rclcpp::Time &stamp)
  {
    auto intrinsics = profile.get_intrinsics();

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = intrinsics.width;
    info.height = intrinsics.height;
    info.distortion_model = DistortionModelToRos(intrinsics.model);
    info.d = {
        intrinsics.coeffs[0],
        intrinsics.coeffs[1],
        intrinsics.coeffs[2],
        intrinsics.coeffs[3],
        intrinsics.coeffs[4]};
    info.k = {
        intrinsics.fx, 0.0, intrinsics.ppx,
        0.0, intrinsics.fy, intrinsics.ppy,
        0.0, 0.0, 1.0};
    info.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    info.p = {
        intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
        0.0, intrinsics.fy, intrinsics.ppy, 0.0,
        0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void TimerCallback()
  {
    //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //RCLCPP_INFO(logger_, "t265 timer period: %d",std::chrono::duration_cast<std::chrono::milliseconds>(end - begin_).count());
    //begin_=end;

    // Wait for the next set of frames from the camera
    auto frameset = pipe_.wait_for_frames();

    // Find and retrieve IMU and/or tracking data

    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
      rs2_vector gyro_sample = gyro_frame.get_motion_data();
      imu_msg_.angular_velocity.x = gyro_sample.x;
      imu_msg_.angular_velocity.y = gyro_sample.y;
      imu_msg_.angular_velocity.z = gyro_sample.z;
    }

    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
      rs2_vector accel_sample = accel_frame.get_motion_data();
      imu_msg_.header.stamp = rclcpp::Clock().now();
      imu_msg_.header.frame_id = "t265_frame";
      imu_msg_.linear_acceleration.x = accel_sample.x;
      imu_msg_.linear_acceleration.y = accel_sample.y;
      imu_msg_.linear_acceleration.z = accel_sample.z;
      imu_publisher_->publish(imu_msg_);
    }

    if (publish_fisheye_)
    {
      if (rs2::video_frame fisheye1_frame = frameset.get_fisheye_frame(1))
      {
        const auto stamp = rclcpp::Clock().now();
        cv::Mat fisheye1_image(
            fisheye1_frame.get_height(),
            fisheye1_frame.get_width(),
            CV_8UC1,
            const_cast<void *>(fisheye1_frame.get_data()),
            fisheye1_frame.get_stride_in_bytes());

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", fisheye1_image).toImageMsg();
        msg->header.stamp = stamp;
        msg->header.frame_id = "t265_fisheye1_frame";
        fisheye1_publisher_->publish(*msg);

        auto fisheye1_profile = fisheye1_frame.get_profile().as<rs2::video_stream_profile>();
        fisheye1_camera_info_publisher_->publish(BuildCameraInfo(fisheye1_profile, "t265_fisheye1_frame", stamp));
      }

      if (rs2::video_frame fisheye2_frame = frameset.get_fisheye_frame(2))
      {
        const auto stamp = rclcpp::Clock().now();
        cv::Mat fisheye2_image(
            fisheye2_frame.get_height(),
            fisheye2_frame.get_width(),
            CV_8UC1,
            const_cast<void *>(fisheye2_frame.get_data()),
            fisheye2_frame.get_stride_in_bytes());

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", fisheye2_image).toImageMsg();
        msg->header.stamp = stamp;
        msg->header.frame_id = "t265_fisheye2_frame";
        fisheye2_publisher_->publish(*msg);

        auto fisheye2_profile = fisheye2_frame.get_profile().as<rs2::video_stream_profile>();
        fisheye2_camera_info_publisher_->publish(BuildCameraInfo(fisheye2_profile, "t265_fisheye2_frame", stamp));
      }
    }

    

    if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
    {
      rs2_pose pose_data = pose_frame.get_pose_data();
      // Create odometry msg and publish
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "t265_frame";
      odom_msg.header.stamp = rclcpp::Clock().now();
      odom_msg.pose.pose.position.x = -pose_data.translation.z;
      odom_msg.pose.pose.position.y = -pose_data.translation.x;
      odom_msg.pose.pose.position.z = pose_data.translation.y;

      odom_msg.pose.pose.orientation.x = -pose_data.rotation.z;
      odom_msg.pose.pose.orientation.y = -pose_data.rotation.x;
      odom_msg.pose.pose.orientation.z = pose_data.rotation.y;
      odom_msg.pose.pose.orientation.w = pose_data.rotation.w;

      double r = 0.0f, p = 0.0f, y = 0.0f;
      tf2::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(r, p, y);

      double vel_x = -pose_data.velocity.z;
      double vel_y = -pose_data.velocity.x;
      odom_msg.twist.twist.linear.x = vel_x*cos(y) + vel_y*sin(y);
      odom_msg.twist.twist.linear.y = -vel_x*sin(y) + vel_y*cos(y);
      odom_msg.twist.twist.linear.z = pose_data.velocity.y;

      odom_msg.twist.twist.angular.x = -pose_data.angular_velocity.z;
      odom_msg.twist.twist.angular.y = -pose_data.angular_velocity.x;
      odom_msg.twist.twist.angular.z = pose_data.angular_velocity.y;
      odom_publisher_->publish(odom_msg);

      // Publish tf data
      tf2_msgs::msg::TFMessage tfs;
      geometry_msgs::msg::TransformStamped tf;
      
      tf.header.frame_id = "odom";
      tf.child_frame_id = "t265_frame";
      tf.transform.translation.x = -pose_data.translation.z;
      tf.transform.translation.y = -pose_data.translation.x;
      tf.transform.translation.z = pose_data.translation.y;
      tf.transform.rotation.x = -pose_data.rotation.z;
      tf.transform.rotation.y = -pose_data.rotation.x;
      tf.transform.rotation.z = pose_data.rotation.y;
      tf.transform.rotation.w = pose_data.rotation.w;
      tf.header.stamp = rclcpp::Clock().now();
      tf_broadcaster_.sendTransform(tf);
    }
  }
  // Class Members
  // std::chrono::steady_clock::time_point begin_;
  rclcpp::Logger logger_ = rclcpp::get_logger("T265Node");
  geometry_msgs::msg::TransformStamped tf_static_;
  sensor_msgs::msg::Imu imu_msg_; 
  bool publish_fisheye_;
  bool enable_pose_jumping_;
  bool enable_relocalization_;
  bool publish_transform_to_depth_;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe_;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fisheye1_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fisheye2_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr fisheye1_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr fisheye2_camera_info_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

int main(int argc, char **argv)
{
  printf("starting rs_t265 node\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}
