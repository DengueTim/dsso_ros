//
// Created by tp on 16/12/22.
//

#pragma once

#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
//#include "geometry_msgs/msg/transform.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;

    namespace IOWrap {

        class Ros2Output3DWrapper : public Output3DWrapper {
        public:
            Ros2Output3DWrapper(rclcpp::Node &node, int width, int height);

            virtual ~Ros2Output3DWrapper();

            virtual void publishGraph(
                    const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                            Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override;

            virtual void
            publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib) override;


            virtual void publishCamPose(FrameShell *frame, CalibHessian *HCalib) override;

            virtual void pushLiveFrame(FrameHessian *image) override;

            virtual void pushDepthImage(MinimalImageB3 *imageLeft, MinimalImageB3 *imageRight) override;

            virtual bool needPushDepthImage() override;

            virtual void pushDepthImageFloat(MinimalImageF *image, FrameHessian *KF) override;

        private:
            rclcpp::Node &node;

            std::shared_ptr<tf2_ros::TransformBroadcaster> broadcasterOdometryTransform;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherOdometry;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherCamPointCloud;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherCamPath;

            geometry_msgs::msg::TransformStamped msgOdometryTransform;
            nav_msgs::msg::Odometry msgOdometry;
            sensor_msgs::msg::PointCloud2 msgPc;
            nav_msgs::msg::Path msgPath;

            SE3 firstPoseTransform;
        };
    }
}
