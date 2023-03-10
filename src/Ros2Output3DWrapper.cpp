//
// Created by tp on 16/12/22.
//

#include "Ros2Output3DWrapper.h"
#include "FullSystem/ImmaturePoint.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"


namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;

    namespace IOWrap {
        Ros2Output3DWrapper::Ros2Output3DWrapper(rclcpp::Node &node, int width, int height) : node(node) {
            firstPoseTransform.translation().setZero();
            broadcasterOdometryTransform = std::make_unique<tf2_ros::TransformBroadcaster>(node);
            msgOdometryTransform.header.frame_id = "odom";
            msgOdometryTransform.child_frame_id = "base_link";

            publisherOdometry = node.create_publisher<nav_msgs::msg::Odometry>("dsso/cam_odemetry", 10);
            msgOdometry.header.frame_id = "odom";
            msgOdometry.child_frame_id = "base_link";

            publisherCamPointCloud = node.create_publisher<sensor_msgs::msg::PointCloud2>("dsso/cam_pointcloud", 10);
            msgPc.header.frame_id = "map";
            sensor_msgs::PointCloud2Modifier pcMod(msgPc);
            pcMod.setPointCloud2FieldsByString(1, "xyz");

            publisherCamPath = node.create_publisher<nav_msgs::msg::Path>("dsso/cam_path", 1);
            msgPath.header.frame_id = "map";
        }

        Ros2Output3DWrapper::~Ros2Output3DWrapper() noexcept {

        }

        void Ros2Output3DWrapper::publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) {
            for (
                const std::pair<uint64_t, Eigen::Vector2i> &p
                    : connectivity) {
                int idHost = p.first >> 32;
                int idTarget = p.first & ((uint64_t) 0xFFFFFFFF);
            }

        }

        struct PointAdder {
            CalibHessian *ch;
            sensor_msgs::PointCloud2Iterator<float> iter_x;
            sensor_msgs::PointCloud2Iterator<float> iter_y;
            sensor_msgs::PointCloud2Iterator<float> iter_z;
            Eigen::Matrix4f p;

            PointAdder(CalibHessian *calibHessian, sensor_msgs::msg::PointCloud2 &pointCloudMessage) : ch(calibHessian),
                                                                                                      iter_x(pointCloudMessage, "x"),
                                                                                                      iter_y(pointCloudMessage, "y"),
                                                                                                      iter_z(pointCloudMessage, "z"),
                                                                                                      p(Eigen::Matrix4f::Identity()) {
            }

            void setFrameHessian(FrameHessian *fh) {
                p = fh->PRE_camToWorld.matrix().cast<float>();
            }

            void addPoint(float x, float y, float z) {
                Eigen::Vector4f pointV(((x - ch->cxl()) * ch->fxli()) * z,((y - ch->cxl()) * ch->fxli()) * z, z, 1.0);
                Eigen::Vector4f projectedPointV = p * pointV;

                *iter_x = projectedPointV.x();
                ++iter_x;
                *iter_y = projectedPointV.y();
                ++iter_y;
                *iter_z = projectedPointV.z();
                ++iter_z;
            }
        };

        void
        Ros2Output3DWrapper::publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib) {
            // publishKeyframes() is call twice!?
            if (frames.size() <= 1) {
                return;
            }

            RCLCPP_INFO(node.get_logger(), "frames.size = %lu)", frames.size());

            msgPath.poses.resize(frames.size());
            int i = 0;
            int pointCount = 0;
            for (FrameHessian *fh : frames) {
                geometry_msgs::msg::PoseStamped &pose = msgPath.poses.at(i++);
                Sophus::Vector3d &t = fh->PRE_camToWorld.translation();
                pose.pose.position.x = t.x();
                pose.pose.position.y = t.y();
                pose.pose.position.z = t.z();
                const Eigen::Quaternion<double> &q = fh->PRE_camToWorld.so3().unit_quaternion();
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();

                pointCount += fh->immaturePoints.size();
                pointCount += fh->pointHessians.size();
                pointCount += fh->pointHessiansMarginalized.size();
                pointCount += fh->pointHessiansOut.size();
            }
            publisherCamPath->publish(msgPath);

            if (pointCount != msgPc.data.size()) {
                sensor_msgs::PointCloud2Modifier pcMod(msgPc);
                pcMod.resize(pointCount);
            }

            PointAdder pa(HCalib, msgPc);
            for (FrameHessian *fh : frames) {
                pa.setFrameHessian(fh);
                for (ImmaturePoint *ip : fh->immaturePoints) {
                    float z = 2.0 / (ip->idepth_max + ip->idepth_min);
                    pa.addPoint(ip->u, ip->v, z);
                }
                for (PointHessian *ph: fh->pointHessians) {
                    float z = 1.0 / ph->idepth_scaled;
                    pa.addPoint(ph->u, ph->v, z);
                }
                for (PointHessian *ph : fh->pointHessiansMarginalized) {
                    float z = 1.0 / ph->idepth_scaled;
                    pa.addPoint(ph->u, ph->v, z);
                }
                for (PointHessian *ph : fh->pointHessiansOut) {
                    float z = 1.0 / ph->idepth_scaled;
                    pa.addPoint(ph->u, ph->v, z);
                }
            }

            msgPc.header.stamp = node.get_clock()->now();
            publisherCamPointCloud->publish(msgPc);

        }

        void Ros2Output3DWrapper::publishCamPose(FrameShell *frame, CalibHessian *HCalib) {
            RCLCPP_INFO(node.get_logger(), "publishCamPose()");

            msgOdometryTransform.header.stamp = node.get_clock()->now();
            msgOdometry.header.stamp = node.get_clock()->now();

            SE3 &poseTransform = frame->camToWorld;
//            if (firstPoseTransform.translation().isZero()) {
//                firstPoseTransform = poseTransform.inverse();
//            }

            //SE3 relPoseTransform = firstPoseTransform * poseTransform;
            Eigen::Matrix<double, 3, 1> &t = poseTransform.translation();
            const Eigen::Quaternion<double> &q = poseTransform.so3().unit_quaternion();

            msgOdometryTransform.transform.translation.x = t.x();
            msgOdometryTransform.transform.translation.y = t.y();
            msgOdometryTransform.transform.translation.z = t.z();
            msgOdometryTransform.transform.rotation.x = q.x();
            msgOdometryTransform.transform.rotation.y = q.y();
            msgOdometryTransform.transform.rotation.z = q.z();
            msgOdometryTransform.transform.rotation.w = q.w();
            broadcasterOdometryTransform->sendTransform(msgOdometryTransform);

            geometry_msgs::msg::PoseWithCovariance pose;
            pose.pose.position.x = t.x();
            pose.pose.position.y = t.y();
            pose.pose.position.z = t.z();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            msgOdometry.pose = pose;
            //msgCamOdometry.twist = ...
            publisherOdometry->publish(msgOdometry);
        };

        void Ros2Output3DWrapper::pushLiveFrame(FrameHessian *image) {

        };

        void Ros2Output3DWrapper::pushDepthImage(MinimalImageB3 *imageLeft, MinimalImageB3 *imageRight) {

        };

        bool Ros2Output3DWrapper::needPushDepthImage() {
            return false;
        };

        void Ros2Output3DWrapper::pushDepthImageFloat(MinimalImageF *image, FrameHessian *KF) {

            for (int y = 0; y < image->h; y++) {
                for (int x = 0; x < image->w; x++) {
                    if (image->at(x, y) <= 0) {
                        continue;
                    }
                }
            }
        };
    }
}
