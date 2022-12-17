//
// Created by tp on 16/12/22.
//

#include "Ros2Output3DWrapper.h"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;

    namespace IOWrap {

        Ros2Output3DWrapper::Ros2Output3DWrapper(int width, int height) {
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

        void
        Ros2Output3DWrapper::publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib) {
            for (FrameHessian *f: frames) {
                for (PointHessian *p: f->pointHessians) {

                }
            }
        }

        void Ros2Output3DWrapper::publishCamPose(FrameShell *frame, CalibHessian *HCalib) {

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
