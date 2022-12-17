//
// Created by tp on 16/12/22.
//

#pragma once

#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

namespace dso {

    class FrameHessian;

    class CalibHessian;

    class FrameShell;

    namespace IOWrap {

        class Ros2Output3DWrapper : public Output3DWrapper {
        public:
            Ros2Output3DWrapper(int width, int height);

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

        };
    }
}
