//
// Created by tp on 17/11/22.
//
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "util/NumType.h"
#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "Ros2Output3DWrapper.h"

#include "cv_bridge/cv_bridge.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

class DssoRos : public rclcpp::Node {
public:
    DssoRos() : Node("dsso_ros") {
        this->declare_parameter("gammaFile", "");
        this->declare_parameter("vignetteFile", "");
        this->declare_parameter("calibFileL", "");
        this->declare_parameter("calibFileR", "");

        std::string gammaFile =
                this->get_parameter("gammaFile").get_parameter_value().get<std::string>();
        std::string vignetteFile =
                this->get_parameter("vignetteFile").get_parameter_value().get<std::string>();
        std::string calibFileL =
                this->get_parameter("calibFileL").get_parameter_value().get<std::string>();
        std::string calibFileR =
                this->get_parameter("calibFileR").get_parameter_value().get<std::string>();

        dso::disableAllDisplay = false;

        dso::setting_desiredImmatureDensity = 1000;
        dso::setting_desiredPointDensity = 1200;
        dso::setting_minFrames = 5;
        dso::setting_maxFrames = 7;
        dso::setting_maxOptIterations=4;
        dso::setting_minOptIterations=1;
        dso::setting_logStuff = false;
        dso::setting_kfGlobalWeight = 1.3;
        dso::setting_photometricCalibration = 2;
        dso::setting_affineOptModeA = 0;
        dso::setting_affineOptModeB = 0;

        undistorterL = dso::Undistort::getUndistorterForFile(calibFileL, gammaFile, vignetteFile, false);
        if (!undistorterL) abort();
        undistorterR = dso::Undistort::getUndistorterForFile(calibFileR, gammaFile, vignetteFile, true);
        if (!undistorterR) abort();

        Eigen::Matrix<double, 4, 4> leftExtrinsics;
        //leftExtrinsics << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0, 0.0, 1.0;
        leftExtrinsics.setIdentity();
        //leftExtrinsics << 0.9989, 0.0107, -0.0464, 0, -0.0108, 0.9999, -0.0009, 0, 0.0464, 0.0014, 0.9989, 0, 0, 0, 0, 1.0000;
        Eigen::Matrix<double, 4, 4> rightExtrinsics;
        //rightExtrinsics << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0, 0.0, 1.0;
        rightExtrinsics.setIdentity();
        rightExtrinsics(0,3) = 0.06; // 60mm baseline;
        //rightExtrinsics << 0.9992, 0.0116, -0.0374, 0.0605, -0.0116, 0.9999, 0.0013, 0.0007, 0.0374, -0.0009, 0.9993, -0.0023, 0, 0, 0, 1.0000;

        // leftToRight is the transform for a point between the cameras, not the camera transform..  inv(inv(leftExtrinsics) * rightExtrinsics))
        leftToRight = dso::SE3(rightExtrinsics).inverse() * dso::SE3(leftExtrinsics);

        width = undistorterL->getSize()[0];
        height = undistorterL->getSize()[1];

        dso::setGlobalCalib(width, height);
        iae = std::unique_ptr<dso::ImageAndExposure>(new dso::ImageAndExposure(width, height));
        fullSystem = std::unique_ptr<dso::FullSystem>(
                new dso::FullSystem(undistorterL->getK(), undistorterR->getK(), leftToRight));
        fullSystem->linearizeOperation = false;

        //outputWrapper = std::unique_ptr<dso::IOWrap::Output3DWrapper>(new dso::IOWrap::Ros2Output3DWrapper(*this, width, height));
        outputWrapper = std::unique_ptr<dso::IOWrap::Output3DWrapper>(new dso::IOWrap::PangolinDSOViewer(width, height, true));
        fullSystem->outputWrapper.push_back(outputWrapper.get());

//        dso::IOWrap::PangolinDSOViewer *viewer = 0;
//        viewer = new dso::IOWrap::PangolinDSOViewer(width, height, true);
//        fullSystem->outputWrapper.push_back(viewer);

        std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> fnc;
        fnc = std::bind(&DssoRos::camImageCallback, this, std::placeholders::_1, false);
        subscriptionCamLeft = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 10, fnc);
        fnc = std::bind(&DssoRos::camImageCallback, this, std::placeholders::_1, true);
        subscriptionCamRight = this->create_subscription<sensor_msgs::msg::Image>("/cam1/image_raw", 10, fnc);
    }

private:
    void camImageCallback(const sensor_msgs::msg::Image::SharedPtr img, const bool rightNotLeft) {
        RCLCPP_INFO(this->get_logger(), "Got '%s' image with time stamp %d.%d", rightNotLeft ? "right" : "left",
                    img->header.stamp.sec, img->header.stamp.nanosec);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        assert(cv_ptr->image.type() == CV_8U);
        assert(cv_ptr->image.channels() == 1);

        dso::MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows,
                                  (unsigned char *) cv_ptr->image.data);

        if (rightNotLeft) {
            undistorterR->undistort<unsigned char>(&minImg, iae.get(), 1.0f);
        } else {
            undistorterL->undistort<unsigned char>(&minImg, iae.get(), 1.0f);
        }

        if (lastStamp != img->header.stamp) {
            lastStamp = img->header.stamp;
        } else {
            iae->timestamp = img->header.stamp.sec + img->header.stamp.nanosec * 1e-9;
            if (dso::setting_fullResetRequested) {
                std::vector<dso::IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
                for (dso::IOWrap::Output3DWrapper *ow: wraps) ow->reset();
                fullSystem = std::unique_ptr<dso::FullSystem>(
                        new dso::FullSystem(undistorterL->getK(), undistorterR->getK(), leftToRight));
                fullSystem->linearizeOperation = false;
                fullSystem->outputWrapper = wraps;
                if (undistorterL->photometricUndist != 0)
                    fullSystem->setGammaFunction(undistorterL->photometricUndist->getG());
                dso::setting_fullResetRequested = false;
            }

            fullSystem->addActiveFrame(iae.get(), frameID);
            frameID++;
        }
    }

    const dso::Undistort *undistorterL, *undistorterR;
    dso::SE3 leftToRight;
    int width, height;
    std::unique_ptr<dso::ImageAndExposure> iae;
    std::unique_ptr<dso::FullSystem> fullSystem;
    std::unique_ptr<dso::IOWrap::Output3DWrapper> outputWrapper;
    int frameID = 0;
    builtin_interfaces::msg::Time_<std::allocator<void>> lastStamp;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionCamLeft;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionCamRight;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DssoRos>());
    rclcpp::shutdown();

    return 0;
}
