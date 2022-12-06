//
// Created by tp on 17/11/22.
//
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "util/NumType.h"
#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include "cv_bridge/cv_bridge.h"

class DssoRos : public rclcpp::Node {
public:
    DssoRos(const dso::Undistort *undistorterL, const dso::Undistort *undistorterR, const dso::SE3 &leftToRight) :
            Node("DssoRos"), undistorterL(undistorterL), undistorterR(undistorterR), leftToRight(leftToRight),
            width(undistorterL->getSize()[0]),
            height(undistorterL->getSize()[1]) {

        std::string my_param =
                this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

        dso::setGlobalCalib(width, height);
        iae = std::unique_ptr<dso::ImageAndExposure>(new dso::ImageAndExposure(width, height));
        fullSystem = std::unique_ptr<dso::FullSystem>(
                new dso::FullSystem(undistorterL->getK(), undistorterR->getK(), leftToRight));

        std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> fnc;
        fnc = std::bind(&DssoRos::camImageCallback, this, std::placeholders::_1, false);
        subscriptionCamLeft = this->create_subscription<sensor_msgs::msg::Image>("/cam0/image_raw", 10, fnc);
        fnc = std::bind(&DssoRos::camImageCallback, this, std::placeholders::_1, true);
        subscriptionCamRight = this->create_subscription<sensor_msgs::msg::Image>("/cam1/image_raw", 10, fnc);

        publisherCamTransforms = this->create_publisher<geometry_msgs::msg::Transform>("dsso/cam_transforms", 10);
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
        iae->timestamp = img->header.stamp.sec;


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

    const dso::Undistort *const undistorterL, *const undistorterR;
    const dso::SE3 leftToRight;
    const int width, height;
    std::unique_ptr<dso::ImageAndExposure> iae;
    std::unique_ptr<dso::FullSystem> fullSystem;
    int frameID = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionCamLeft;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionCamRight;

    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisherCamTransforms;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    dso::Undistort *undistorterL = 0; //dso::Undistort::getUndistorterForFile(calibFileL, gammaFile, vignetteFile, false);
    dso::Undistort *undistorterR = 0; //dso::Undistort::getUndistorterForFile(calibFileR, gammaFile, vignetteFile, true);

    Eigen::Matrix<double, 4, 4> leftExtrinsics;
    leftExtrinsics << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 4, 4> rightExtrinsics;
    rightExtrinsics << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0, 0.0, 1.0;
    // leftToRight is the transform for a point between the cameras, not the camera transform..  inv(inv(leftExtrinsics) * rightExtrinsics))
    dso::SE3 leftToRight = dso::SE3(rightExtrinsics).inverse() * dso::SE3(leftExtrinsics);

    rclcpp::spin(std::make_shared<DssoRos>(undistorterL, undistorterR, leftToRight));
    rclcpp::shutdown();

    delete undistorterL;
    delete undistorterR;
    return 0;
}
