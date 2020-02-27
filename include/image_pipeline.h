#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>

class ImagePipeline
{
 private:
    cv::Mat img;
    bool isValid;
    image_transport::Subscriber sub;
 public:
    explicit ImagePipeline(ros::NodeHandle& n);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    int get_template_ID(Boxes& boxes);
};
