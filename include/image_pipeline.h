#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include "boxes.h"

// FLANN-based keypoint matching constants
#define NUM_REMATCH 2
#define REMATCH_THRESH 30
#define GOOD_MATCH_DIST 0.2
#define MIN_HESSIAN 600

// possible template IDs
enum TEMPLATE
{
    UNINITIALIZED = -1,
    BLANK = 0,
    RAISIN_BRAN = 1,
    CIN_TOAST = 2,
    RICE_KRISP = 3
};

struct ImageFeatures
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
};

class ImagePipeline
{
 private:
    cv::Mat scene_img;
    bool is_valid;
    image_transport::Subscriber sub;
    std::vector<ImageFeatures> box_features;
    cv::Ptr<cv::xfeatures2d::SURF> detector;
    cv::FlannBasedMatcher matcher;
    TEMPLATE templateID;
    void load_template_features(const Boxes& boxes);
    void match_to_templates(const Boxes& boxes);
    int match_to_template(const ImageFeatures& template_features, const ImageFeatures& feature);

 public:
    explicit ImagePipeline(ros::NodeHandle& n, const Boxes& boxes);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    int get_template_ID(const Boxes& boxes);
};
