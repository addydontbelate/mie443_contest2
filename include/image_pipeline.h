#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include "boxes.h"
#include "logger.h"

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_US std::chrono::duration_cast<std::chrono::microseconds>

// FLANN-based keypoint matching constants
#define NUM_REMATCH 2
#define REMATCH_THRESH 20
#define GOOD_MATCH_DIST 0.2
#define MIN_HESSIAN 400
#define RATIO_THRESH 0.75
#define MIN_AREA 75

// log file
#define VISLOG_FILEPATH "/home/turtlebot/Desktop/vision_log.txt"

// possible template IDs
enum TEMPLATE
{
    UNINITIALIZED = -1,
    BLANK = 0,
    RAISIN_BRAN = 1,
    CIN_TOAST = 2,
    RICE_KRISP = 3
};

const std::string TEMPLATE_NAME[] = {"Blank", "Raisin Bran", "Cinnamon Toast Crunch", "Rice Krispies"};

struct ImageFeatures
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
};

class ImagePipeline
{
 private:
    Logger logger;
    cv::Mat scene_img;
    bool is_valid;
    image_transport::Subscriber sub;
    std::vector<ImageFeatures> box_features;
    cv::Ptr<cv::xfeatures2d::SURF> flann_detector;
    cv::FlannBasedMatcher flann_matcher;
    TEMPLATE templateID;
    void load_template_features(const Boxes&);
    TEMPLATE match_to_templates_flann_dist(const Boxes&);
    TEMPLATE match_to_templates_flann_knn(const Boxes&);
    TEMPLATE match_to_templates_homog(const Boxes&);
    int match_to_template_flann_dist(const ImageFeatures&, const ImageFeatures&);
    int match_to_template_flann_knn(const ImageFeatures&, const ImageFeatures&);
    int match_to_template_homog(const ImageFeatures&, const cv::Mat&, const ImageFeatures&);
    TEMPLATE get_majority_template(const std::vector<TEMPLATE>&);

 public:
    explicit ImagePipeline(ros::NodeHandle& n, const Boxes& boxes);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    int get_template_ID(const Boxes& boxes);
};
