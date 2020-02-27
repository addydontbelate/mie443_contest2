#include <image_pipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw"; webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n, const Boxes& boxes)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::image_callback, this);
    is_valid = false;
    templateID = TEMPLATE::UNINITIALIZED;
    detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);

    load_template_features(boxes);
}

void ImagePipeline::load_template_features(const Boxes& boxes)
{
    // store features of each box object/template to match against later
    for (const auto& box : boxes.templates)
    {
        ImageFeatures features;
        detector->detectAndCompute(box, cv::Mat(), features.keypoints, features.descriptors);
        box_features.push_back(features);
    }

    if (box_features.empty())
        ROS_WARN("[IMG_PIPE] No templates were contained in boxes! No features added.");
}

void ImagePipeline::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        if(is_valid)
            scene_img.release();

        scene_img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        is_valid = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("[IMG_PIPE] Could not convert from %s to %s!",
                msg->encoding.c_str(), IMAGE_TYPE.c_str());
        is_valid = false;
    }
}

int ImagePipeline::get_template_ID(const Boxes& boxes)
{
    // reset template ID
    templateID = TEMPLATE::UNINITIALIZED;
    ros::spinOnce();

    // check if scene image is valid
    if (!is_valid)
    {
        ROS_ERROR("[IMG_PIPE] Invalid scene image! Skipping detection procedure.");
    }
    else if(scene_img.empty() || scene_img.rows <= 0 || scene_img.cols <= 0)
    {
        ROS_ERROR("[IMG_PIPE] Valid scene image but a problem exists!");
        ROS_INFO("scene_img.empty(): %s", scene_img.empty() ? "true" : "false");
        ROS_INFO("scene_img.rows: %d", scene_img.rows);
        ROS_INFO("scene_img.cols: %d", scene_img.cols);
    }
    else
    {
        // send current scene
        cv::imshow("view", scene_img);
        cv::waitKey(10);

        // find a match and update templateID
        match_to_templates(boxes);
    }

    if (templateID != TEMPLATE::BLANK || templateID != TEMPLATE::UNINITIALIZED)
    {
        cv::imshow("view", boxes.templates[templateID-1]);
        cv::waitKey(2000); // display detected template for 2 seconds
    }

    return templateID;
}

void ImagePipeline::match_to_templates(const Boxes& boxes)
{
    // detect scene features
    ImageFeatures scene_features;
    detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

    // match against box object/template features
    int rematches_reqd = 0, box_idx = 0;
    std::vector<int> num_matches(box_features.size(), 0);

    for (const auto& template_features : box_features)
        num_matches[(box_idx++)] = match_to_template(template_features, scene_features);

    // get iterator to the maximum in num_matches
    auto max_match = std::max_element(num_matches.begin(), num_matches.end());

    // while best match is less than rematch thresh, run rematching
    while (*max_match < REMATCH_THRESH && rematches_reqd < NUM_REMATCH)
    {
        ROS_INFO("[IMG_PIPE] Running a rematch");
        ros::spinOnce();
        detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

        // rematch
        box_idx = 0;
        for (const auto& template_features : box_features)
            num_matches[(box_idx++)] = match_to_template(template_features, scene_features);

        max_match = std::max_element(num_matches.begin(), num_matches.end());
        rematches_reqd++;
    }

    // if at the end of rematching, best match was less than low match thresh, then return blank
    if (*max_match < REMATCH_THRESH)
        templateID = TEMPLATE::BLANK;
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        templateID = TEMPLATE(idx + 1);
    }
}

int ImagePipeline::match_to_template(const ImageFeatures& template_features, const ImageFeatures& scene_features)
{
    // feature matching
    std::vector<cv::DMatch> matches;
    matcher.match(template_features.descriptors, scene_features.descriptors, matches);

    // count "good" matches
    int num_good_matches = 0;
    for (const auto& match : matches)
    {
        if (match.distance < GOOD_MATCH_DIST)
            num_good_matches++;
    }

    return num_good_matches;
}