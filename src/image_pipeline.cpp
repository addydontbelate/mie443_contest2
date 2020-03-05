#include "image_pipeline.h"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect
// #define IMAGE_TOPIC "camera/image" // webcam

ImagePipeline::ImagePipeline(ros::NodeHandle& n, const Boxes& boxes)
{h
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::image_callback, this);
    is_valid = false;
    templateID = TEMPLATE::UNINITIALIZED;
    flann_detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    logger.open(VISION_LOG_FILE);
    logger.write("\n\n************ NEW RUN *************\n");
    load_template_features(boxes);
}

void ImagePipeline::load_template_features(const Boxes& boxes)
{
    // store features of each box object/template to match against later
    for (const auto& box : boxes.templates)
    {
        ImageFeatures features;
        flann_detector->detectAndCompute(box, cv::Mat(), features.keypoints, features.descriptors);
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
        cv::waitKey(500); // show for some time

        // find a match and update templateID
        match_to_templates_flann_dist(boxes);
        match_to_templates_flann_knn(boxes);
        match_to_templates_homog(boxes);
    }

    if (templateID != TEMPLATE::BLANK && templateID != TEMPLATE::UNINITIALIZED)
    {
        cv::imshow("view", boxes.templates[templateID-1]);
        cv::waitKey(500); // display detected template for some time
    }
    else if (templateID == TEMPLATE::BLANK)
    {
        cv::imshow("view",cv::Mat(400, 400, CV_8UC3, cv::Scalar(255, 255, 255)));
        cv::waitKey(500); // display white blank image for some time
    }
    else
        ROS_WARN("[IMG_PIPE] Could not assign template ID; sending uninitialized ID!");

    return templateID;
}

void ImagePipeline::match_to_templates_flann_dist(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();

    // detect scene features
    ImageFeatures scene_features;
    flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

    // match against box object/template features
    int rematch_tries = 0, box_idx = 0;
    std::vector<int> num_matches(box_features.size(), 0);

    for (const auto& template_features : box_features)
        num_matches[(box_idx++)] = match_to_template_flann_dist(template_features, scene_features);

    // get iterator to the maximum in num_matches
    auto max_match = std::max_element(num_matches.begin(), num_matches.end());

    // while best match is less than rematch thresh, run rematching
    while (*max_match < REMATCH_THRESH && rematch_tries < NUM_REMATCH)
    {
        ROS_INFO("[IMG_PIPE] Trying to rematch");
        ros::spinOnce();
        flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);
        
        // rematch
        box_idx = 0;
        for (const auto& template_features : box_features)
            num_matches[(box_idx++)] = match_to_template_flann_dist(template_features, scene_features);
        
        max_match = std::max_element(num_matches.begin(), num_matches.end());
        rematch_tries++;
    }

    // if at the end of rematching, best match was less than low match thresh, then return blank
    if (*max_match < REMATCH_THRESH)
    {
        templateID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        templateID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[FLANN DIST THRESH] Detetcted <" + TEMPLATE_NAME[templateID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches\n");
}

void ImagePipeline::match_to_templates_flann_knn(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();

    ImageFeatures scene_features;
    flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

    // match against box object/template features
    int rematch_tries = 0, box_idx = 0;
    std::vector<int> num_matches(box_features.size(), 0);

    for (const auto& template_features : box_features)
        num_matches[(box_idx++)] = match_to_template_flann_knn(template_features, scene_features);

    // get iterator to the maximum in num_matches
    auto max_match = std::max_element(num_matches.begin(), num_matches.end());

    // while best match is less than rematch thresh, run rematching
    while (*max_match < REMATCH_THRESH && rematch_tries < NUM_REMATCH)
    {
        ROS_INFO("[IMG_PIPE] Trying to rematch");
        ros::spinOnce();
        flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);
        
        // rematch
        box_idx = 0;
        for (const auto& template_features : box_features)
            num_matches[(box_idx++)] = match_to_template_flann_knn(template_features, scene_features);
        
        max_match = std::max_element(num_matches.begin(), num_matches.end());
        rematch_tries++;
    }

    // if at the end of rematching, best match was less than low match thresh, then return blank
    if (*max_match < REMATCH_THRESH)
    {
        templateID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        templateID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[FLANN KNN RATIO] Detetcted <" + TEMPLATE_NAME[templateID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches\n");
}

void ImagePipeline::match_to_templates_homog(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();

    ImageFeatures scene_features;
    flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

    // match against box object/template features
    int rematch_tries = 0, box_idx = 0;
    std::vector<double> num_matches(box_features.size(), 0);

    for (const auto& template_features : box_features)
    {
        num_matches[box_idx] = match_to_template_homog(template_features, boxes.templates[box_idx], scene_features);
        box_idx++;
    }

    // get iterator to the maximum in num_matches
    auto max_match = std::max_element(num_matches.begin(), num_matches.end());

    // while best match is less than rematch thresh, run rematching
    while (*max_match < REMATCH_THRESH && rematch_tries < NUM_REMATCH)
    {
        ROS_INFO("[IMG_PIPE] Trying to rematch");
        ros::spinOnce();
        flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);
        
        // rematch
        box_idx = 0;
        for (const auto& template_features : box_features)
        {
            num_matches[box_idx] = match_to_template_homog(template_features, boxes.templates[box_idx], scene_features);
            box_idx++;
        }

        max_match = std::max_element(num_matches.begin(), num_matches.end());
        rematch_tries++;
    }

    // if at the end of rematching, best match was less than low match thresh, then return blank
    if (*max_match < REMATCH_THRESH)
    {
        templateID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        templateID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[HOMOG] Detetcted <" + TEMPLATE_NAME[templateID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches\n");
}

int ImagePipeline::match_to_template_flann_dist(const ImageFeatures& template_features, const ImageFeatures& scene_features)
{
    // feature matching
    std::vector<cv::DMatch> matches;
    flann_matcher.match(template_features.descriptors, scene_features.descriptors, matches);

    // count "good" matches using hamming distance threshold
    int num_good_matches = 0;
    for (const auto& match : matches)
    {
        if (match.distance < GOOD_MATCH_DIST)
            num_good_matches++;
    }

    return num_good_matches;
}

int ImagePipeline::match_to_template_flann_knn(const ImageFeatures& template_features, const ImageFeatures& scene_features)
{
    // feature matching
    std::vector<std::vector<cv::DMatch> > knn_matches;
    flann_matcher.knnMatch(template_features.descriptors, scene_features.descriptors, knn_matches, 2);

    // apply ratio test (D. Lowe's Paper)
    int num_good_matches = 0;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < RATIO_THRESH*knn_matches[i][1].distance)
            num_good_matches++;
    }

    return num_good_matches;
}

int ImagePipeline::match_to_template_homog(const ImageFeatures& template_features, const cv::Mat& template_img,
        const ImageFeatures& scene_features)
{
    // feature matching
    std::vector<cv::DMatch> matches;
    flann_matcher.match(template_features.descriptors, scene_features.descriptors, matches);

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < template_features.descriptors.rows; i++)
    { 
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<cv::DMatch> good_matches;

    for (int i = 0; i < template_features.descriptors.rows; i++)
    { 
        if (matches[i].distance < 3*min_dist)
            good_matches.push_back( matches[i]);
    }

    /***
     * We have completed the matches. In the next section of code we will attempt to fit a
     * bounding box over the identified object in the scene image.
     */

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (auto & match : good_matches)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(template_features.keypoints[match.queryIdx].pt);
        scene.push_back(scene_features.keypoints[match.trainIdx].pt);
    }

    cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point(0,0); 
    obj_corners[1] = cv::Point(template_img.cols, 0);
    obj_corners[2] = cv::Point(template_img.cols, template_img.rows);
    obj_corners[3] = cv::Point(0, template_img.rows);
    std::vector<cv::Point2f> scene_corners(4);

    // Define scene_corners using Homography
    cv::perspectiveTransform(obj_corners, scene_corners, H);

    // Define a contour using the scene_corners
    std::vector<cv::Point2f> contour;
    for (int i = 0; i < 4; i++)
	    contour.push_back(scene_corners[i] + cv::Point2f(template_img.cols, 0));
    
    
    double area = cv::contourArea(contour);
    if (area >  template_img.cols*template_img.rows || area < MIN_AREA) // larger than template or very small, disregard.
        return 0;

    int num_best_matches = 0;

    // Check if the good match is inside the contour.
    for (auto & match : good_matches)
    {
        cv::Point2f matched_point = scene_features.keypoints[match.trainIdx].pt + cv::Point2f(template_img.cols, 0);
        if (cv::pointPolygonTest(contour, matched_point, false) > 0)
            num_best_matches++;
    }

    /***
     * In this section of the code we use a chosen heuristic to decided how good the match
     * is the to given template.
     * One such heuristic is the absolute number of good_matches found.
     */
    return num_best_matches;
}