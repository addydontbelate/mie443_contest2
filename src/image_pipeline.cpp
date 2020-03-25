#include "image_pipeline.h"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect
// #define IMAGE_TOPIC "camera/image" // webcam

ImagePipeline::ImagePipeline(ros::NodeHandle& n, const Boxes& boxes)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::image_callback, this);
    is_valid = false;
    templateID = TEMPLATE::UNINITIALIZED;
    flann_detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    logger.open(VIS_LOG_FILEPATH);
    logger.write("\n\n************ NEW RUN *************");
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
        // cv::imshow("view", scene_img);
        // cv::waitKey(200); // show for some time

        // find a match and update templateID based on majority (if it exists) for robustness
        std::vector<TEMPLATE> matched_templates;
        matched_templates.push_back(match_to_templates_flann_dist(boxes));
        matched_templates.push_back(match_to_templates_flann_knn(boxes));
        matched_templates.push_back(match_to_templates_homog(boxes));
        templateID = get_majority_template(matched_templates);
        logger.write("\n");
    }

    if (templateID != TEMPLATE::BLANK && templateID != TEMPLATE::UNINITIALIZED)
    {
        // cv::imshow("view", boxes.templates[templateID-1]);
        // cv::waitKey(200); // display detected template for some time
    }
    else if (templateID == TEMPLATE::BLANK)
    {
        // cv::imshow("view",cv::Mat(400, 400, CV_8UC3, cv::Scalar(255, 255, 255)));
        // cv::waitKey(200); // display white blank image for some time
    }
    else
        ROS_WARN("[IMG_PIPE] Could not assign template ID; sending uninitialized ID!");

    return templateID;
}

TEMPLATE ImagePipeline::match_to_templates_flann_dist(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();
    TEMPLATE template_ID = TEMPLATE::UNINITIALIZED;

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
        template_ID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        template_ID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[FLANN DIST THRESH] Detetcted <" + TEMPLATE_NAME[template_ID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches");
    
    return template_ID;
}

TEMPLATE ImagePipeline::match_to_templates_flann_knn(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();
    TEMPLATE template_ID = TEMPLATE::UNINITIALIZED;

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
        template_ID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        template_ID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[FLANN KNN RATIO] Detetcted <" + TEMPLATE_NAME[template_ID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches");
    
    return template_ID;
}

TEMPLATE ImagePipeline::match_to_templates_homog(const Boxes& boxes)
{
    uint64_t time_elapsed = 0.0;
    TIME start = CLOCK::now();
    TEMPLATE template_ID = TEMPLATE::UNINITIALIZED;

    ImageFeatures scene_features;
    flann_detector->detectAndCompute(scene_img, cv::Mat(), scene_features.keypoints, scene_features.descriptors);

    // match against box object/template features
    int rematch_tries = 0, box_idx = 0;
    std::vector<int> num_matches(box_features.size(), 0);

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
        template_ID = TEMPLATE::BLANK;
    }
    else
    {
        // template ID corresponds to idx of (maximum + 1) since BLANK is at 0 in TEMPLATE
        int idx = max_match - num_matches.begin();
        template_ID = TEMPLATE(idx + 1);
    }

    // print to log
    time_elapsed = TIME_US(CLOCK::now()-start).count();
    logger.write("[HOMOG] Detetcted <" + TEMPLATE_NAME[template_ID] + "> in <" + std::to_string(time_elapsed) + 
        " us> with <" + std::to_string(*max_match) + "> matches");
    
    return template_ID;
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
    for (const auto& knn_match : knn_matches)
    {
        if (knn_match[0].distance < RATIO_THRESH*knn_match[1].distance)
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

    // calculate max and min distances between keypoints
    for (int i = 0; i < template_features.descriptors.rows; i++)
    { 
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    // get "good" candidate matches (i.e. distance is less than 3*min_dist)
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < template_features.descriptors.rows; i++)
    { 
        if (matches[i].distance < 3*min_dist)
            good_matches.push_back( matches[i]);
    }

    // having completed the matches, we will attempt to fit a
    // bounding box over the identified object in the scene image.

    // localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (const auto& match : good_matches)
    {
        // get the keypoints from the good matches
        obj.push_back(template_features.keypoints[match.queryIdx].pt);
        scene.push_back(scene_features.keypoints[match.trainIdx].pt);
    }

    // compute homography from scene to object
    cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);
    if (H.rows == 0 && H.cols == 0)
        return 0;

    // get corners from the template to be "detected"
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point(0,0); 
    obj_corners[1] = cv::Point(template_img.cols, 0);
    obj_corners[2] = cv::Point(template_img.cols, template_img.rows);
    obj_corners[3] = cv::Point(0, template_img.rows);
    std::vector<cv::Point2f> scene_corners(4);

    // define scene_corners using the homography        
    cv::perspectiveTransform(obj_corners, scene_corners, H);

    // define a contour using the scene_corners
    std::vector<cv::Point2f> contour;
    for (int i = 0; i < 4; i++)
	    contour.push_back(scene_corners[i] + cv::Point2f(template_img.cols, 0));

    // check if area is permissiable for a "good" match
    // i.e., disregard if area is larger than template or very small
    double area = cv::contourArea(contour);
    if (area >  template_img.cols*template_img.rows || area < MIN_AREA)
        return 0;
    
    // check if the good match is inside the contour to compute the "valid" match candidates
    int num_best_matches = 0;
    for (const auto& match : good_matches)
    {
        cv::Point2f matched_point = scene_features.keypoints[match.trainIdx].pt + cv::Point2f(template_img.cols, 0);
        if (cv::pointPolygonTest(contour, matched_point, false) > 0)
            num_best_matches++;
    }

    return num_best_matches;
}

TEMPLATE ImagePipeline::get_majority_template(const std::vector<TEMPLATE>& matched_templates)
{
    // find a majority in the results using Moore’s Voting algorithm
    int maj_idx = 0;
    for (int count = 1, i = 1; i < matched_templates.size(); i++) 
    { 
        if (matched_templates[maj_idx] == matched_templates[i]) 
            count++; 
        else
            count--; 
        
        if (count == 0) 
        { 
            maj_idx = i; 
            count = 1; 
        } 
    }

    // make sure the frequency of the majority element is > n/2
    int freq = 0; 
    for (const auto& tmp : matched_templates) 
      if (tmp == matched_templates[maj_idx]) 
        freq++; 
          
    // if we dont have a majority, we send uninitialized
    if (freq > matched_templates.size()/2) 
        return matched_templates[maj_idx]; 
    else
        return TEMPLATE::UNINITIALIZED;
}