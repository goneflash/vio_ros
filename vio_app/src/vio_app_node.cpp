#include "include/VIOAppNode.hpp"

VIOAppNode::VIOAppNode()
   : it_(nh_), inited_(false) {
    image_sub_ = it_.subscribe(input_image_topic_name, 5, 
      &VIOAppNode::NewImageCallback, this);
    cv::namedWindow(OPENCV_WINDOW);
    ROS_INFO("VIO App Node Initialized.");
}

VIOAppNode:: ~VIOAppNode() {
  cv::destroyWindow(OPENCV_WINDOW);
}

void VIOAppNode::NewImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){
        if (!inited_) {
          feature_tracker_.DetectFeatureInFirstFrame(cv_ptr->image,
                                            last_keyframe_kp_, last_keyframe_desc_);
          inited_ = true;
        } else {
          addNewFrame(cv_ptr->image);
	  trackFeatures(cv_ptr->image);
        }
    }
}

void VIOAppNode::addNewFrame(cv::Mat img) {
  vector<cv::KeyPoint> kp;
  cv::Mat desc;
  vector<cv::DMatch> matches;
  feature_tracker_.TrackFeature(last_keyframe_desc_, img, kp, desc, matches);

  
}

void VIOAppNode::trackFeatures(cv::Mat img) {
    vector<cv::KeyPoint> kp;
    cv::Mat desc;
    vector<cv::DMatch> matches;

    feature_tracker_.TrackFeature(last_keyframe_desc_, img, kp, desc, matches);

    cv::Mat output_img;
    vector<cv::KeyPoint> kp_draw;
    for (int i = 0; i < matches.size(); ++i)
      kp_draw.push_back(kp[matches[i].trainIdx]);

    // Only draw matched keypoints
    drawKeypoints(output_img, kp_draw, output_img, cv::Scalar(255, 0, 0));

    for (int i = 0; i < matches.size(); ++i) {
      line(output_img, kp[matches[i].trainIdx].pt, last_keyframe_kp_[matches[i].queryIdx].pt,
           cv::Scalar(255, 0, 0));
    }

    desc.copyTo(last_keyframe_desc_);
    last_keyframe_kp_.swap(kp);

    cv::imshow(OPENCV_WINDOW, output_img);
    cv::waitKey(3);
  }


