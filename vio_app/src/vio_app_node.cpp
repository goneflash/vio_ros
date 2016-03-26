#include "vio_app_node.hpp"

VIOAppNode::VIOAppNode(const string &input_image_topic)
    : it_(nh_),
      inited_(false),
      use_keyframe_(true),
      keyframe_count_(0),
      visualize_3d_(true) {
  visualizer_ = new PoseLandmarkROSVisualizer(&nh_);

  image_sub_ =
      it_.subscribe(input_image_topic, 5, &VIOAppNode::NewImageCallback, this);
  feature_track_window_name_ = "Feature_tracks";
  cv::namedWindow(feature_track_window_name_.c_str());
  ROS_INFO("VIO App Node Initialized.");
}

VIOAppNode::~VIOAppNode() { cv::destroyWindow(feature_track_window_name_); }

void VIOAppNode::NewImageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  namespace enc = sensor_msgs::image_encodings;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600) {
    if (!inited_) {
      feature_tracker_.DetectFeatureInFirstFrame(
          cv_ptr->image, last_keyframe_kp_, last_keyframe_desc_);

      landmark_server_.AddFirstFrameFeature(last_keyframe_kp_);
      keyframe_count_++;
      inited_ = true;
    } else {
      AddNewFrame(cv_ptr->image);
    }
  }
}

void VIOAppNode::AddNewFrame(const cv::Mat &img) {
  vector<cv::KeyPoint> kp;
  cv::Mat desc;
  vector<cv::DMatch> matches;
  feature_tracker_.TrackFeature(last_keyframe_kp_, last_keyframe_desc_, img, kp, desc, matches);

  if (use_keyframe_) {
    if (keyframe_selector_.isKeyframe(img, kp, matches)) {
      landmark_server_.AddNewFeatureAssociationToLastFrame(kp, matches);
      keyframe_count_++;
      ROS_INFO("Added a keyframe.");

      // landmark_server_.PrintStats();

      desc.copyTo(last_keyframe_desc_);
      last_keyframe_kp_.swap(kp);

      // Should use a new thread to do this.
      if (keyframe_count_ == 10) TriangulateFeatures();

    } else {
      ROS_INFO("Dropped a new frame.");
      // TODO: Should do pnp here.
      DrawFeatureTracks(img, kp, last_keyframe_kp_, matches);
    }
  }
}

void VIOAppNode::TriangulateFeatures() {
  int num_frame = landmark_server_.num_frame();
  vector<vector<cv::Vec2d> > feature_vectors(num_frame);
  cv::Matx33d K_initial, K_final;
  vector<cv::Mat> points3d;
  vector<cv::Mat> R_ests, t_ests;

  K_initial = cv::Matx33d(1914, 0, 640, 0, 1914, 360, 0, 0, 1);

  // output: points3d is vector< Mat(height = 3, width = 1) >
  //         type : CV_32F
  landmark_server_.MakeFeatureVectorsForReconstruct(feature_vectors);
  pose_optimizer_.initialize3DPointsFromViews(
      num_frame, feature_vectors, K_initial, points3d, R_ests, t_ests, K_final);

  if (visualize_3d_)
    visualizer_->AddPointsWithCameras(points3d, R_ests, t_ests);
}

void VIOAppNode::DrawFeatureTracks(const cv::Mat &img,
                                   const vector<cv::KeyPoint> &kp,
                                   const vector<cv::KeyPoint> &last_kp,
                                   const vector<cv::DMatch> &matches) {
  cv::Mat output_img;
  vector<cv::KeyPoint> kp_draw;
  for (int i = 0; i < matches.size(); ++i)
    kp_draw.push_back(kp[matches[i].trainIdx]);

  // Only draw matched keypoints
  drawKeypoints(img, kp_draw, output_img, cv::Scalar(255, 0, 0));

  for (int i = 0; i < matches.size(); ++i) {
    line(output_img, kp[matches[i].trainIdx].pt,
         last_kp[matches[i].queryIdx].pt, cv::Scalar(255, 0, 0));
  }

  cv::imshow(feature_track_window_name_, output_img);
  cv::waitKey(3);
}
