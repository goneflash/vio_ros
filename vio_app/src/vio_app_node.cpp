#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

#include "ShuminSLAM/feature_tracker.hpp"

using namespace std;

static const std::string OPENCV_WINDOW = "Raw Image window";

class FeatureTrackerNode {
public:
  FeatureTrackerNode()
    : it_(nh_), inited_(false) {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 5, 
      &FeatureTrackerNode::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
    ROS_INFO("Tracker Node Initialized.");
  }

  ~FeatureTrackerNode() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
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
          feature_tracker_.DetectFeatureInFirstFrame(cv_ptr->image, last_kp_, last_desc_);
          inited_ = true;
        } else {
	  trackFeatures(cv_ptr->image);
        }
    }
  }
  void trackFeatures(cv::Mat img) {
    vector<cv::KeyPoint> kp;
    cv::Mat desc;
    vector<cv::DMatch> matches;

    feature_tracker_.TrackFeature(last_desc_, img, kp, desc, matches);

    cv::Mat output_img;
    vector<cv::KeyPoint> kp_draw;
    for (int i = 0; i < matches.size(); ++i)
      kp_draw.push_back(kp[matches[i].trainIdx]);

    // Only draw matched keypoints
    drawKeypoints(output_img, kp_draw, output_img, cv::Scalar(255, 0, 0));

    for (int i = 0; i < matches.size(); ++i) {
      line(output_img, kp[matches[i].trainIdx].pt, last_kp_[matches[i].queryIdx].pt,
           cv::Scalar(255, 0, 0));
    }

    desc.copyTo(last_desc_);
    last_kp_.swap(kp);

    cv::imshow(OPENCV_WINDOW, output_img);
    cv::waitKey(3);
  }

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  bool inited_;
  FeatureTracker feature_tracker_;
  cv::Mat last_desc_;
  vector<cv::KeyPoint> last_kp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Feature_Tracker");
  FeatureTrackerNode ic;
  ros::spin();
  return 0;
}
