#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ShuminSLAM/feature_tracker.hpp"
#include "ShuminSLAM/keyframe_selector.hpp"

#include <string>

using namespace std;

class VIOAppNode {
public:
  explicit VIOAppNode(string input_image_topic_name);
  ~VIOAppNode();

  void NewImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void addNewFrame(cv::Mat img);
  void trackFeatures(cv::Mat img);

 private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  bool inited_;
  FeatureTracker feature_tracker_;
  cv::Mat last_keyframe_desc_;
  vector<cv::KeyPoint> last_keyframe_kp_;

  bool use_keyframe_;
  KeyframeSelector keyframe_selector_;
};

