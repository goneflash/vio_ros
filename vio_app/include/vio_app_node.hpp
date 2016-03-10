#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ShuminSLAM/feature_tracker.hpp"
#include "ShuminSLAM/keyframe_selector.hpp"
#include "ShuminSLAM/landmark_server.hpp"
#include "ShuminSLAM/pose_optimizer.hpp"

#include <string>

using namespace std;

class VIOAppNode {
 public:
  explicit VIOAppNode(const string &input_image_topic);
  ~VIOAppNode();

 private:
  void NewImageCallback(const sensor_msgs::ImageConstPtr &msg);

  void AddNewFrame(const cv::Mat &img);
  void TriangulateFeatures();
  void DrawFeatureTracks(const cv::Mat &img, const vector<cv::KeyPoint> &kp,
                         const vector<cv::KeyPoint> &last_kp,
                         const vector<cv::DMatch> &matches);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  bool inited_;
  FeatureTracker feature_tracker_;
  cv::Mat last_keyframe_desc_;
  vector<cv::KeyPoint> last_keyframe_kp_;
  string feature_track_window_name_;

  bool use_keyframe_;
  int keyframe_count_;
  KeyframeSelector keyframe_selector_;

  LandmarkServer landmark_server_;

  PoseOptimizer pose_optimizer_;
};
