#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class PoseLandmarkROSVisualizer {
 public:
  explicit PoseLandmarkROSVisualizer(ros::NodeHandle *nh) : nh_(nh) {
    landmark_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("landmarks", 1);
  };
  ~PoseLandmarkROSVisualizer(){};

  bool AddPointsWithCameras(const vector<cv::Mat> &points3d,
                            const vector<cv::Mat> &R,
                            const vector<cv::Mat> &t) {
    ROS_INFO("Added %lu 3d landmarks to visualizer.", points3d.size());
    // ROS_INFO("%d  %d  %d", points3d[0].type(), points3d[0].depth(),
    // points3d[0].channels());

    for (int i = 0; i < points3d.size(); ++i) {
      float x = points3d[i].at<float>(0);
      float y = points3d[i].at<float>(1);
      float z = points3d[i].at<float>(2);
      // ROS_INFO("Added point (%f, %f, %f).", x, y, z);
      landmarks_.push_back(pcl::PointXYZ(x, y, z));
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(landmarks_, pcl_pc2);

    sensor_msgs::PointCloud2 msg;
    pcl_conversions::fromPCL(pcl_pc2, msg); 
    msg.header.frame_id = "map";

    landmark_pub_.publish(msg);

    return true;
  }

 private:
  ros::NodeHandle *nh_;
  ros::Publisher landmark_pub_;
  ros::Publisher pose_track_pub_;

  pcl::PointCloud<pcl::PointXYZ> landmarks_;
  // pcl::PCLPointCloud2 landmarks_;
};
