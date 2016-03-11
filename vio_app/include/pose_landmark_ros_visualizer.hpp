#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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

    /*
        sensor_msgs::PointCloud2::Ptr msg (new sensor_msgs::PointCloud2);

        msg->header.frame_id = "landmark_frame";
        msg->height = msg->width = 1;

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new
       pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


        msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
    */
    return true;
  }

 private:
  ros::NodeHandle *nh_;
  ros::Publisher landmark_pub_;
  ros::Publisher pose_track_pub_;

  pcl::PointCloud<pcl::PointXYZ> landmarks_;
  // pcl::PCLPointCloud2 landmarks_;
};
