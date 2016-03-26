#include <ros/ros.h>

#include "vio_app_node.hpp"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "VIO_app");
  string image_topic = "/camera/rgb/image_color";
  //string image_topic = "/camera/image_raw";
  VIOAppNode vio_node(image_topic);

  ros::spin();
  return 0;
}
