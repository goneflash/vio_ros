#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf {typedef btMatrix3x3 Matrix3x3;}
#endif

void imuMsgCallback(const sensor_msgs::Imu& imu_msg);
 
int main(int argc, char **argv) {
	ros::init(argc, argv, "trans_transit");

	ros::NodeHandle n;

	n.param("base_stabilized_frame", p_base_stabilized_frame_, std::string("base_stabilized_frame"));
	n.param("base_frame", p_base_frame_, std::string("base_frame"));

	tfB_ = new tf::TransformBroadcaster();
        transform_.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
//	transform_.getOrigin().setX(0.0);
//	transform_.getOrigin().setY(0.0);
//	transform_.getOrigin().setZ(0.0);
	transform_.frame_id_ = "/base_stabilized";//p_base_stabilized_frame_;//;
	transform_.child_frame_id_ = "/map";//p_base_frame_;//
	ros::Subscriber imu_subscriber = n.subscribe("/phone1/android/imu", 5,
			imuMsgCallback);

	ros::spin();

	delete tfB_;

	return 0;
}

void imuMsgCallback(const sensor_msgs::Imu& imu_msg) {
/*
	tfScalar halfYaw = tfScalar(imu_msg.orientation.x / 180 * 3.14159) * tfScalar(0.5);
	tfScalar halfPitch = tfScalar(imu_msg.orientation.z / 180 * 3.14159) * tfScalar(0.5);
	tfScalar halfRoll = tfScalar(imu_msg.orientation.y / 180 * 3.14159) * tfScalar(0.5);
	tfScalar cosYaw = tfCos(halfYaw);
	tfScalar sinYaw = tfSin(halfYaw);
	tfScalar cosPitch = tfCos(halfPitch);
	tfScalar sinPitch = tfSin(halfPitch);
	tfScalar cosRoll = tfCos(halfRoll);
	tfScalar sinRoll = tfSin(halfRoll);

	sensor_msgs::Imu new_imu_msg;
	new_imu_msg.orientation.x = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	new_imu_msg.orientation.y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	new_imu_msg.orientation.z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	new_imu_msg.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
*/
	tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

	tfScalar yaw, pitch, roll;
	tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

//	roll = -(180 - imu_msg.orientation.x) / 180.0 * 3.1415926;
//	pitch = (180 - imu_msg.orientation.y) / 180.0 * 3.1415926;
	tmp_.setRPY(roll, pitch, yaw);

	transform_.setRotation(tmp_);

	transform_.stamp_ = ros::Time::now();//imu_msg.header.stamp;

	ROS_INFO("Published roll: %lf, pitch: %lf.", roll, pitch);
	tfB_->sendTransform(transform_);
}


