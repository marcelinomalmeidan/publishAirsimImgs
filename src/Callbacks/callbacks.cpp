#include "callbacks.h"


void tfCallback(const nav_msgs::Odometry::ConstPtr &msg){
	// std::cout << "debug" << std::endl;
  static tf::TransformBroadcaster br;
	tf::Transform transformQuad;
	transformQuad.setOrigin(tf::Vector3(msg->pose.pose.position.x,
		                            msg->pose.pose.position.y,
		                            msg->pose.pose.position.z) );
  	transformQuad.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
  		                                 msg->pose.pose.orientation.y,
  		                                 msg->pose.pose.orientation.z,
  		                                 msg->pose.pose.orientation.w));

 	br.sendTransform(tf::StampedTransform(transformQuad, ros::Time::now(), "world", "quad"));
}
