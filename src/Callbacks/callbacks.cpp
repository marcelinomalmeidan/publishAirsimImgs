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
  	// transformCamera.setOrigin(tf::Vector3(0.46,0.0,0.0));

 //  	Eigen::Matrix3d R;
 //  	R <<  0.0,  0.0, 1.0,
 //  	     -1.0,  0.0, 0.0,
 //  	      0.0, -1.0, 0.0; 
	// geometry_msgs::Quaternion q = rot2quat(R);
 //  	transformCamera.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
  	// transformCamera.setRotation(tf::Quaternion(0.0,sin(3.14/4.0),0.0,cos(3.14/4.0)));


 	br.sendTransform(tf::StampedTransform(transformQuad, ros::Time::now(), "fcu", "quad"));
}
