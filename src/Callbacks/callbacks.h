#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "HelperFunctions/QuatRotEuler.h"

//Publish tf for the position/orientation of the quadcopter and camera
void tfCallback(const nav_msgs::Odometry::ConstPtr &msg);