#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <Eigen/Dense>

//Function to set quaternion values
geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw);

//Function to convert a rotation matrix to quaternion
geometry_msgs::Quaternion rot2quat(Eigen::Matrix3d R);
