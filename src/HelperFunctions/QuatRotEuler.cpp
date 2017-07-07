#include "QuatRotEuler.h"


geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw){
	geometry_msgs::Quaternion q;
	q.x = qx;
	q.y = qy;
	q.z = qz;
	q.w = qw;
	return q;
}


//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
geometry_msgs::Quaternion rot2quat(Eigen::Matrix3d M){
	double trace = M.trace();

	Eigen::Matrix<float, 4, 1> q;
	if (trace > 0) {// M_EPSILON = 0
		double s = 0.5 / sqrt(trace + 1.0);
		q << 0.25 / s,
			(M(2,1) - M(1,2)) * s,
			(M(0,2) - M(2,0)) * s,
			(M(1,0) - M(0,1)) * s;
	}
	else {
		if (M(0,0) > M(1,1) && M(0,0) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(0,0) - M(1,1) - M(2,2));
			q << (M(2,1) - M(1,2)) / s,
				 0.25 * s,
				 (M(0,1) + M(1,0)) / s,
				 (M(0,2) + M(2,0)) / s;
		}
		else if (M(1,1) > M(2,2)) {
			double s = 2.0 * sqrt(1.0 + M(1,1) - M(0,0) - M(2,2));
			q << (M(0,2) - M(2,0)) / s,
			     (M(0,1) + M(1,0)) / s,
			     0.25 * s,
			     (M(1,2) + M(2,1)) / s;
		}
		else {
			double s = 2.0 * sqrt(1.0 + M(2,2) - M(0,0) - M(1,1));
			q << (M(1,0) - M(0,1))/s,
			     (M(0,2) + M(2,0))/s,
			     (M(1,2) + M(2,1))/s,
			     0.25 * s;
		}
	}

	geometry_msgs::Quaternion quat;
	quat = setQuat(q[1], q[2], q[3], q[0]);
    return quat;	
}
