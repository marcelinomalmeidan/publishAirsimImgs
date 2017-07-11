#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "rpc/RpcLibClient.hpp"
#include <geometry_msgs/Pose.h>

//#include "configs.h"
// Control functions

struct image_response {
	cv::Mat left;
	cv::Mat right;
	
	cv::Mat depth;
	cv::Mat planar_depth;
	cv::Mat disparity;
	
	geometry_msgs::Pose pose;
};

class input_sampler {
public:
	input_sampler();
	input_sampler(const std::string& ip_addr, uint16_t port);

	~input_sampler();

	// *** F:DN Control functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Camera functions
	// cv::Mat poll_frame();
	// cv::Mat poll_frame_depth();
	struct image_response poll_frame(float scale);

private:
	msr::airlib::RpcLibClient * client;
};

#endif
