#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "common/Common.hpp"
//#include "configs.h"

using ImageReq = msr::airlib::ImageCaptureBase::ImageRequest;
using ImageRes = msr::airlib::ImageCaptureBase::ImageResponse;
using ImageTyp = msr::airlib::ImageCaptureBase::ImageType;

// Control functions

struct image_response_decoded {
	cv::Mat left;
	cv::Mat right;
	
	cv::Mat depth_front;
	cv::Mat depth_back;
    cv::Mat planar_depth;
	cv::Mat disparity;
	
	geometry_msgs::Pose pose;
	geometry_msgs::Pose pose_gt; //ground truth
    geometry_msgs::Twist twist;	
    bool valid_data = true;
    uint64_t poll_time;
	uint64_t timestamp;
};

struct image_response{
   std::vector<ImageRes> image;

   msr::airlib::Vector3r p;
   msr::airlib::Quaternionr q;

   uint64_t timestamp;
   uint64_t poll_time;
};


class input_sampler {
public:
	input_sampler();
	input_sampler(const std::string& ip_addr, uint16_t port);
	input_sampler(const std::string& ip_addr, uint16_t port, std::string localization_method);
	~input_sampler();

	// *** F:DN Control functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Odometry functions
    geometry_msgs::Twist twist();

	// *** F:DN Camera functions
	// cv::Mat poll_frame();
	// cv::Mat poll_frame_depth();
	
    void do_nothing();
    void poll_frame(bool);
    struct image_response_decoded poll_frame_and_decode();
    struct image_response_decoded image_decode(bool);

private:
     std::string localization_method;	
    msr::airlib::MultirotorRpcLibClient * client;
    std::string ip_addr;
    uint16_t port;

    uint64_t timestamp_offset;
    bool first_image = true;
};

#endif

