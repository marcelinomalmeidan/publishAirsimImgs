#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <list>
#include <opencv2/opencv.hpp>
#include "rpc/RpcLibClient.hpp"
#include "coord.h"
#include "VehicleCamera.hpp"

//#include "configs.h"
// Control functions
class input_sampler {
public:
	input_sampler();
	input_sampler(const std::string& ip_addr, uint16_t port);

	~input_sampler();

	// *** F:DN Control functions
	void connect();
	void connect(const std::string& ip_addr, uint16_t port);

	// *** F:DN Camera functions
	cv::Mat poll_frame();
	cv::Mat poll_frame_depth();


private:
	//void buffer_images();

	msr::airlib::RpcLibClient * client;
	msr::airlib::VehicleCamera::ImageType cam_type;
	std::list<cv::Mat> cam_buf;
	//std::mutex cam_buf_mutex;
	//std::thread cam_thread;
	//std::atomic<bool> stop_buffer_flag;
};

#endif
