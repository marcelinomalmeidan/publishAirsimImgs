#include "input_sampler.h"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include "common/Common.hpp"
//#include "log__class.h"
#include <fstream>
using namespace std;
//extern log__class log__f;
input_sampler::input_sampler() : client(0)
{
	connect();
}

input_sampler::input_sampler(const std::string& ip_addr, uint16_t port) : client(0)
{
	connect(ip_addr, port);
}

input_sampler::~input_sampler()
{
	if (client != 0)
		delete client;
}

void input_sampler::connect()
{
	if (client != 0)
		delete client;
	client = new msr::airlib::RpcLibClient();
}

void input_sampler::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::RpcLibClient(ip_addr, port);
}


cv::Mat input_sampler:: poll_frame()
{
    cv::Mat result;
	cv::Mat scaled;
	const int max_tries = 1000000;
	std::vector<uint8_t> png;

	try {
		if (cam_type != msr::airlib::VehicleCamera::ImageType::Scene) {
            cam_type = msr::airlib::VehicleCamera::ImageType::Scene;
			//client->setImageTypeForCamera(0, cam_type);
		}

        for (int i = 0; png.size() <= 1 && i < max_tries; i++) {
            png = client->getImageForCamera(0, msr::airlib::VehicleCamera::ImageType::Scene);
		}
	} catch (...) {
		std::cerr << "poll_frame failed" << std::endl;
	}

	if (png.size() > 1) {
#if CV_MAJOR_VERSION==3
		result = cv::imdecode(png, cv::IMREAD_COLOR);
#else
		result = cv::imdecode(png, CV_LOAD_IMAGE_COLOR);
#endif
	} else {
		std::cerr << "read_frame: Empty image returned" << std::endl;
	}

	//cv::resize(result, scaled, cv::Size(427,240));
	//return scaled;
	return result;
}

cv::Mat input_sampler::poll_frame_depth()
{
	cv::Mat resultBGR, resultGray;
	cv::Mat scaled;
	const int max_tries = 1000000;
	std::vector<uint8_t> png;

	try {
		if (cam_type != msr::airlib::VehicleCamera::ImageType::Depth) {
			cam_type = msr::airlib::VehicleCamera::ImageType::Depth;
			//client->setImageTypeForCamera(0, cam_type);
		}

		for (int i = 0; png.size() <= 1 && i < max_tries; i++) {
			png = client->getImageForCamera(0, msr::airlib::VehicleCamera::ImageType::Depth);
		}
	} catch (...) {
		std::cerr << "read_frame_depth failed" << std::endl;
	}

	if (png.size() > 1) {
		#if CV_MAJOR_VERSION==3
			resultGray = cv::imdecode(png, cv::IMREAD_GRAYSCALE);
			// resultBGR = cv::imdecode(png, cv::IMREAD_UNCHANGED);
			// cv::cvtColor(resultBGR,resultGray, cv::COLOR_BGR2GRAY);
		#else
			resultGray = cv::imdecode(png, CV_LOAD_IMAGE_GRAYSCALE);
		#endif
	} else {
		std::cerr << "read_frame: Empty image returned" << std::endl;
	}

	//cv::resize(result, scaled, cv::Size(427,240));
	//return scaled;
	return resultGray;
}
