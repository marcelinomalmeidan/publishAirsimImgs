#include "input_sampler.h"
#include <iostream>
#include <vector>
#include <cstring>
#include "common/Common.hpp"

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

/*
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
*/

static void convertToPlanDepth(const cv::Mat& input, cv::Mat& output, float f = 320)
{
	int width = input.cols;
	int height = input.rows;

	float center_i = width / 2.0f - 1;
	float center_j = height / 2.0f - 1;

	output = cv::Mat(height, width, CV_32FC1);

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			float dist = std::sqrt((i - center_i)*(i - center_i) + (j - center_j)*(j - center_j));
			float denom = (dist / f);
			denom *= denom;
			denom = std::sqrt(1 + denom);
			output.at<float>(j, i) = input.at<float>(j, i) / denom;
		}
	}
}


static void convertToDisparity(const cv::Mat& input, cv::Mat& output, float f = 320, float baseline_meters = 1)
{
	int width = input.cols;
	int height = input.rows;
	int size = width * height;

	output = cv::Mat(height, width, CV_32FC1);

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			output.at<float>(j, i) = f * baseline_meters * (1.0f / input.at<float>(j, i));
		}
	}
}

struct image_response input_sampler::poll_frame()
{
	using ImageRequest = msr::airlib::DroneControllerBase::ImageRequest;
	using ImageResponse = msr::airlib::VehicleCameraBase::ImageResponse;
	using ImageType_ = msr::airlib::VehicleCameraBase::ImageType_;

	struct image_response result;
	const int max_tries = 1000000;

	std::vector<ImageRequest> request = {
		ImageRequest(0, ImageType_::Scene),
		ImageRequest(1, ImageType_::Scene),
		ImageRequest(1, ImageType_::Depth)
	};

	std::vector<ImageResponse> response = client->simGetImages(request);

	int i;
	for (i = 0; response.size() != request.size() && i < max_tries; i++) {
		response = client->simGetImages(request);
	}

	if (response.size() == request.size()) {
#if CV_MAJOR_VERSION==3
		result.left = cv::imdecode(response.at(0).image_data, cv::IMREAD_COLOR);
		result.right = cv::imdecode(response.at(1).image_data, cv::IMREAD_COLOR);
		result.depth = cv::imdecode(response.at(2).image_data, cv::IMREAD_GRAYSCALE);
#else
		result.left = cv::imdecode(response.at(0).image_data, CV_LOAD_IMAGE_COLOR);
		result.right = cv::imdecode(response.at(1).image_data, CV_LOAD_IMAGE_COLOR);
		result.depth = cv::imdecode(response.at(2).image_data, CV_LOAD_IMAGE_GRAYSCALE);
#endif
		cv::Mat depth_float;

		result.depth.convertTo(depth_float, CV_32FC1);

		convertToPlanDepth(depth_float, result.planar_depth);

		float f = depth_float.cols / 2.0;
		convertToDisparity(result.planar_depth, result.disparity, f, 25 / 100.0f);

		result.depth = depth_float;
	} else {
		std::cerr << "Images not returned successfully" << std::endl;
	}

	return result;
}
