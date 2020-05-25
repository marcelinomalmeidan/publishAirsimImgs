#include "input_sampler.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include "common/Common.hpp"
#include <mutex>
#include <chrono>
#include "common/VectorMath.hpp"
#include <ros/ros.h>
using namespace std::chrono;
std::mutex image_response_queue_mutex;
std::queue<struct image_response> image_response_queue;
std::mutex client_mutex;
volatile bool exit_out = false;
input_sampler::input_sampler() : client(0), localization_method("ground_truth"), ip_addr("not provided"), port(000)
{
	connect();
}

input_sampler::input_sampler(const std::string& ip_addr, uint16_t port) : client(0), localization_method("ground_truth")
{
      this->ip_addr = ip_addr;
      this->port = port;
      connect(ip_addr, port);
}


input_sampler::input_sampler(const std::string& ip_addr, uint16_t port, std::string localization_method) : client(0)
{
	connect(ip_addr, port);
      this->ip_addr = ip_addr;
      this->port = port;
      this->localization_method = localization_method;
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
	client = new msr::airlib::MultirotorRpcLibClient();
}

void input_sampler::connect(const std::string& ip_addr, uint16_t port)
{
	if (client != 0)
		delete client;
	client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
}


static void convertToPlanDepth(const cv::Mat& input, cv::Mat& output, float f = 128)
{
	int width = input.cols;
	int height = input.rows;

	float center_i = width / 2.0f - 1;
	float center_j = height / 2.0f - 1;

    // output = cv::Mat(height, width, CV_32FC1);

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


static void convertToDisparity(const cv::Mat& input, cv::Mat& output, float f = 128, float baseline_meters = 0.14)
{
	int width = input.cols;
	int height = input.rows;
	int size = width * height;

	output = cv::Mat(height, width, CV_32FC1);

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			output.at<float>(j, i) = f * baseline_meters / input.at<float>(j, i);
		}
	}
}


std::ofstream file_to_output;
void input_sampler::poll_frame(bool all_front)
{
    static uint64 last_time_stamp = 0; 
    msr::airlib::MultirotorRpcLibClient *my_client = new msr::airlib::MultirotorRpcLibClient(this->ip_addr, this->port);

    const int max_tries = 1000000;
    
    ImageTyp image_type;
    int cameraId;
    if (all_front) {
         cameraId = 0;
         image_type = ImageTyp::Scene;
    }else{
         cameraId = 4;
         image_type = ImageTyp::DepthPlanner;
    }
    std::vector<ImageReq> request = {ImageReq(cameraId, image_type),
	    ImageReq(0, ImageTyp::DepthPlanner)};

    try{ 
        
        struct image_response response;
        while(true){
            ros::Time start_hook_t = ros::Time::now();
            client_mutex.lock(); 
            if (exit_out) {
                std::cout << "killing the poll thread" << std::endl;
                client_mutex.unlock(); 
                return;
            } 
            
            response.image = my_client->simGetImages(request);
            response.p = my_client->getPosition();
            response.q = my_client->getOrientation();

            for (int i = 0; response.image.size() != request.size() && i < max_tries; i++) {
                response.image = my_client->simGetImages(request);
            }
            
            ros::Time now =  ros::Time::now();
            if (this->localization_method == "vins_mono"){//for vins mono the imgs and imu 
                                                          //need to be synchronized
                response.timestamp = response.image.at(0).time_stamp;
            }else{            //for profiling purposes 
                response.timestamp = (uint64_t) now.sec*1e9 + (uint64_t)now.nsec;
            }
            
            if (last_time_stamp >= response.timestamp) {
                ROS_ERROR_STREAM("imag time stamps shouldn't be out of order"<< last_time_stamp<< " " <<response.timestamp<< " "); 
            }
            last_time_stamp = response.timestamp;

            image_response_queue_mutex.lock(); 
            if (response.image.size() == request.size()) {
                response.poll_time = (ros::Time::now() - start_hook_t).toSec()*1e9;
                //ROS_INFO_STREAM("poll only"<<response.poll_time); 
                image_response_queue.push(response);
            } 
            image_response_queue_mutex.unlock(); 
            
            client_mutex.unlock(); 
        }
    }
    catch(...){
        printf("got here"); 
        return; 
        exit(0);
    }
}

void input_sampler::do_nothing(void) {
    while(true){
        ;
    }
}


struct image_response_decoded input_sampler::image_decode(bool all_front){
    try{ 
        image_response_queue_mutex.lock(); 
        if (image_response_queue.empty()){
            image_response_queue_mutex.unlock(); 
            struct image_response_decoded result;
            result.valid_data = false; 
            return  result;
        }

        struct image_response response = image_response_queue.back();
        std::queue<struct image_response>().swap(image_response_queue);
        image_response_queue_mutex.unlock(); 

        struct image_response_decoded result;


#if CV_MAJOR_VERSION==3
        // result.left = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.depth_front = cv::imdecode(response.image.at(1).image_data_uint8, cv::IMREAD_GRAYSCALE);

        if(all_front) {	
            result.right = cv::imdecode(response.image.at(0).image_data_uint8, cv::IMREAD_COLOR);
        }else{
            result.depth_back = cv::imdecode(response.image.at(0).image_data_uint8, cv::IMREAD_GRAYSCALE);
        }

#else
        // result.left = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.depth_front = cv::imdecode(response.image.at(1).image.image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);

        if (all_front) {	
            result.right = cv::imdecode(response.image.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        }else{
            result.depth_back = cv::imdecode(response.image.at(0).image.image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);
        }
#endif

        result.depth_front.convertTo(result.depth_front, CV_32FC1, 25.6/256);

        if (!all_front) {	
            result.depth_back.convertTo(result.depth_back, CV_32FC1, 25.6/256);
        }


        //ground truth values
        static auto initial_pos_gt= response.image.back().camera_position;
        result.pose_gt.position.x = response.image.back().camera_position.x() - initial_pos_gt.x();
        result.pose_gt.position.y = response.image.back().camera_position.y() - initial_pos_gt.y();
        result.pose_gt.position.z = response.image.back().camera_position.z() - initial_pos_gt.z();

        result.pose_gt.orientation.x = response.image.back().camera_orientation.x();
        result.pose_gt.orientation.y = response.image.back().camera_orientation.y();
        result.pose_gt.orientation.z = response.image.back().camera_orientation.z();
        result.pose_gt.orientation.w = response.image.back().camera_orientation.w();

        static msr::airlib::Vector3r initial_pos_gps = client->getPosition();

        if(this->localization_method == "gps") {
            result.pose.position.x = response.p.x() - initial_pos_gps.x();
            result.pose.position.y = response.p.y() - initial_pos_gps.y();
            result.pose.position.z = response.p.z() - initial_pos_gps.z();

            result.pose.orientation.x = response.q.x();
            result.pose.orientation.y = response.q.y();
            result.pose.orientation.z = response.q.z();
            result.pose.orientation.w = response.q.w();
        }

        result.timestamp = response.timestamp;
        result.poll_time = response.poll_time;
        return result;
    }
    catch(...){
        exit(0); 
    }
}


struct image_response_decoded input_sampler::poll_frame_and_decode()
{

    file_to_output.open("/home/nvidia/catkin_ws/src/publishAirsimImgs/src/timing.txt",
            std::ios_base::app);
    steady_clock::time_point allf_s; //total function time s
    steady_clock::time_point allf_e; //total function time e


    steady_clock::time_point partf_s; //one invocation of tracker start
    steady_clock::time_point partf_e; //one invocation of tracker end
    steady_clock::time_point partf2_s; //one invocation of tracker start
    steady_clock::time_point partf2_e; //one invocation of tracker end
    allf_s  = steady_clock::now();





    //using ImageRequest = msr::airlib::VehicleCameraBase::ImageRequest;
    //using ImageResponse = msr::airlib::VehicleCameraBase::ImageResponse;
    //using ImageType = msr::airlib::VehicleCameraBase::ImageType;

    struct image_response_decoded result;
    const int max_tries = 1000000;

    std::vector<ImageReq> request = {
        // ImageRequest(0, ImageType::Scene),
        ImageReq(1, ImageTyp::Scene),
        ImageReq(1, ImageTyp::DepthPlanner)
    };

    //result.twist = twist();

    partf_s = steady_clock::now();
    std::vector<ImageRes> response = client->simGetImages(request);

    partf_e = steady_clock::now();
    auto partf_t = duration_cast<milliseconds>(partf_e - partf_s).count();




    for (int i = 0; response.size() != request.size() && i < max_tries; i++) {
        response = client->simGetImages(request);
    }


    partf2_s  = steady_clock::now();
    if (response.size() == request.size()) {
#if CV_MAJOR_VERSION==3
        // result.left = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.right = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);
        result.depth_front = cv::imdecode(response.at(1).image_data_uint8, cv::IMREAD_GRAYSCALE);
#else
        // result.left = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.right = cv::imdecode(response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
        result.depth_front = cv::imdecode(response.at(1).image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);
#endif

        /*
           int width = response.at(1).width;
           int height = response.at(1).height;
           std::vector<float>& floats = response.at(1).image_data_float;

           result.depth = cv::Mat(height, width, CV_32FC1);
           for (int i = 0; i < height; i++) {
           for (int j = 0; j < width; j++) {
           float dist = floats[i*width + j];
           result.depth.at<float>(i,j) = dist; 
           }
           }
           */

        result.depth_front.convertTo(result.depth_front, CV_32FC1, 25.6/256);

        // result.planar_depth = cv::Mat(height, width, CV_32FC1);
        // convertToPlanDepth(result.depth, result.depth);

        // result.disparity = cv::Mat(height, width, CV_32FC1);
        // convertToDisparity(result.depth, result.disparity);
    } else {
        std::cerr << "Images not returned successfully" << std::endl;
    }
    partf2_e  = steady_clock::now();
    auto partf2_t = duration_cast<milliseconds>(partf2_e - partf2_s).count();


    //ground truth values
    static auto initial_pos_gt= response.back().camera_position;
    result.pose_gt.position.x = response.back().camera_position.x() - initial_pos_gt.x();
    result.pose_gt.position.y = response.back().camera_position.y() - initial_pos_gt.y();
    result.pose_gt.position.z = response.back().camera_position.z() - initial_pos_gt.z();

    result.pose_gt.orientation.x = response.back().camera_orientation.x();
    result.pose_gt.orientation.y = response.back().camera_orientation.y();
    result.pose_gt.orientation.z = response.back().camera_orientation.z();
    result.pose_gt.orientation.w = response.back().camera_orientation.w();

    if(this->localization_method == "gps") {
        static auto initial_pos_gps = client->getPosition();
        auto p = client->getPosition();
        auto q = client->getOrientation();
        result.pose.position.x = p.x() - initial_pos_gps.x();
        result.pose.position.y = p.y() - initial_pos_gps.y();
        result.pose.position.z = p.z() - initial_pos_gps.z();

        result.pose.orientation.x = q.x();
        result.pose.orientation.y = q.y();
        result.pose.orientation.z = q.z();
        result.pose.orientation.w = q.w();
    }


    allf_e = steady_clock::now();
    auto allf_t = duration_cast<milliseconds>(allf_e - allf_s).count();
    file_to_output<<"part_f"<<partf_t<< std::endl;
    file_to_output<<"part2_f"<<partf2_t<< std::endl;
    file_to_output<<"all_f"<<allf_t<< std::endl;
    file_to_output.close();

    return result;
}

geometry_msgs::Twist input_sampler::twist()
{
    geometry_msgs::Twist result;
    auto lv = client->getVelocity();

    // Set linear velocities
    result.linear.x = lv.y();
    result.linear.y = lv.x();
    result.linear.z = -(lv.z());

    // Set angular velocities (we can't currently get them, so just assume they're all 0)
    result.angular.x = 0;
    result.angular.y = 0;
    result.angular.z = 0;

    return result;
}

void input_sampler::poll_frame_sphere()
{
    static uint64 last_time_stamp = 0; 
    msr::airlib::MultirotorRpcLibClient *my_client = new msr::airlib::MultirotorRpcLibClient(this->ip_addr, this->port);

    const int max_tries = 1000000;
    
    std::vector<ImageReq> request;
    for (int i = 0; i < N_CAMERAS; ++i) {
    	request.push_back(request_options[i]);
    }

    try{ 
        struct image_response response;
        while(true){
            ros::Time start_hook_t = ros::Time::now();
            client_mutex.lock(); 
            if (exit_out) {
                std::cout << "killing the poll thread" << std::endl;
                client_mutex.unlock(); 
                return;
            } 
            
            response.image = my_client->simGetImages(request);
            response.p = my_client->getPosition();
            response.q = my_client->getOrientation();

            for (int i = 0; response.image.size() != request.size() && i < max_tries; i++) {
                response.image = my_client->simGetImages(request);
            }
            
            ros::Time now =  ros::Time::now();
            if (this->localization_method == "vins_mono"){//for vins mono the imgs and imu 
                                                          //need to be synchronized
                response.timestamp = response.image.at(0).time_stamp;
            }else{            //for profiling purposes 
                response.timestamp = (uint64_t) now.sec*1e9 + (uint64_t)now.nsec;
            }
            
            if (last_time_stamp >= response.timestamp) {
                ROS_ERROR_STREAM("imag time stamps shouldn't be out of order"<< last_time_stamp<< " " <<response.timestamp<< " "); 
            }
            last_time_stamp = response.timestamp;

            image_response_queue_mutex.lock(); 
            if (response.image.size() == request.size()) {
                response.poll_time = (ros::Time::now() - start_hook_t).toSec()*1e9;
                //ROS_INFO_STREAM("poll only"<<response.poll_time); 
                image_response_queue.push(response);
            } 
            image_response_queue_mutex.unlock(); 
            
            client_mutex.unlock(); 
        }
    }
    catch(...){
        printf("got here"); 
        return; 
        exit(0);
    }
}

struct image_response_decoded input_sampler::image_decode_sphere(){
    try{ 
        image_response_queue_mutex.lock(); 
        if (image_response_queue.empty()){
            image_response_queue_mutex.unlock(); 
            struct image_response_decoded result;
            result.valid_data = false; 
            return  result;
        }

        struct image_response response = image_response_queue.back();
        std::queue<struct image_response>().swap(image_response_queue);
        image_response_queue_mutex.unlock(); 

        struct image_response_decoded result;

        geometry_msgs::Pose hardcoded_camera_pos;
        static auto hardcoded_initial_pos_gt = response.image[0].camera_position;
		hardcoded_camera_pos.position.x = response.image[0].camera_position.x() - hardcoded_initial_pos_gt.x();
		hardcoded_camera_pos.position.y = response.image[0].camera_position.y() - hardcoded_initial_pos_gt.y();
		hardcoded_camera_pos.position.z = response.image[0].camera_position.z() - hardcoded_initial_pos_gt.z();

        for (int i = 0; i < N_CAMERAS; ++i) {
            result.depths.push_back(cv::Mat());

#if CV_MAJOR_VERSION==3
            result.depths[i] = cv::imdecode(response.image.at(i).image_data_uint8, cv::IMREAD_GRAYSCALE);
#else
            result.depths[i] = cv::imdecode(response.image.at(i).image.image_data_uint8, CV_LOAD_IMAGE_GRAYSCALE);
#endif

            result.depths[i].convertTo(result.depths[i], CV_32FC1, 25.6/256);

            result.poses_gt.push_back(geometry_msgs::Pose());

            //ground truth values
            static auto initial_pos_gt = response.image[i].camera_position;

            if (camera_names[i] == "right" || camera_names[i] == "left") {
            	result.poses_gt[i].position.x = hardcoded_camera_pos.position.x;
            	result.poses_gt[i].position.y = hardcoded_camera_pos.position.y;
            	result.poses_gt[i].position.z = hardcoded_camera_pos.position.z;
            }
            else {
				result.poses_gt[i].position.x = response.image[i].camera_position.x() - initial_pos_gt.x();
				result.poses_gt[i].position.y = response.image[i].camera_position.y() - initial_pos_gt.y();
				result.poses_gt[i].position.z = response.image[i].camera_position.z() - initial_pos_gt.z();
            }

            result.poses_gt[i].orientation.x = response.image[i].camera_orientation.x();
            result.poses_gt[i].orientation.y = response.image[i].camera_orientation.y();
            result.poses_gt[i].orientation.z = response.image[i].camera_orientation.z();
            result.poses_gt[i].orientation.w = response.image[i].camera_orientation.w();
        }

        //// this isn't relevant for roborun since we're using
        // ground truth for localization to begin with
        // TODO: fix this in the future
        static msr::airlib::Vector3r initial_pos_gps = client->getPosition();

        if(this->localization_method == "gps") {
            result.pose.position.x = response.p.x() - initial_pos_gps.x();
            result.pose.position.y = response.p.y() - initial_pos_gps.y();
            result.pose.position.z = response.p.z() - initial_pos_gps.z();

            result.pose.orientation.x = response.q.x();
            result.pose.orientation.y = response.q.y();
            result.pose.orientation.z = response.q.z();
            result.pose.orientation.w = response.q.w();
        }

        result.timestamp = response.timestamp;
        result.poll_time = response.poll_time;
        return result;
    }
    catch(...){
        exit(0); 
    }
}
