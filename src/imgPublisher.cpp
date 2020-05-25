#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <math.h>
#include <iterator>
#include "common/Common.hpp"
#include <fstream>
#include "input_sampler.h"
#include "Callbacks/callbacks.h"
#include <signal.h>
#include "stereo_msgs/DisparityImage.h"
#include <thread>
#include <mutex>
#include <profile_manager/profiling_data_srv.h>
#include <profile_manager/start_profiling_srv.h>

using namespace std;

// Profiling
long long g_poll_decode_acc = 0;
int g_poll_decode_ctr = 0;
bool CLCT_DATA;

void log_data_before_shutting_down(){

    std::string ns = ros::this_node::getName();
    profile_manager::profiling_data_srv profiling_data_srv_inst;
    
    profiling_data_srv_inst.request.key = "poll_decode";
    profiling_data_srv_inst.request.value = (((double)g_poll_decode_acc)/1e9)/g_poll_decode_ctr;
    if (ros::service::waitForService("/record_profiling_data", 10)){ 
        if(!ros::service::call("/record_profiling_data",profiling_data_srv_inst)){
            ROS_ERROR_STREAM("could not probe data using stats manager");
        }
    }
} 

string localization_method;
extern std::mutex client_mutex;
extern volatile bool exit_out;
void sigIntHandlerPrivate(int sig)
{
    
    log_data_before_shutting_down();
    //my_thread.join(); 
    // client_mutex.lock(); 
    ros::shutdown();
    //abort();
    exit_out = true; 
    std::cout << "killing the main thread" << std::endl;
    std::ofstream myfile;
    myfile.open("/home/wcui/catkin_ws/blah.txt", std::ofstream::app);
    myfile << "killing the main thread" << std::endl;
    myfile.close(); 
    // client_mutex.unlock();
}



sensor_msgs::CameraInfo getCameraParams(){
    double Tx, Fx, Fy, cx, cy, width, height;
    sensor_msgs::CameraInfo CameraParam;

    // Read camera parameters from launch file
    ros::param::get("/airsim_imgPublisher/Tx",Tx);
    ros::param::get("/airsim_imgPublisher/Fx",Fx);
    ros::param::get("/airsim_imgPublisher/Fy",Fy);
    ros::param::get("/airsim_imgPublisher/cx",cx);
    ros::param::get("/airsim_imgPublisher/cy",cy);
    ros::param::get("/airsim_imgPublisher/scale_x",width);
    ros::param::get("/airsim_imgPublisher/scale_y",height);
    ros::param::get("/CLCT_DATA",CLCT_DATA);


    //CameraParam.header.frame_id = "camera";
    CameraParam.header.frame_id = localization_method;

    CameraParam.height = height;
    CameraParam.width = width;

    CameraParam.distortion_model = "plumb_bob";
    CameraParam.D = {0.0, 0.0, 0.0, 0.0, 0.0};

    CameraParam.K = {Fx,  0.0, cx, 
                     0.0, Fy,  cy, 
                     0.0, 0.0, 1};
    CameraParam.R = {1.0, 0.0, 0.0, 
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};
    CameraParam.P = {Fx,  0.0, cx,  Tx, 
                     0.0, Fy,  cy,  0.0, 
                     0.0, 0.0, 1.0, 0.0};

    CameraParam.binning_x = 0;
    CameraParam.binning_y = 0;

    return CameraParam;
}

void CameraPosePublisher(geometry_msgs::Pose CamPose, geometry_msgs::Pose CamPose_gt, const ros::Time& timestamp, std::string pose_string)
{
    static tf::TransformBroadcaster br;
    tf::Transform transformQuad, transformCamera;
    const double sqrt_2 = 1.41421356237;
    transformCamera.setOrigin(tf::Vector3(CamPose.position.y,
                                        CamPose.position.x,
                                        -CamPose.position.z));

    geometry_msgs::Vector3 rpy =  quat2rpy(CamPose.orientation);
    rpy.y = -rpy.y;
    rpy.z = -rpy.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam = rpy2quat(rpy);
    q_cam = quatProd(q_body2cam, q_cam);
    transformCamera.setRotation(tf::Quaternion(q_cam.x,
                                             q_cam.y,
                                             q_cam.z, 
                                             q_cam.w));

    if (localization_method == "gps"){ //note that slam itself posts this transform
        br.sendTransform(tf::StampedTransform(transformCamera, timestamp, "world", localization_method));
    }  
    
    
    //ground truth values
    static tf::TransformBroadcaster br_gt;
    tf::Transform transformQuad_gt, transformCamera_gt;
    transformCamera_gt.setOrigin(tf::Vector3(CamPose_gt.position.y,
                                        CamPose_gt.position.x,
                                        -CamPose_gt.position.z));

    geometry_msgs::Vector3 rpy_gt =  quat2rpy(CamPose_gt.orientation);
    rpy_gt.y = -rpy_gt.y;
    rpy_gt.z = -rpy_gt.z + M_PI/2.0;

    geometry_msgs::Quaternion q_body2cam_gt = setQuat(0.5, -0.5, 0.5, -0.5);

    geometry_msgs::Quaternion q_cam_gt = rpy2quat(rpy_gt);
    q_cam_gt = quatProd(q_body2cam_gt, q_cam_gt);
    transformCamera_gt.setRotation(tf::Quaternion(q_cam_gt.x,
                                             q_cam_gt.y,
                                             q_cam_gt.z, 
                                             q_cam_gt.w));
    br_gt.sendTransform(tf::StampedTransform(transformCamera_gt, timestamp, "world", pose_string));

    if (pose_string == ("camera_" + camera_names[0])) {
		tf::Transform cam2quad(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.35));
		// cam2quad.setOrigin(tf::Vector3(0, -0.45, 0));
        br_gt.sendTransform(tf::StampedTransform(cam2quad, timestamp, pose_string, "ground_truth"));
    }
}

void do_nothing(){
    return;
}
//std::thread poll_frame_thread(do_nothing);


int main(int argc, char **argv)
{
  
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_imgPublisher");
  ros::NodeHandle n;

  double loop_rate_hz;
  ros::param::get("/airsim_imgPublisher/loop_rate", loop_rate_hz);
  ros::Rate loop_rate(loop_rate_hz);
    
  //ROS Messages

  //Parameters for communicating with Airsim
  string ip_addr;
  int portParam;
  ros::param::get("/airsim_imgPublisher/Airsim_ip",ip_addr);
  ros::param::get("/airsim_imgPublisher/Airsim_port", portParam);
  uint16_t port = portParam;

  // Parameter for localizing camera
  if(!ros::param::get("/airsim_imgPublisher/localization_method", localization_method)){
    ROS_FATAL_STREAM("you have not set the localization method");
    return -1;
  }

   //this connects us to the drone 
  //client = new msr::airlib::MultirotorRpcLibClient(ip_addr, port);
  //client->enableApiControl(false);


  //Verbose
  ROS_INFO("Image publisher started! Connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);
  
  //Local variables
  input_sampler input_sample__obj(ip_addr.c_str(), port, localization_method);

  bool all_front = false;
  if (!ros::param::get("/airsim_imgPublisher/all_front",all_front)){
      ROS_ERROR_STREAM("all front is not defined for airsim_imgPublisher");
      exit(0);
  }

  // radhika: note that sphere_view overrides all_front; if sphere_view
  // is set to true, we don't care about all_front anymore
  bool sphere_view = false;
  ros::param::get("/airsim_imgPublisher/sphere_view", sphere_view);

  //Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);

  std::vector<image_transport::Publisher> depth_pubs;

  sensor_msgs::CameraInfo msgCameraInfo = getCameraParams();
  ros::Publisher imgParamDepth_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/camera_info", 1);

  // for multiple cameras
  sensor_msgs::ImagePtr msgDepths[N_CAMERAS];

  // to not break vanilla mavbench
  sensor_msgs::ImagePtr msgImgL, msgImgR, msgDepth_front, msgDepth_back, msgDepth_bottom;
  image_transport::Publisher depth_pub_front, depth_pub_back, imgR_pub, imgL_pub;
  ros::Publisher imgParamL_pub, imgParamR_pub, disparity_pub;

  if (sphere_view) {
	  for (int i = 0; i < N_CAMERAS; ++i) {
		  std::string cam_topic = "/Airsim/depth_" + camera_names[i];
		  image_transport::Publisher pub = it.advertise(cam_topic, 1);
		  depth_pubs.push_back(pub);
	  }
  }
  else {
	  depth_pub_front = it.advertise("/Airsim/depth_front", 1);
	  depth_pub_back = it.advertise("/Airsim/depth_back", 1);
	  imgR_pub = it.advertise("/Airsim/right/image_raw", 1);
	  // imgL_pub = it.advertise("/Airsim/left/image_raw", 1);
	  imgParamL_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/left/camera_info", 1);
	  imgParamR_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/right/camera_info", 1);
  }

  std::thread poll_frame_thread;
  // radhika: I've split the functions for `sphere_view` so that
  // it doesn't break the mavbench API; the way to fix this would
  // be to change `all_front`'s type from a bool to an int.
  if (sphere_view) {
      poll_frame_thread = std::thread(&input_sampler::poll_frame_sphere, 
              &input_sample__obj);
  }
  else {
      poll_frame_thread = std::thread(&input_sampler::poll_frame, 
              &input_sample__obj, all_front);
  }

  signal(SIGINT, sigIntHandlerPrivate);

  ros::Time this_time, last_time;

  while (ros::ok())
  {
	  //this_time = ros::Time::now();
	  //ros::Duration frame_latency = (this_time - last_time);
	  //last_time = ros::Time::now();

	  //std::cout << frame_latency.toSec() << "\n";

      ros::Time start_hook_t = ros::Time::now();

      struct image_response_decoded imgs;
      if (sphere_view) {
          imgs = input_sample__obj.image_decode_sphere();
      }
      else {
          imgs = input_sample__obj.image_decode(all_front);
      }

      if (!imgs.valid_data) {
          continue;
      }

      uint32_t timestamp_s = uint32_t(imgs.timestamp / 1000000000);
      uint32_t timestamp_ns = uint32_t(imgs.timestamp % 1000000000);
      ros::Time timestamp(timestamp_s, timestamp_ns);
      if(imgs.timestamp != uint64_t(timestamp_s)*1000000000 + timestamp_ns){
          std::cout<<"---------------------failed"<<std::setprecision(30)<<imgs.timestamp<< "!=" 
              <<std::setprecision(30)<<timestamp_s*1000000000 + timestamp_ns<<std::endl;
          ROS_ERROR_STREAM("coversion in img publisher failed");
      }

      cv::Mat disparityImageMat;
      imgs.depth_front.convertTo(disparityImageMat, CV_8UC1);
      imgs.depth_back.convertTo(disparityImageMat, CV_8UC1);
      stereo_msgs::DisparityImage disparityImg;
      disparityImg.header.stamp = timestamp;

      //disparityImg.header.frame_id= localization_method;
      disparityImg.header.frame_id= "camera_front";

      disparityImg.f = 128; //focal length, half of the image width
      disparityImg.T = .14; //baseline, half of the distance between the two cameras
      disparityImg.min_disparity = .44; // f.t/z(depth max)
      disparityImg.max_disparity = 179; // f.t/z(depth min)
      disparityImg.delta_d = .018; //possibly change
      disparityImg.image = *(cv_bridge::CvImage(std_msgs::Header(), "8UC1", disparityImageMat).toImageMsg());
      disparityImg.valid_window.x_offset = 0;
      disparityImg.valid_window.y_offset = 0;
      disparityImg.valid_window.height =  144;
      disparityImg.valid_window.width =  256;
      disparityImg.valid_window.do_rectify =  false; //possibly change

      // now for multiple camera
      if (sphere_view) {
          msgCameraInfo.header.stamp = timestamp;
          imgParamDepth_pub.publish(msgCameraInfo);

          for (int i = 0; i < N_CAMERAS; ++i) {
              msgDepths[i] = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depths[i]).toImageMsg();
              msgDepths[i]->header.stamp =  msgCameraInfo.header.stamp;
              msgDepths[i]->header.frame_id = "camera_" + camera_names[i];
              CameraPosePublisher(imgs.pose, imgs.poses_gt[i], timestamp, "camera_" + camera_names[i]);
              depth_pubs[i].publish(msgDepths[i]);
          }
      }
      else {
          // *** F:DN conversion of opencv images to ros images
          // msgImgL = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.left).toImageMsg();
          msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.right).toImageMsg();
          msgDepth_front = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth_front).toImageMsg();
          msgDepth_back = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgs.depth_back).toImageMsg();

          //Stamp messages
          msgCameraInfo.header.stamp = timestamp;
          // msgImgL->header.stamp = msgCameraInfo.header.stamp;
          msgImgR->header.stamp = msgCameraInfo.header.stamp;
          msgDepth_front->header.stamp =  msgCameraInfo.header.stamp;
          msgDepth_back->header.stamp =  msgCameraInfo.header.stamp;

          // Set the frame ids
          //msgDepth_front->header.frame_id = localization_method;
          //msgDepth_back->header.frame_id = localization_method;
          msgDepth_front->header.frame_id = "camera_front";
          msgDepth_back->header.frame_id = "camera_back";

          CameraPosePublisher(imgs.pose, imgs.poses_gt[0], timestamp, "camera_front");
          CameraPosePublisher(imgs.pose, imgs.poses_gt[2], timestamp, "camera_back");

          //Publish images
          imgR_pub.publish(msgImgR);
          depth_pub_front.publish(msgDepth_front);
          depth_pub_back.publish(msgDepth_back);
          imgParamL_pub.publish(msgCameraInfo);
          imgParamR_pub.publish(msgCameraInfo);
          imgParamDepth_pub.publish(msgCameraInfo);
          disparity_pub.publish(disparityImg);
      }

      ros::spinOnce();

      ros::Time end_hook_t = ros::Time::now();

      if (CLCT_DATA) { 
          g_poll_decode_acc += (imgs.poll_time + ((end_hook_t - start_hook_t).toSec()*1e9));
          //ROS_INFO_STREAM("decode "<< (((end_hook_t - start_hook_t).toSec()*1e9))); 
          //ROS_INFO_STREAM("decode "<< imgs.poll_time);
          
          g_poll_decode_ctr++; 
      }
      loop_rate.sleep(); 
  }

  exit_out = true; 
  poll_frame_thread.join();
  //ros::shutdown(); 
  return 0;
}

