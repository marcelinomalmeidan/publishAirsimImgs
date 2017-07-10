#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "rpc/RpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include <iterator>
#include "common/Common.hpp"
#include <fstream>
#include "input_sampler.h"
#include "Callbacks/callbacks.h"



using namespace std;



sensor_msgs::CameraInfo getCameraParams(){
    double Tx, Fx, Fy, cx, cy, width, height;
    sensor_msgs::CameraInfo CameraParam;
    ros::param::get("/airsim_imgPublisher/Tx",Tx);
    ros::param::get("/airsim_imgPublisher/Fx",Fx);
    ros::param::get("/airsim_imgPublisher/Fy",Fy);
    ros::param::get("/airsim_imgPublisher/cx",cx);
    ros::param::get("/airsim_imgPublisher/cy",cy);
    ros::param::get("/airsim_imgPublisher/scale_x",width);
    ros::param::get("/airsim_imgPublisher/scale_y",height);

    CameraParam.header.frame_id = "camera";

    CameraParam.height = height;
    CameraParam.width = width;

    CameraParam.distortion_model = "plumb_bob";
    CameraParam.D = {-0.000591, 0.000519, 0.000001, -0.000030, 0.000000};

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


cv::Mat invertImg16(cv::Mat source){
  cv::Mat saturatedMat = cv::Mat::ones(source.size(), source.type())*65535;
  return saturatedMat - source;
}


cv::Mat invertImg8(cv::Mat source){
  cv::Mat saturatedMat = cv::Mat::ones(source.size(), source.type())*255;
  return saturatedMat - source;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_imgPublisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  //Subscribers --------------------------------------------------------------
  std::string odomTopic;
  ros::param::get("/airsim_imgPublisher/odomTopic", odomTopic);
  ros::Subscriber tfSub = n.subscribe(odomTopic, 10, tfCallback); //Create tf tree

  //Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);
  image_transport::Publisher img_pub = it.advertise("/Airsim/image", 1);
  image_transport::Publisher depth8_pub = it.advertise("/Airsim/depth8", 1);
  image_transport::Publisher depth16_pub = it.advertise("/Airsim/depth", 1);
  ros::Publisher imgParam_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/camera_info", 1);
    
  //ROS Messages
  sensor_msgs::ImagePtr msgImg, msgDepth16, msgDepth8;
  sensor_msgs::CameraInfo msgCameraInfo;

	//Parameters for communicating with Airsim
  string ip_addr;
  int portParam;
  ros::param::get("/airsim_imgPublisher/Airsim_ip",ip_addr);
  ros::param::get("/airsim_imgPublisher/Airsim_port", portParam);
  uint16_t port = portParam;

  //Verbose
  ROS_INFO("Image publisher started! Connecting to:");
  ROS_INFO("IP: %s", ip_addr.c_str());
  ROS_INFO("Port: %d", port);

  
  //Local variables
  input_sampler input_sampler__obj(ip_addr.c_str(), port);
  cv::Mat img, imgDepth8, imgDepth16, img3D; //images to store the polled images
  const string display_name = "Drone View";
  msgCameraInfo = getCameraParams();
  double scale;

  // *** F:DN end of communication with simulator (Airsim)

  while (ros::ok())
  {

    //Get image data
    // img = input_sampler__obj.poll_frame();
    // imgDepth8 = input_sampler__obj.poll_frame_depth();
    // imgDepth8.convertTo(imgDepth16,CV_16U,255);
    // cv::imshow(display_name, img);
    // cv::waitKey(10); //I think we need this otherwise the image shown 
    //                   // is too fast to be observerd

    // *** F:DN conversion of opencv images to ros images
    // msgImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    // msgDepth8 = cv_bridge::CvImage(std_msgs::Header(), "mono8", invertImg8(imgDepth8)).toImageMsg();
    // msgDepth16 = cv_bridge::CvImage(std_msgs::Header(), "16UC1", invertImg16(imgDepth16)).toImageMsg();

    //Stamp messages
    // msgCameraInfo.header.stamp = ros::Time::now();
    // msgImg->header.stamp = msgCameraInfo.header.stamp;
    // msgDepth8->header.stamp =  msgCameraInfo.header.stamp;
    // msgDepth16->header.stamp =  msgCameraInfo.header.stamp;
    

    //Publish images
    // ROS_INFO("New images arrived! Publishing...");
    // img_pub.publish(msgImg);
    // depth8_pub.publish(msgDepth8);
    // depth16_pub.publish(msgDepth16);
    // imgParam_pub.publish(msgCameraInfo);

    auto imgs = input_sampler__obj.poll_frame();

    //cv::imshow("left", imgs.left);
    //cv::imshow("right", imgs.right);
    cv::imshow("depth", imgs.depth);

    imgs.planar_depth.convertTo(imgs.planar_depth, CV_8UC1);
    cv::imshow("planar_depth", imgs.planar_depth);

    imgs.disparity.convertTo(imgs.disparity, CV_8UC1);
    cv::imshow("disparity", imgs.disparity);

    cv::waitKey(10);

    ros::spinOnce();
    
    loop_rate.sleep();
  }

  cv::destroyAllWindows();

  return 0;
}
