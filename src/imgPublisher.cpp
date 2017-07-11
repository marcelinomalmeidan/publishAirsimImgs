// #include "ros/ros.h"
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

void CameraPosePublisher(geometry_msgs::Pose CamPose){
  static tf::TransformBroadcaster br;
  tf::Transform transformQuad, transformCamera;
  double sqrt_2 = 1.41421356237;

  transformCamera.setOrigin(tf::Vector3(CamPose.position.y,
                                        CamPose.position.x,
                                        -CamPose.position.z));
   Eigen::Matrix3d R;
   // R <<  0.0,  0.0, 1.0,
   //      -1.0,  0.0, 0.0,
   //       0.0, -1.0, 0.0; 
   // R <<  1.0,  0.0, 0.0,
   //       0.0,  1.0, 0.0,
   //       0.0,  0.0, 1.0; 
   // R = rotx(-M_PI/2.0);

         //Function to get yaw from a quaternion
  double YawCam = getHeadingFromQuat(CamPose.orientation);

  // geometry_msgs::Quaternion q = rot2quat(R);
  geometry_msgs::Quaternion q_Yaw = setQuat(0.0,0.0,sin(-YawCam),cos(-YawCam));
  geometry_msgs::Quaternion q_body2cam = setQuat(-sqrt_2/2.0, 0, 0, sqrt_2/2.0);
  geometry_msgs::Quaternion q_cam = quatProd(q_Yaw,CamPose.orientation);
  q_cam = quatProd(q_body2cam,q_cam);
  transformCamera.setRotation(tf::Quaternion(q_cam.x,
                                             q_cam.y,
                                             q_cam.z, 
                                             q_cam.w));

  br.sendTransform(tf::StampedTransform(transformCamera, ros::Time::now(), "fcu", "camera"));
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "airsim_imgPublisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  //Subscribers --------------------------------------------------------------
  std::string odomTopic;
  ros::param::get("/airsim_imgPublisher/odomTopic", odomTopic);
  ros::Subscriber tfSub = n.subscribe(odomTopic, 10, tfCallback); //Create tf tree

  //Publishers ---------------------------------------------------------------
  image_transport::ImageTransport it(n);
  image_transport::Publisher imgL_pub = it.advertise("/stereo/left/image_raw", 1);
  image_transport::Publisher imgR_pub = it.advertise("/stereo/right/image_raw", 1);
  image_transport::Publisher depth_pub = it.advertise("/Airsim/depth", 1);
  ros::Publisher imgParamL_pub = n.advertise<sensor_msgs::CameraInfo> ("/stereo/left/camera_info", 1);
  ros::Publisher imgParamR_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/right/camera_info", 1);
  ros::Publisher imgParamDepth_pub = n.advertise<sensor_msgs::CameraInfo> ("/Airsim/camera_info", 1);
    
  //ROS Messages
  sensor_msgs::ImagePtr msgImgL, msgImgR, msgDepth;
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
  cv::Mat img, imgDepth, imgDepthThr; //images to store the polled images
  const string display_name = "Drone View";
  msgCameraInfo = getCameraParams();
  float scale, maxDist;

  // *** F:DN end of communication with simulator (Airsim)

  while (ros::ok())
  {
    ros::param::get("/airsim_imgPublisher/scale",scale);
    auto imgs = input_sampler__obj.poll_frame(scale);

    //cv::imshow("left", imgs.left);
    //cv::imshow("right", imgs.right);
    // cv::imshow("depth", imgs.depth);

    // imgs.planar_depth.convertTo(imgs.planar_depth, CV_8UC1);
    // cv::imshow("planar_depth", imgs.planar_depth);

    // imgs.disparity.convertTo(imgs.disparity, CV_8UC1);
    // cv::imshow("disparity", imgs.disparity);

    // imgs.planar_depth.convertTo(imgDepth16,CV_16U,255);
    // cv::waitKey(10);

    //Saturate depth to maximum threshold
    ros::param::get("/airsim_imgPublisher/maxDist",maxDist);
    printf("Max dist: %f", maxDist);
    threshold(imgs.depth, imgDepthThr, maxDist,100, 4);

    // *** F:DN conversion of opencv images to ros images
    msgImgL = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.left).toImageMsg();
    msgImgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs.right).toImageMsg();
    msgDepth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", imgDepthThr).toImageMsg();

    //Stamp messages
    msgCameraInfo.header.stamp = ros::Time::now();
    msgImgL->header.stamp = msgCameraInfo.header.stamp;
    msgImgR->header.stamp = msgCameraInfo.header.stamp;
    msgDepth->header.stamp =  msgCameraInfo.header.stamp;

    //Publish transforms into tf tree
    CameraPosePublisher(imgs.pose);

    //Publish images
    ROS_INFO("New images arrived! Publishing...");
    imgL_pub.publish(msgImgL);
    imgR_pub.publish(msgImgR);
    depth_pub.publish(msgDepth);
    imgParamL_pub.publish(msgCameraInfo);
    imgParamR_pub.publish(msgCameraInfo);
    imgParamDepth_pub.publish(msgCameraInfo);


    ros::spinOnce();
    
    loop_rate.sleep();
  }

  cv::destroyAllWindows();

  return 0;
}
