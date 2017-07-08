# Disclaimer
Most of the code in this repo was developed by Behzad Boroujerdian and Hasan Genc from the Department Of Electrical and Computer Engineering at The University of Texas at Austin. I put some finishing touches and wrote the guidelines below in the hope that this will be useful for others. This is supposed to be a preliminary solution while Microsoft don't provide an alternative one.

# Contents
This repo allows you to publish images from airsim into ROS.
Features within this repo:
- Publishes RGB data into the topic /Airsim/image
- Publishes depth data into the topic /Airsim/depth
- Publishes camera calibration parameters into /Airsim/camera_info
- Publishes a tf tree with the origin, the position/orientation of the quadcoper, and the position/orientation of the camera.

## Dependencies
This repo is supposed to run on Linux (only tested with Ubuntu 16.04, ROS Kinetic). 

- Eigen

```sudo apt-get install libeigen3-dev ```

- Then, you have to compile Airlib.

``` cd ~ ``` 

```  git clone https://github.com/Microsoft/AirSim.git ``` 

   - Replace the /Airsim/build.sh with the build.sh in /extras/ in the current repo.
   - Replace the /Airsim/cmake/CMakeLists.txt with the CMakeLists1.txt in /extras/ in the current repo. Rename it to CMakeLists.txt.
   - Replace the /Airsim/cmake/MavLinkCom/CMakeLists.txt with the CMakeLists2.txt in /extras/ in the current repo. Rename it to CMakeLists.txt.
   - Run "build.sh" from Airsim's root directory.
  
- If you want the tf tree to be published, you will need mavros to communicate with px4 and get its pose. Then, you need to install mavros as follows:

``` sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras```

  - In order to run mavros, you can follow the example in /launch/mavrosAirsim.launch. Note that you will have to change the fcu_url parameter to match the IP/Ports in which Airsim is running. All these informations can be found in the settings.json file for your Airsim configuration. The ports you are looking for are the "LogViewerPort" and the "UdpPort". Note that the settings.json file have to be configured such that "LogViewerHostIp" and "UdpIp" both have the IP of the computer that will run mavros. 
  
- Copy the present repo into your catkin workspace (e.g.):

``` cd ~/catkin_ws/src ```

``` git clone https://github.com/marcelinomalmeidan/publishAirsimImgs.git ```

- Open the CMakeLists.txt and change the aliases for ```Airlib_addr``` and ```catkin_workspace_path``` to match your local Airlib folder and your local catkin workspace folder.

``` cd ~/catkin_ws```

``` catkin_make```

## Running image publisher
- Run Airsim.
- Run mavros:

```roslaunch airsim_img_publisher mavrosAirsim.launch```

- Change the IP configuration in ```/launch/pubImages```  to match the IP in which Airsim is running. Then:

```roslaunch airsim_img_publisher pubImages.launch```

## RVIZ configuration file

An RVIZ configuration file can be found in ```/Extras/rvizConfig.rviz```. This configuration allows a user to see the published images, as well as the tf tree.

- ```rosrun rviz rviz -d ~/catkin_ws/src/px4_control/Extras/rvizConfig.rviz ```
