cmake_minimum_required(VERSION 2.8.3)

project(airsim_img_publisher)


set (Airlib_addr  $ENV{HOME}/AirSim/AirLib)
set (catkin_workspace_path $ENV{HOME}/catkin_ws) 


## Compile as C++11, supported in ROS Kinetic and newer

# F:DN to be added when airsim communication is desired
add_compile_options(-std=c++11)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF) #behzad added, need this always
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
# include
    ${Airlib_addr}/include   
    ${Airlib_addr}/include/controllers
    ${EIGEN3_INCLUDE_DIR}
    #/usr/local/cuda/include/
   ${catkin_workspace_path}/src/publishAirsimImgs/include 
   ${catkin_workspace_path}/src/publishAirsimImgs/src
   )
link_directories(
    ${Airlib_addr}/lib 
    ${Airlib_addr}/deps/MavLinkCom/lib  
    ${Airlib_addr}/deps/rpclib/lib
    ${PCL_LIBRARY_DIRS})





## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  cv_bridge
  tf
)

find_package( PCL REQUIRED )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)

####################

###################################
## catkin specific configuration ##
###################################
catkin_package( CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy 
  std_msgs
)

###########
## Build ##
###########
include_directories( # include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ) 

 add_definitions(${PCL_DEFINITIONS})



add_executable(airsim_imgPublisher 
               src/imgPublisher.cpp 
               src/input_sampler.cpp
               src/HelperFunctions/QuatRotEuler.cpp
               src/Callbacks/callbacks.cpp)
target_link_libraries(airsim_imgPublisher ${catkin_LIBRARIES} AirLib  MavLinkCom rpc pthread ${OpenCV_LIBS})

add_dependencies(airsim_imgPublisher airsim_img_publisher_generate_messages_cpp)

