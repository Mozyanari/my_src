# CMakeLists.txt for my_tutorials
# F.Qian, Kanto Gakuin University, Japan
# 10 Feb. 2015

cmake_minimum_required(VERSION 2.8.3)
project(chapter04)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  cv_bridge
  image_transport
)

include_directories(${catkin_INCLUDE_DIRS}
  if($ENV{LEAP_SDK} $ENV{LEAP_SDK}/include)
)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
   leap_motion.msg
)

#add_service_files(DIRECTORY srv
#  FILES
#)

#add_action_files(DIRECTORY action
#  FILES
#)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

# -- for chapter 04 -----------------------------------------------
macro(chapter_04 T)
  add_executable(${T} ${T}/${T}.cpp)
  target_link_libraries(${T} ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES} $ENV{LEAP_SDK}/lib/x64/libLeap.so)
  add_dependencies(${T} chapter04_gencpp)
  install(TARGETS ${T}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

foreach(dir04
  my_joystick
  usb_camera_subscriber
  image_publisher
  image_subscriber
  camera_publisher
  camera_subscriber
  leap_motion_publisher
  leap_motion_subscriber
  leap_turtle_teleop
  laser_scan_stopper
  my_kinect
  )
  chapter_04(${dir04})
endforeach()
