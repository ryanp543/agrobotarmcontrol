cmake_minimum_required(VERSION 2.8.3)

project(agrobot_arm_v2)

find_package(catkin REQUIRED COMPONENTS
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FloatList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS urdf controller_manager joint_state_controller robot_state_publisher roscpp rospy std_msgs message_runtime
)


find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
