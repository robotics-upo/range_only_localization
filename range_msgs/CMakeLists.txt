cmake_minimum_required(VERSION 2.8.3)
project(range_msgs)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
	geometry_msgs
    message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
    P2PRange.msg
    P2PRangeWithPose.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime)

install(DIRECTORY include/range_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
