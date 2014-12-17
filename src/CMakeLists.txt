remake_ros_package(
  actuated_lidar
  DEPENDS roscpp rospy rosbash laser_assembler
  DESCRIPTION "Actuated lidar interface."
)
remake_ros_package_config_generate(PACKAGE actuated_lidar
  EXTRA_CFLAGS -std=c++0x)
