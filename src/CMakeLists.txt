remake_ros_package(
  actuated_lidar
  DEPENDS roscpp rospy rosbash hokuyo_node laser_assembler dynamixel
    tf2_ros sensor_msgs angles
  DESCRIPTION "Actuated lidar interface."
)
remake_ros_package_config_generate(PACKAGE actuated_lidar
  EXTRA_CFLAGS -std=c++0x)
