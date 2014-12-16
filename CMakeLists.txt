cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  actuated-lidar-${ROS_DISTRIBUTION}
  VERSION 0.1
  RELEASE alpha
  SUMMARY "ROS interface for actuated lidar."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "http://github.com/jmaye/actuated-lidar"
  LICENSE "GNU Lesser General Public License (LGPLv3)"
  PREFIX actuated-lidar-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/actuated-lidar
  EXTRA_CXX_FLAGS "-std=c++0x -Wall -Wextra"
)

remake_var_name(GIT_REVISION_VAR ${REMAKE_PROJECT_NAME} GIT_REVISION)

remake_ros_pack_deb()

if(${ROS_DISTRIBUTION} STREQUAL indigo)
  remake_ros_distribute_deb(
    DISTRIBUTION trusty
    SECTION libs
    DEPENDS remake pkg-config
    UPLOAD ppa:jmaye/ros
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif()
