/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "ActuatedLidarNode.h"

#include <cmath>

#include <ros/rate.h>

#include <angles/angles.h>

#include <laser_assembler/AssembleScans.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <dynamixel/SetGoalPosition.h>
#include <dynamixel/SetMovingSpeed.h>
#include <dynamixel/SetAngleLimits.h>

namespace actuated_lidar {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ActuatedLidarNode::ActuatedLidarNode(const ros::NodeHandle& nh) :
      nodeHandle_(nh),
      setInitialPosition_(true),
      initialized_(false) {
    // retrieve configurable parameters
    getParameters();

    // init publisher
    if (publishPointCloud2_)
      pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(
        pointCloudPublisherTopic_, pointCloudPublisherQueueSize_);
    else
      pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(
        pointCloudPublisherTopic_, pointCloudPublisherQueueSize_);

    // init subscriber
    jointStateSubscriber_ = nodeHandle_.subscribe(jointStateSubscriberTopic_,
      jointStateSubscriberQueueSize_,
      &ActuatedLidarNode::jointStateSubscriberCallback, this);

    // init services
    assembleScansServiceClient_ =
      nodeHandle_.serviceClient<laser_assembler::AssembleScans>(
      assembleScansServiceClientName_);
    setGoalPositionServiceClient_ =
      nodeHandle_.serviceClient<dynamixel::SetGoalPosition>(
      setGoalPositionServiceClientName_);
    setMovingSpeedServiceClient_ =
      nodeHandle_.serviceClient<dynamixel::SetMovingSpeed>(
      setMovingSpeedServiceClientName_);
    setAngleLimitsServiceClient_ =
      nodeHandle_.serviceClient<dynamixel::SetAngleLimits>(
      setAngleLimitsServiceClientName_);
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void ActuatedLidarNode::spin() {
    ros::Rate loopRate(loopRate_);
    while (nodeHandle_.ok()) {
      if (setInitialPosition_) {
        ROS_INFO_STREAM_NAMED("actuated_lidar_node",
          "Setting initial position");
        if (ros::service::exists(setAngleLimitsServiceClientName_, false)) {
          dynamixel::SetAngleLimits srv;
          srv.request.cw_angle_limit = 0.0;
          srv.request.ccw_angle_limit = 2 * M_PI;
          if (!setAngleLimitsServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node",
              "setAngleLimitsServiceClient: service call failed");
            continue;
          }
          else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node", srv.response.message);
            continue;
          }
        }
        if (ros::service::exists(setGoalPositionServiceClientName_, false)) {
          dynamixel::SetGoalPosition srv;
          srv.request.goal_position = startPosition_;
          srv.request.moving_speed = movingSpeed_;
          srv.request.torque_limit = 1.0;
          if (!setGoalPositionServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node",
              "setGoalPositionServiceClient: service call failed");
            continue;
          }
          else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node", srv.response.message);
            continue;
          }
          else {
            ROS_INFO_STREAM_NAMED("actuated_lidar_node",
              "Initial position command sent: " << startPosition_);
            setInitialPosition_ = false;
          }
        }
      }
      ros::spinOnce();
      loopRate.sleep();
    }
  }

  void ActuatedLidarNode::getParameters() {
    // Miscellaneous ROS parameters
    nodeHandle_.param<double>("ros/loop_rate", loopRate_, 1.0);

    // Laser assembler service client parameters
    nodeHandle_.param<std::string>("assemble_scan_service_client/name",
      assembleScansServiceClientName_, "assemble_scans");

    // Joint state subscriber parameters
    nodeHandle_.param<std::string>("joint_state_subscriber/topic",
      jointStateSubscriberTopic_, "/dynamixel/joint_state");
    nodeHandle_.param<int>("joint_state_subscriber/queue_size",
      jointStateSubscriberQueueSize_, 100);

    // Point cloud publisher parameters
    nodeHandle_.param<std::string>("point_cloud_publisher/topic",
      pointCloudPublisherTopic_, "point_cloud");
    nodeHandle_.param<int>("point_cloud_publisher/queue_size",
      pointCloudPublisherQueueSize_, 100);
    nodeHandle_.param<bool>("point_cloud_publisher/publish_point_cloud_2",
      publishPointCloud2_, true);

    // Set goal position service client parameters
    nodeHandle_.param<std::string>("set_goal_positio_service_client/name",
      setGoalPositionServiceClientName_, "/dynamixel/set_goal_position");

    // Set moving speed service client parameters
    nodeHandle_.param<std::string>("set_moving_speed_service_client/name",
      setMovingSpeedServiceClientName_, "/dynamixel/set_moving_speed");

    // Set angle limits service client parameters
    nodeHandle_.param<std::string>("set_angle_limits_service_client/name",
      setAngleLimitsServiceClientName_, "/dynamixel/set_angle_limits");

    /// Dynamixel parameters
    nodeHandle_.param<double>("dynamixel/start_position", startPosition_, 0.0);
    nodeHandle_.param<double>("dynamixel/moving_speed", movingSpeed_, 0.1);
    nodeHandle_.param<double>("dynamixel/angle", angle_, M_PI / 4.0);
    nodeHandle_.param<double>("dynamixel/tolerance", tolerance_, 0.003068711);
  }

  void ActuatedLidarNode::jointStateSubscriberCallback(const
      sensor_msgs::JointStateConstPtr& msg) {
    const auto position = msg->position[0];
    if (!initialized_) {
      const auto error = std::fabs(angles::shortest_angular_distance(
        startPosition_, position));
      if (error <= tolerance_) {
        ROS_INFO_STREAM_NAMED("actuated_lidar_node",
          "Initial position reached: " << startPosition_);
        if (ros::service::exists(setAngleLimitsServiceClientName_, false)) {
          dynamixel::SetAngleLimits srv;
          srv.request.cw_angle_limit = 0.0;
          srv.request.ccw_angle_limit = 0.0;
          if (!setAngleLimitsServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node",
              "setAngleLimitsServiceClient: service call failed");
            return;
          }
          else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node", srv.response.message);
            return;
          }
        }
        if (ros::service::exists(setMovingSpeedServiceClientName_, false)) {
          dynamixel::SetMovingSpeed srv;
          srv.request.moving_speed = movingSpeed_;
          srv.request.torque_limit = 1.0;
          if (!setMovingSpeedServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node",
              "setMovingSpeedServiceClient: service call failed");
            return;
          }
          else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node", srv.response.message);
            return;
          }
        }
        initialized_ = true;
        positiveRotation_ = true;
      }
    }
    else {
      if ((std::fabs(angles::shortest_angular_distance(position,
          startPosition_ + angle_)) < tolerance_ && positiveRotation_) ||
          (std::fabs(angles::shortest_angular_distance(position,
          startPosition_ - angle_)) < tolerance_ && !positiveRotation_)) {
        if (ros::service::exists(setMovingSpeedServiceClientName_, false)) {
          positiveRotation_ = !positiveRotation_;
          dynamixel::SetMovingSpeed srv;
          srv.request.moving_speed = positiveRotation_? movingSpeed_ :
            -movingSpeed_;
          srv.request.torque_limit = 1.0;
          if (!setMovingSpeedServiceClient_.call(srv)) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node",
              "setMovingSpeedServiceClient: service call failed");
            return;
          }
          else if (!srv.response.response) {
            ROS_WARN_STREAM_NAMED("actuated_lidar_node", srv.response.message);
            return;
          }
        }
      }
    }
  }

}
