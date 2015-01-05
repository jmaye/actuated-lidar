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

#include <angles/angles.h>

#include <laser_assembler/AssembleScans2.h>
#include <laser_assembler/AssembleScans.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <dynamixel/SetMovingSpeed.h>
#include <dynamixel/SetAngleLimits.h>

namespace actuated_lidar {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  ActuatedLidarNode::ActuatedLidarNode(const ros::NodeHandle& nh) :
      nodeHandle_(nh),
      wheelMode_(false),
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
    if (publishPointCloud2_)
      assembleScansServiceClient_ =
        nodeHandle_.serviceClient<laser_assembler::AssembleScans2>(
        assembleScansServiceClientName_);
    else
      assembleScansServiceClient_ =
        nodeHandle_.serviceClient<laser_assembler::AssembleScans>(
        assembleScansServiceClientName_);
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
    ros::spin();
  }

  void ActuatedLidarNode::getParameters() {
    // Laser assembler service client parameters
    nodeHandle_.param<std::string>("assemble_scan_service_client/name",
      assembleScansServiceClientName_, "/assemble_scans2");

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

    // Set moving speed service client parameters
    nodeHandle_.param<std::string>("set_moving_speed_service_client/name",
      setMovingSpeedServiceClientName_, "/dynamixel/set_moving_speed");

    // Set angle limits service client parameters
    nodeHandle_.param<std::string>("set_angle_limits_service_client/name",
      setAngleLimitsServiceClientName_, "/dynamixel/set_angle_limits");

    /// Dynamixel parameters
    nodeHandle_.param<double>("dynamixel/moving_speed", movingSpeed_, 0.1);
    nodeHandle_.param<double>("dynamixel/min_angle", minAngle_, -M_PI / 4.0);
    nodeHandle_.param<double>("dynamixel/max_angle", maxAngle_, M_PI / 4.0);
    if (minAngle_ >= maxAngle_) {
      ROS_WARN_STREAM_NAMED("actuated_lidar_node",
        "Minimum angle should be smaller than maximum angle (min=" << minAngle_
        << ", max=" << maxAngle_ << "). Setting default values.");
      minAngle_ = -M_PI / 4.0;
      maxAngle_ = M_PI / 4.0;
    }
  }

  void ActuatedLidarNode::jointStateSubscriberCallback(const
      sensor_msgs::JointStateConstPtr& msg) {
    if (!wheelMode_) {
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
        else {
          wheelMode_ = true;
          ROS_INFO_STREAM_NAMED("actuated_lidar_node", "Wheel mode set");
        }
      }
      else {
        ROS_WARN_STREAM_NAMED("actuated_lidar_node",
          "setAngleLimitsServiceClient not available");
        return;
      }
    }
    if (!initialized_) {
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
        else {
          initialized_ = true;
          positiveRotation_ = true;
          oldPositiveRotation_ = true;
          lastDirectionChange_ = ros::Time(0, 0);
          ROS_INFO_STREAM_NAMED("actuated_lidar_node", "Servo initialized");
        }
      }
      else {
        ROS_WARN_STREAM_NAMED("actuated_lidar_node",
          "setMovingSpeedServiceClient not available");
        return;
      }
    }
    const auto position = angles::normalize_angle(msg->position[0]);
    if (position > maxAngle_ || position < minAngle_) {
      oldPositiveRotation_ = positiveRotation_;
      positiveRotation_ = position < minAngle_;
      if (ros::service::exists(setMovingSpeedServiceClientName_, false)) {
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
      else {
        ROS_WARN_STREAM_NAMED("actuated_lidar_node",
          "setMovingSpeedServiceClient not available");
        return;
      }
    }
    if (positiveRotation_ != oldPositiveRotation_) {
      if (!lastDirectionChange_.isZero()) {
        if (ros::service::exists(assembleScansServiceClientName_, false)) {
          if (publishPointCloud2_) {
            laser_assembler::AssembleScans2 srv;
            srv.request.begin = lastDirectionChange_;
            srv.request.end = ros::Time::now();
            if (!assembleScansServiceClient_.call(srv))
              ROS_WARN_STREAM_NAMED("actuated_lidar_node",
                "assembleScansServiceClient: service call failed");
            else
              pointCloudPublisher_.publish(srv.response.cloud);
          }
          else {
            laser_assembler::AssembleScans srv;
            srv.request.begin = lastDirectionChange_;
            srv.request.end = ros::Time::now();
            if (!assembleScansServiceClient_.call(srv))
              ROS_WARN_STREAM_NAMED("actuated_lidar_node",
                "assembleScansServiceClient: service call failed");
            else
              pointCloudPublisher_.publish(srv.response.cloud);
          }
        }
        else {
          ROS_WARN_STREAM_NAMED("actuated_lidar_node",
            "assembleScansServiceClient not available");
        }
      }
      lastDirectionChange_ = ros::Time::now();
      oldPositiveRotation_ = positiveRotation_;
    }
  }

}
