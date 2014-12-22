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

/** \file ActuatedLidarNode.h
    \brief This file defines the ActuatedLidarNode class which implements the
           actuated lidar node.
  */

#ifndef ACTUATED_LIDAR_NODE_H
#define ACTUATED_LIDAR_NODE_H

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

namespace actuated_lidar {

  /** The class ActuatedLidarNode implements the actuated lidar node.
      \brief Actuated lidar node
    */
  class ActuatedLidarNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    ActuatedLidarNode(const ros::NodeHandle& nh);
    /// Copy constructor
    ActuatedLidarNode(const ActuatedLidarNode& other) = delete;
    /// Copy assignment operator
    ActuatedLidarNode& operator = (const ActuatedLidarNode& other) = delete;
    /// Move constructor
    ActuatedLidarNode(ActuatedLidarNode&& other) = delete;
    /// Move assignment operator
    ActuatedLidarNode& operator = (ActuatedLidarNode&& other) = delete;
    /// Destructor
    ~ActuatedLidarNode() = default;
    /** @}
      */

    /** \name Public Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Retrieves parameters from the parameter server
    void getParameters();
    /// Joint state subscriber callback
    void jointStateSubscriberCallback(const sensor_msgs::JointStateConstPtr&
      msg);
    /** @}
      */

    /** \name Private members
      @{
      */
    /// ROS node handle
    ros::NodeHandle nodeHandle_;
    /// Loop rate
    double loopRate_;
    /// ROS laser assembler service client
    ros::ServiceClient assembleScansServiceClient_;
    /// ROS laser assembler service client name
    std::string assembleScansServiceClientName_;
    /// ROS joint state subscriber
    ros::Subscriber jointStateSubscriber_;
    /// ROS joint state subscriber topic name
    std::string jointStateSubscriberTopic_;
    /// ROS joint state subscriber queue size
    int jointStateSubscriberQueueSize_;
    /// ROS point cloud publisher
    ros::Publisher pointCloudPublisher_;
    /// ROS point cloud publisher topic name
    std::string pointCloudPublisherTopic_;
    /// ROS point cloud publisher queue size
    int pointCloudPublisherQueueSize_;
    /// Use sensor_msgs/PointCloud2
    bool publishPointCloud2_;
    /// Start position of the servo
    double startPosition_;
    /// Velocity of the servo
    double movingSpeed_;
    /// Minimum angle of the servo
    double minAngle_;
    /// Maximum angle of the servo
    double maxAngle_;
    /// ROS set goal position service client
    ros::ServiceClient setGoalPositionServiceClient_;
    /// ROS set goal position service client name
    std::string setGoalPositionServiceClientName_;
    /// ROS set moving speed service client
    ros::ServiceClient setMovingSpeedServiceClient_;
    /// ROS set moving speed service client name
    std::string setMovingSpeedServiceClientName_;
    /// ROS set angle limits service client
    ros::ServiceClient setAngleLimitsServiceClient_;
    /// ROS set angle limits service client name
    std::string setAngleLimitsServiceClientName_;
    /// Set the servo to the initial position
    bool setInitialPosition_;
    /// Servo has reached the initial position and is ready
    bool initialized_;
    /** @}
      */

  };

}

#endif // ACTUATED_LIDAR_NODE_H
