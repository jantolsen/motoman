﻿/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, NOV
 * All rights reserved.
 *
 */

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef MOTOMAN_PARALLEL_LINKAGE_JOINT_FEEDBACK_RELAY_HANDLER_H
#define MOTOMAN_PARALLEL_LINKAGE_JOINT_FEEDBACK_RELAY_HANDLER_H

// Include Header-files:
// -------------------------------
#include "motoman_driver/industrial_robot_client/joint_feedback_relay_handler.h"
#include "motoman_driver/parallel_linkage/pl_utilities.h"

// Namespace: Motoman
// -------------------------------
namespace motoman
{
// Namespace: Parallel-Linkage
// -------------------------------
namespace ParallelLinkage
{

/**
 * \brief Motman Parallel-Linkage Specific Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 * This class inherits from the "joint_feedback_relay_handler" of the "industrial_robot_client" to accomodate 
 * Parallel-Linkage implementation  
 */
class JointFeedbackRelayHandler : public industrial_robot_client::joint_feedback_relay_handler::JointFeedbackRelayHandler
{
    bool J23_coupled_;  // Linkage Between joint 2 and joint 3

  // Public Class members
  // -------------------------------
  // Accessible for everyone
  public:

    // Class constructor
    // -------------------------------
    JointFeedbackRelayHandler(int robot_id = -1);

    // Class destructor
    // -------------------------------
    ~JointFeedbackRelayHandler();

    // Class methods
    // -----------------------

      // Parallel-Linkage
      // -------------------------------
      // Check for Parameter-Server for Parallel-Linkage
      bool checkParalellLinkage();

      // Transform
      // -------------------------------
      // Correct for Parellel-Linkage effects if desired 
      // Function Overloading:
      //  Multiple definitions, allows for different ways of calling the function

        // Transform
        // (Joint Trajectory Points)
        bool transform(const trajectory_msgs::JointTrajectoryPoint& pos_in, trajectory_msgs::JointTrajectoryPoint* pos_out);

        // Transform
        // (Dynamic Joints Group)
        bool transform(const motoman_msgs::DynamicJointsGroup& pos_in, motoman_msgs::DynamicJointsGroup* pos_out);

        // Transform
        // (Joint-Vector)
        bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out);

  // Protected Class members
  // -------------------------------
  // Accessible within the class which defines them, 
  // and classes which inherits from the parent class
  protected:
    int robot_id_;
};  

} // Namespace Parallel-Linkage
} // Namespace Motoman
#endif  // MOTOMAN_PARALLEL_LINKAGE_JOINT_FEEDBACK_RELAY_HANDLER_H
