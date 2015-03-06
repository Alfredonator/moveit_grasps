/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Dave Coleman
// Desc:   Generates grasps for a cube

#ifndef MOVEIT_GRASPS__MOVEIT_GRASPS_H_
#define MOVEIT_GRASPS__MOVEIT_GRASPS_H_

// ROS
#include <ros/ros.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++
#include <cstdlib>
#include <string>
#include <math.h>
#define _USE_MATH_DEFINES

#include <moveit_grasps/grasp_data.h>

namespace moveit_grasps
{

static const double RAD2DEG = 57.2957795;

// Grasp axis orientation
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_direction_t {UP, DOWN};
enum grasp_rotation_t {FULL, HALF};

// Class
class Grasps
{
private:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Transform from frame of box to global frame
  Eigen::Affine3d object_global_transform_;

  // Display more output both in console and in Rviz (with arrows and markers)
  bool verbose_;

  // Number of grasp points to generate around 
  int number_grasp_points_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles). With GCC, this is done with a attribute ((aligned(16))).

  /**
   * \brief Constructor
   */
  Grasps(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose = false);

  /**
   * \brief Destructor
   */
  ~Grasps();

  /**
   * \brief Create all possible grasp positions for a block
   * \param pose of block, where vector arrow is parallel to table plane
   * \param data describing end effector
   * \param resulting generated possible grasps
   * \return true if successful
   */ 
  bool generateBlockGrasps(const Eigen::Affine3d& object_pose, const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Create grasp positions in one axis around a single pose
   *        Note: to visualize these grasps use moveit_visual_tools.publishAnimatedGrasps() function or
   *        moveit_visual_tools.publishIKSolutions() with the resulting data
   * \param pose - center point of object to be grasped
   * \param axis - axis relative to object pose to rotate generated grasps around
   * \param direction - a parallel gripper is typically symetric such that it can perform the same grasp 
   *                    180 degree around. this option allows to generate a flipped grasp pose
   * \param rotation - amount to rotate around the object - 180 or 360 degrees
   * \param hand_roll - amount in radians to roll wrist with respect to center point of object during grasp. use 0 by default
   * \param grasp_data - parameters specific to the robot geometry
   * \param possible_grasps - the output solution vector of possible grasps to attempt. ok if pre-populated
   * \return true if successful
   */
  bool generateAxisGrasps(
    const Eigen::Affine3d& object_pose,
    grasp_axis_t axis,
    grasp_direction_t direction,
    grasp_rotation_t rotation,
    double hand_roll,
    const GraspData& grasp_data,
    std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Create possible grasp positions around a cuboid 
   * \param cuboid_pose pose of cuboid 
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param grasp_data data describing end effector
   * \param possible_grasps possible grasps generated
   * \return true if successful
   */
  bool generateCuboidGrasps(const Eigen::Affine3d& cuboid_pose, float depth, float width,float height, 
                            float max_grasp_size, const moveit_grasps::GraspData& grasp_data, 
                            std::vector<moveit_msgs::Grasp>& possible_grasps);
  
  /**
   * \brief Create grasp positions around one axis of a cuboid
   * \param cuboid_pose pose of cuboid 
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param axis axis of cuboid to generate grasps around
   * \param grasp_data data describing end effector
   * \param possible_grasps possible grasps generated
   * \return true if successful
   */
  bool generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose, float depth, float width, float height, 
                                grasp_axis_t axis, const moveit_grasps::GraspData& grasp_data, 
                                std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Generate grasp points around the perimeter of the cuboid
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \return a list of points around the cuboid
   */
  Eigen::ArrayXXf generateCuboidGraspPoints(double length, double width, double radius);

  /**
   * \brief Using an input grasp description, get the pregrasp pose
   * \param grasp description
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Helper to convert a robot-specific grasp to an arrow pointed in the right direction
   * \param grasp - the grasp to show
   * \param arm - the planning group of the arm we want to display
   * \return true on success
   */
  void publishGraspArrow(geometry_msgs::Pose grasp, const GraspData& grasp_data, const rviz_visual_tools::colors &color, double approach_length = 0.1);

  /**
   * \brief Getter for Verbose
   */ 
  bool getVerbose()
  {
    return verbose_;
  }
  
  /**
   * \brief Setter for Verbose
   */
  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }
  


}; // end of class

typedef boost::shared_ptr<Grasps> GraspsPtr;
typedef boost::shared_ptr<const Grasps> GraspsConstPtr;

} // namespace

#endif
