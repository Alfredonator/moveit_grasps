/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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

/* Author: Mike Laumtman
   Desc:   Demonstrates a full pick using MoveIt Grasps
*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//overload

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/RobotState.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_planner.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Server message
#include <moveit_grasps/Grasp.h>

namespace moveit_grasps {
    static const std::string LOGNAME = "grasp_pipeline_demo";


    namespace {
        bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                          moveit_visual_tools::MoveItVisualToolsPtr visual_tools, robot_state::RobotState *robot_state,
                          const robot_model::JointModelGroup *group, const double *ik_solution) {
            robot_state->setJointGroupPositions(group, ik_solution);
            robot_state->update();
            return !planning_scene->isStateColliding(*robot_state, group->getName());
        }

        void waitForNextStep(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, std::string prompt) {
           // visual_tools->prompt(prompt);
        }

    }  // end annonymous namespace


    class GraspPipelineDemo {
    public:
        // Constructor
        GraspPipelineDemo() : nh_("~") {
            // Get arm info from param server
            const std::string parent_name = "grasp_filter_demo";  // for namespacing logging messages
            rosparam_shortcuts::get(parent_name, nh_, "planning_group_name", planning_group_name_);
            rosparam_shortcuts::get(parent_name, nh_, "ee_group_name", ee_group_name_);

            ROS_INFO_STREAM_NAMED("test", "End Effector: " << ee_group_name_);
            ROS_INFO_STREAM_NAMED("test", "Planning Group: " << planning_group_name_);

            loadScene();
            setupGraspPipeline();
        }

        void loadScene() {
            // ---------------------------------------------------------------------------------------------
            // Load planning scene to share
            planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
            if (!planning_scene_monitor_->getPlanningScene()) {
                ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
                return;
            }
            planning_scene_monitor_->startPublishingPlanningScene(
                    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                    "move_group/monitored_planning_scene");
            planning_scene_monitor_->getPlanningScene()->setName("move_group/monitored_planning_scene");

            robot_model_loader::RobotModelLoaderPtr robot_model_loader;
            robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));

            // Load the robot model
            robot_model_ = robot_model_loader->getModel();
            arm_jmg_ = robot_model_->getJointModelGroup(planning_group_name_);

            // ---------------------------------------------------------------------------------------------
            // Load the Robot Viz Tools for publishing to Rviz
            visual_tools_.reset(
                    new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(), "/rviz_visual_tools", planning_scene_monitor_));
//            visual_tools_->loadMarkerPub();
//            visual_tools_->loadRobotStatePub("/display_robot_state");
//            visual_tools_->loadTrajectoryPub("/display_planned_path");
//            visual_tools_->loadSharedRobotState();
//            visual_tools_->enableBatchPublishing();
//            visual_tools_->deleteAllMarkers();
//            visual_tools_->removeAllCollisionObjects();
//            visual_tools_->hideRobot();
//            visual_tools_->trigger();
//
//            // Publish the global frame
//            visual_tools_->publishAxis(Eigen::Isometry3d::Identity());
//            visual_tools_->trigger();
        }

        void setupGraspPipeline() {
            // ---------------------------------------------------------------------------------------------
            // Load grasp data specific to our robot
            grasp_data_.reset(new GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));

            // ---------------------------------------------------------------------------------------------
            // Load grasp generator
            grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));

            // Set the ideal grasp orientation for scoring
            std::vector<double> ideal_grasp_rpy = {3.14, 0.0, 0.0};
            grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

            // Set custom grasp score weights
            moveit_grasps::GraspScoreWeights grasp_score_weights;
            grasp_score_weights.orientation_x_score_weight_ = 2.0;
            grasp_score_weights.orientation_y_score_weight_ = 2.0;
            grasp_score_weights.orientation_z_score_weight_ = 2.0;
            grasp_score_weights.translation_x_score_weight_ = 1.0;
            grasp_score_weights.translation_y_score_weight_ = 1.0;
            grasp_score_weights.translation_z_score_weight_ = 1.0;
            // Finger gripper specific weights.
            // Note that we do not need to set the suction gripper specific weights for our finger gripper.
            grasp_score_weights.depth_score_weight_ = 2.0;
            grasp_score_weights.width_score_weight_ = 2.0;
            grasp_generator_->setGraspScoreWeights(grasp_score_weights);

            // ---------------------------------------------------------------------------------------------
            // Load grasp filter
            grasp_filter_.reset(new moveit_grasps::GraspFilter(visual_tools_->getSharedRobotState(), visual_tools_));

            // ---------------------------------------------------------------------------------------------
            // Load grasp planner for approach, lift and retreat planning
            grasp_planner_.reset(new moveit_grasps::GraspPlanner(visual_tools_));

            // MoveIt Grasps allows for a manual breakpoint debugging tool to be optionally passed in
            grasp_planner_->setWaitForNextStepCallback(boost::bind(&waitForNextStep, visual_tools_, _1));

            // -----------------------------------------------------
            // Load the motion planning pipeline
            planning_pipeline_.reset(
                    new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_"
                                                                                                  "adapter"));
        }

        bool
        demoRandomGrasp(std::string object_name,
                        geometry_msgs::Pose * robotMsgs) {
            // -----------------------------------
            // Generate random object to grasp

            std::vector <std::string> object_name_query;
            object_name_query.push_back(object_name);
            moveit::planning_interface::MoveGroupInterface move_group(planning_group_name_);
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            //TODO handle exception might not find the object
            ROS_ERROR_STREAM_NAMED(LOGNAME, "BEFORE MF TIME");
            ros::Duration(5).sleep();

            std::map <std::string, moveit_msgs::CollisionObject> planning_scene_objects = planning_scene_interface.getObjects(
                    object_name_query);
            moveit_msgs::CollisionObject object_msg = planning_scene_objects[object_name];
            geometry_msgs::Pose object_pose = object_msg.primitive_poses[0];

            double object_x_depth = object_msg.primitives[0].dimensions[0];
            double object_y_width = object_msg.primitives[0].dimensions[1];
            double object_z_height = object_msg.primitives[0].dimensions[2];

            //FIXME hacking this guy, for some reason is always gripping from the edge of the cube, so we center the pose
            //object_pose.position.x = object_pose.position.x - object_x_depth / 2;

            ROS_ERROR_STREAM_NAMED(LOGNAME, "STARTING SOLVER");

            setACMFingerEntry(object_name, true);

            // -----------------------------------
            // Generate grasp candidates
            std::vector <moveit_grasps::GraspCandidatePtr> grasp_candidates;

            // Configure the desired types of grasps
            moveit_grasps::GraspCandidateConfig grasp_generator_config = moveit_grasps::GraspCandidateConfig();
            grasp_generator_config.disableAll();
            grasp_generator_config.enable_face_grasps_ = true;
            grasp_generator_config.generate_y_axis_grasps_ = true;
            grasp_generator_config.generate_x_axis_grasps_ = true;
            grasp_generator_config.generate_z_axis_grasps_ = true;

            if (!grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), object_x_depth,
                                                  object_y_width,
                                                  object_z_height, grasp_data_, grasp_candidates,
                                                  grasp_generator_config)) {
                ROS_ERROR_NAMED(LOGNAME, "Grasp generator failed to generate any valid grasps");
                return false;
            }

            // --------------------------------------------
            // Generating a seed state for filtering grasps
            robot_state::RobotStatePtr seed_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
            Eigen::Isometry3d eef_mount_grasp_pose =
                    visual_tools_->convertPose(object_pose) * grasp_data_->tcp_to_eef_mount_.inverse();
            if (!getIKSolution(arm_jmg_, eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName())) {
                ROS_WARN_STREAM_NAMED(LOGNAME, "The ideal seed state is not reachable. Using start state as seed.");
            }

            // --------------------------------------------
            // Filtering grasps
            // Note: This step also solves for the grasp and pre-grasp states and stores them in grasp candidates)
            bool filter_pregrasps = true;
            if (!grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_, seed_state,
                                             filter_pregrasps)) {
                ROS_ERROR_STREAM_NAMED(LOGNAME, "Filter grasps failed");
                return false;
            }
            if (!grasp_filter_->removeInvalidAndFilter(grasp_candidates)) {
                ROS_WARN_NAMED(LOGNAME, "Grasp filtering removed all grasps");
                return false;
            }
            ROS_INFO_STREAM_NAMED(LOGNAME, "" << grasp_candidates.size() << " remain after filtering");

            // Plan free-space approach, cartesian approach, lift and retreat trajectories
            moveit_grasps::GraspCandidatePtr selected_grasp_candidate;
            moveit_msgs::MotionPlanResponse pre_approach_plan;
            if (!planFullGrasp(grasp_candidates, selected_grasp_candidate, pre_approach_plan)) {
                ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to plan grasp motions");
                return false;
            }

            ROS_ERROR_STREAM_NAMED(LOGNAME, "ENDING SOLVER");
            generatePickMsg(selected_grasp_candidate, robotMsgs);
            setACMFingerEntry(object_name, false);

            return true;
        }

        void generatePickMsg(const GraspCandidatePtr &valid_grasp_candidate,
                             geometry_msgs::Pose * robotStatePtr) {
            moveit_msgs::RobotState pre_grasp_msg;
            moveit_msgs::RobotState grasp_msg;

            geometry_msgs::PoseStamped pre_grasp_pose = GraspGenerator::getPreGraspPose(valid_grasp_candidate, valid_grasp_candidate->grasp_data_->parent_link_->getName());

//            robot_state::RobotStatePtr pre_grasp_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
//            valid_grasp_candidate->getPreGraspState(pre_grasp_state);
//            moveit::core::robotStateToRobotStateMsg(*pre_grasp_state, pre_grasp_msg);
//
//            robot_state::RobotStatePtr grasp_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
//            valid_grasp_candidate->getGraspStateClosed(grasp_state);
//            moveit::core::robotStateToRobotStateMsg(*grasp_state, grasp_msg);

            robotStatePtr[0] = pre_grasp_pose.pose;
            robotStatePtr[1] = valid_grasp_candidate->grasp_.grasp_pose.pose;
        }

        void visualizePick(const GraspCandidatePtr &valid_grasp_candidate,
                           const moveit_msgs::MotionPlanResponse &pre_approach_plan) {
            EigenSTL::vector_Isometry3d waypoints;
            GraspGenerator::getGraspWaypoints(valid_grasp_candidate, waypoints);

            // Visualize waypoints
            visual_tools_->publishAxisLabeled(waypoints[0], "pregrasp");
            visual_tools_->publishAxisLabeled(waypoints[1], "grasp");
            visual_tools_->publishAxisLabeled(waypoints[2], "lifted");
            visual_tools_->publishAxisLabeled(waypoints[3], "retreat");
            visual_tools_->trigger();

            moveit_msgs::RobotStatePtr pre_grasp_msg;

            robot_state::RobotStatePtr pre_grasp_state(
                    new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
            valid_grasp_candidate->getPreGraspState(pre_grasp_state);
            moveit::core::robotStateToRobotStateMsg(*pre_grasp_state, *pre_grasp_msg);


            visual_tools_->publishRobotState(pre_grasp_state, rviz_visual_tools::ORANGE);
            ROS_ERROR_STREAM_NAMED(LOGNAME, pre_grasp_state);

            robot_state::RobotStatePtr grasp_state(new robot_state::RobotState(*visual_tools_->getSharedRobotState()));
            valid_grasp_candidate->getGraspStateClosed(grasp_state);
            moveit_msgs::RobotStatePtr grasp_msg;
            moveit::core::robotStateToRobotStateMsg(*grasp_state, *grasp_msg);

            visual_tools_->publishRobotState(grasp_state, rviz_visual_tools::YELLOW);

            visual_tools_->publishRobotState(valid_grasp_candidate->segmented_cartesian_traj_[1].back(),
                                             rviz_visual_tools::BLUE);

            visual_tools_->publishRobotState(valid_grasp_candidate->segmented_cartesian_traj_[2].back(),
                                             rviz_visual_tools::PURPLE);

            //visual_tools_->prompt("show free space approach");
            visual_tools_->hideRobot();
            visual_tools_->trigger();

            bool wait_for_animation = false;
            visual_tools_->publishTrajectoryPath(pre_approach_plan.trajectory, pre_grasp_state, wait_for_animation);
           // ros::Duration(0.25).sleep();
            visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[APPROACH],
                                                 valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
           // ros::Duration(0.25).sleep();
            visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[LIFT],
                                                 valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
            visual_tools_->publishTrajectoryPath(valid_grasp_candidate->segmented_cartesian_traj_[RETREAT],
                                                 valid_grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
           // ros::Duration(0.25).sleep();
        }

        bool planFullGrasp(std::vector <moveit_grasps::GraspCandidatePtr> grasp_candidates,
                           moveit_grasps::GraspCandidatePtr &valid_grasp_candidate,
                           moveit_msgs::MotionPlanResponse &pre_approach_plan) {
            moveit::core::RobotStatePtr current_state;
            {
                boost::scoped_ptr <planning_scene_monitor::LockedPlanningSceneRW> ls(
                        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));
                current_state.reset(new moveit::core::RobotState((*ls)->getCurrentState()));
            }

            bool success = false;
            for (; !grasp_candidates.empty(); grasp_candidates.erase(grasp_candidates.begin())) {
                valid_grasp_candidate = grasp_candidates.front();
                valid_grasp_candidate->getPreGraspState(current_state);
                if (!grasp_planner_->planApproachLiftRetreat(valid_grasp_candidate, current_state,
                                                             planning_scene_monitor_,
                                                             false)) {
                    ROS_INFO_NAMED(LOGNAME, "failed to plan approach lift retreat");
                    continue;
                }

                robot_state::RobotStatePtr pre_grasp_state = valid_grasp_candidate->segmented_cartesian_traj_[APPROACH].front();
                if (!planPreApproach(*pre_grasp_state, pre_approach_plan)) {
                    ROS_WARN_NAMED(LOGNAME, "failed to plan to pregrasp_state");
                    continue;
                }

                success = true;
                break;
            }
            return success;
        }

        // this method seems to remove collision avoidance check between object name and robot grip.
        void setACMFingerEntry(const std::string &object_name, bool allowed) {
            planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);  // Lock planning scene

            // Get links of end effector
            const std::vector <std::string> &ee_links = grasp_data_->ee_jmg_->getLinkModelNames();

            // Set collision checking between fingers and object
            for (std::size_t i = 0; i < ee_links.size(); ++i) {
                scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_links[i], allowed);
            }
        }

        bool
        planPreApproach(const robot_state::RobotState &goal_state, moveit_msgs::MotionPlanResponse &pre_approach_plan) {
            planning_interface::MotionPlanRequest req;
            planning_interface::MotionPlanResponse res;

            double tolerance_above = 0.01;
            double tolerance_below = 0.01;
            // req.planner_id = "RRTConnectkConfigDefault";
            req.group_name = arm_jmg_->getName();
            req.num_planning_attempts = 5;
            req.allowed_planning_time = 1.5;
            moveit_msgs::Constraints goal =
                    kinematic_constraints::constructGoalConstraints(goal_state, arm_jmg_, tolerance_below,
                                                                    tolerance_above);

            req.goal_constraints.push_back(goal);
            boost::scoped_ptr <planning_scene_monitor::LockedPlanningSceneRW> ls(
                    new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

            // ---------------------------------
            // Change the robot current state
            // NOTE: We have to do this since Panda start configuration is in self collision.
            robot_state::RobotState rs = (*ls)->getCurrentState();
            // TODO get this real states, leave this for last, maybe is not needed.
            std::vector<double> starting_joint_values = {};
            std::vector <std::string> joint_names = {"edo_joint_1", "edo_joint_2", "edo_joint_3", "edo_joint_4",
                                                     "edo_joint_5", "edo_joint_6"};

            for (std::size_t i = 0; i < joint_names.size(); ++i) {
                double value = (double) *rs.getJointPositions(joint_names[i]);
                starting_joint_values.push_back(value);
            }

            // arm_jmg_->getActiveJointModelNames();
            for (std::size_t i = 0; i < joint_names.size(); ++i) {
                rs.setJointPositions(joint_names[i], &starting_joint_values[i]);
            }
            rs.update();
            robot_state::robotStateToRobotStateMsg(rs, req.start_state);
            // ---------------------------

            planning_pipeline_->generatePlan(*ls, req, res);
            if (res.error_code_.val != res.error_code_.SUCCESS) {
                ROS_INFO_NAMED(LOGNAME, "Failed to plan approach successfully");
                return false;
            }

            res.getMessage(pre_approach_plan);
            return true;
        }

        bool getIKSolution(const moveit::core::JointModelGroup *arm_jmg, const Eigen::Isometry3d &target_pose,
                           robot_state::RobotState &solution, const std::string &link_name) {
            boost::scoped_ptr <planning_scene_monitor::LockedPlanningSceneRW> ls(
                    new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

            moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
                    &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls).get(), visual_tools_,
                    _1, _2, _3);

            // seed IK call with current state
            solution = (*ls)->getCurrentState();

            // Solve IK problem for arm
            // disable explicit restarts to guarantee close solution if one exists
            const double timeout = 0.1;
            return solution.setFromIK(arm_jmg, target_pose, link_name, timeout, constraint_fn);
        }

        bool generateRandomCuboid(std::string &object_name, geometry_msgs::Pose &object_pose, double &x_depth,
                                  double &y_width, double &z_height) {
            // Generate random cuboid
            double xmin = 0.45;
            double xmax = 0.45;
            double ymin = 0.0;
            double ymax = 0.0;
            double zmin = 0.025;
            double zmax = 0.025;
            rviz_visual_tools::RandomPoseBounds pose_bounds(xmin, xmax, ymin, ymax, zmin, zmax);

            double cuboid_size_min = 0.05;
            double cuboid_size_max = 0.05;
            rviz_visual_tools::RandomCuboidBounds cuboid_bounds(cuboid_size_min, cuboid_size_max);

            object_name = "pick_target";
            visual_tools_->generateRandomCuboid(object_pose, x_depth, y_width, z_height, pose_bounds, cuboid_bounds);
            visual_tools_->publishCollisionCuboid(object_pose, x_depth, y_width, z_height, object_name,
                                                  rviz_visual_tools::RED);
            visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
            visual_tools_->trigger();

            bool success = true;
            double timeout = 5;  // seconds
            ros::Rate rate(100);
            while (success && !planning_scene_monitor_->getPlanningScene()->knowsFrameTransform(object_name)) {
                rate.sleep();
                success = rate.cycleTime().toSec() < timeout;
            }
            return success;
        }

    private:
        // A shared node handle
        ros::NodeHandle nh_;

        // Tool for visualizing things in Rviz
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        // MoveIt! Grasps
        GraspGeneratorPtr grasp_generator_;

        // Robot-specific data for generating grasps
        GraspDataPtr grasp_data_;

        // For planning approach and retreats
        GraspPlannerPtr grasp_planner_;

        // For selecting good grasps
        GraspFilterPtr grasp_filter_;

        // Shared planning scene (load once for everything)
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        // Arm
        const robot_model::JointModelGroup *arm_jmg_;

        // Robot
        robot_model::RobotModelPtr robot_model_;

        // All the motion planning components
        planning_pipeline::PlanningPipelinePtr planning_pipeline_;

        // Choose which arm to use
        std::string ee_group_name_;
        std::string planning_group_name_;

    };  // end of class

}  // namespace

extern moveit_grasps::GraspPipelineDemo *singleTone = NULL;

bool GenerateGrasp(moveit_grasps::Grasp::Request &req,
                   moveit_grasps::Grasp::Response &res) {
    geometry_msgs::Pose grasp_msgs[2];

    singleTone->demoRandomGrasp(req.object_name, grasp_msgs);
    res.pre_grasp = grasp_msgs[0];
    res.grasp = grasp_msgs[1];
    return true;
}


//   moveit_grasps::GraspPipelineDemo moveit_grasps::GraspPipelineDemo::singleTone = moveit_grasps::GraspPipelineDemo();


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "grasp_pipeline_server");
    ros::NodeHandle nh;

    moveit_grasps::GraspPipelineDemo pipeline;
    singleTone = &pipeline;
    // Create Pipeline object

    ros::ServiceServer service = nh.advertiseService("grasp_pipeline", GenerateGrasp);

    // spin the server
    ros::spin();

    return 0;
}
