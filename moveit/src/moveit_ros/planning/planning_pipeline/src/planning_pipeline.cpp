/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/tokenizer.hpp>
#include <sstream>

#include <Eigen/Core>

#include <moveit/kinematic_constraints/utils.h>
#include <map>
#include <algorithm>

const std::string planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC = "display_planned_path";
const std::string planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC = "motion_plan_request";
const std::string planning_pipeline::PlanningPipeline::MOTION_CONTACTS_TOPIC = "display_contacts";

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model, 
                                                      const std::string &planner_plugin_param_name,
                                                      const std::string &adapter_plugins_param_name) :
  nh_("~"),
  kmodel_(model)
{
  std::string planner;
  if (nh_.getParam(planner_plugin_param_name, planner))
    planner_plugin_name_ = planner;
  
  std::string adapters;
  if (nh_.getParam(adapter_plugins_param_name, adapters))
  { 
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
    for(boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin() ; beg != tok.end(); ++beg)
      adapter_plugin_names_.push_back(*beg);
  }
  
  configure();
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model, 
                                                      const std::string &planner_plugin_name,
                                                      const std::vector<std::string> &adapter_plugin_names) :
  nh_("~"),
  planner_plugin_name_(planner_plugin_name),
  adapter_plugin_names_(adapter_plugin_names),
  kmodel_(model)
{
  configure();
}

void planning_pipeline::PlanningPipeline::configure()
{
  check_solution_paths_ = false;          // this is set to true below
  publish_received_requests_ = false;
  display_computed_motion_plans_ = false; // this is set to true below
  
  // load the planning plugin
  try
  {
    planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::Planner>("moveit_core", "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  const std::vector<std::string> &classes = planner_plugin_loader_->getDeclaredClasses();
  if (planner_plugin_name_.empty() && classes.size() == 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("No '~planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.", planner_plugin_name_.c_str());
  }
  if (planner_plugin_name_.empty() && classes.size() > 1)
  {      
    planner_plugin_name_ = classes[0];   
    ROS_INFO("Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using '%s' for now.", planner_plugin_name_.c_str());
  }
  try
  {
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
    if (!planner_instance_->initialize(kmodel_))
      throw std::runtime_error("Unable to initialize planning plugin");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name_ << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }
  
  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {   
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    try
    {
      adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>("moveit_core", "planning_request_adapter::PlanningRequestAdapter"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    
    for (std::size_t i = 0 ; i < adapter_plugin_names_.size() ; ++i)
    {
      planning_request_adapter::PlanningRequestAdapterConstPtr ad;
      try
      {
        ad.reset(adapter_plugin_loader_->createUnmanagedInstance(adapter_plugin_names_[i]));
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("Exception while loading planning adapter plugin '" << adapter_plugin_names_[i] << "': " << ex.what());
      }
      if (ad)
        ads.push_back(ad);
    }
    if (!ads.empty())
    {
      adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
      for (std::size_t i = 0 ; i < ads.size() ; ++i)
      {
        ROS_INFO_STREAM("Using planning request adapter '" << ads[i]->getDescription() << "'");
        adapter_chain_->addAdapter(ads[i]);
      }
    }
  }
  displayComputedMotionPlans(true);
  checkSolutionPaths(true);
}

void planning_pipeline::PlanningPipeline::displayComputedMotionPlans(bool flag)
{
  if (display_computed_motion_plans_ && !flag)
    display_path_publisher_.shutdown();
  else
    if (!display_computed_motion_plans_ && flag)
      display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PATH_TOPIC, 10, true);
  display_computed_motion_plans_ = flag;
}

void planning_pipeline::PlanningPipeline::publishReceivedRequests(bool flag)
{
  if (publish_received_requests_ && !flag)
    received_request_publisher_.shutdown();
  else
    if (!publish_received_requests_ && flag)
      received_request_publisher_ = nh_.advertise<moveit_msgs::MotionPlanRequest>(MOTION_PLAN_REQUEST_TOPIC, 10, true);
  publish_received_requests_ = flag;
}

void planning_pipeline::PlanningPipeline::checkSolutionPaths(bool flag)
{ 
  if (check_solution_paths_ && !flag)
    contacts_publisher_.shutdown();
  else
    if (!check_solution_paths_ && flag)
      contacts_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(MOTION_CONTACTS_TOPIC, 100, true);
  check_solution_paths_ = flag;
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return generatePlan(planning_scene, req, res, dummy);
}

void getAlignedGrip(double incidenceAngle, double distance, bool flipGrip, Eigen::Affine3d const& refPose, Eigen::Affine3d & gripPose, Eigen::Affine3d & puzzleToGripper)
{
    Eigen::Vector3d eef_pO, eef_p, eefx_p, eefy_p, eefz_p;
    eef_pO[0] = (flipGrip)?(-distance):(distance);
    eef_pO[1] = 0.075;
    eef_pO[2] = 0;

    double gA = incidenceAngle;
    eef_p[0] = eef_pO[0];
    eef_p[1] = eef_pO[1]*std::cos(gA) + eef_pO[2]*std::sin(gA);
    eef_p[2] = -eef_pO[1]*std::sin(gA) + eef_pO[2]*std::cos(gA);

    eefx_p[0] = (flipGrip)?(1):(-1);
    eefx_p[1] = 0;
    eefx_p[2] = 0;
    {
      double n = std::sqrt(eefx_p[0]*eefx_p[0] + eefx_p[1]*eefx_p[1] + eefx_p[2]*eefx_p[2]);
      eefx_p[0] = eefx_p[0]/n;
      eefx_p[1] = eefx_p[1]/n;
      eefx_p[2] = eefx_p[2]/n;
    }

    eefy_p[0] = 0;
    eefy_p[1] = std::cos(gA);
    eefy_p[2] = -std::sin(gA);

    eefz_p[0] = eefx_p[1]*eefy_p[2] - eefx_p[2]*eefy_p[1];
    eefz_p[1] = eefx_p[2]*eefy_p[0] - eefx_p[0]*eefy_p[2];
    eefz_p[2] = eefx_p[0]*eefy_p[1] - eefx_p[1]*eefy_p[0];

    Eigen::Affine3d eefPose_p;
    eefPose_p(0, 0) = eefx_p[0]; eefPose_p(0, 1) = eefy_p[0]; eefPose_p(0, 2) = eefz_p[0]; eefPose_p(0, 3) = eef_p[0];
    eefPose_p(1, 0) = eefx_p[1]; eefPose_p(1, 1) = eefy_p[1]; eefPose_p(1, 2) = eefz_p[1]; eefPose_p(1, 3) = eef_p[1];
    eefPose_p(2, 0) = eefx_p[2]; eefPose_p(2, 1) = eefy_p[2]; eefPose_p(2, 2) = eefz_p[2]; eefPose_p(2, 3) = eef_p[2];
    eefPose_p(3, 0) =     0    ; eefPose_p(3, 1) =     0    ; eefPose_p(3, 2) =     0    ; eefPose_p(3, 3) =    1    ;
    gripPose = refPose*eefPose_p;
    puzzleToGripper = eefPose_p;
}
void getSideGrip(double incidenceAngle, double distance, bool flipGrip, Eigen::Affine3d const& refPose, Eigen::Affine3d & gripPose, Eigen::Affine3d & puzzleToGripper)
{
    Eigen::Vector3d eef_pO, eef_p, eef_pR, eefx_p, eefy_p, eefz_p;
    eef_pO[0] = 0;
    eef_pO[1] = distance;
    eef_pO[2] = 0;

    double gA = incidenceAngle;
    eef_p[0] = eef_pO[0];
    eef_p[1] = eef_pO[1]*std::cos(gA) + eef_pO[2]*std::sin(gA);
    eef_p[2] = -eef_pO[1]*std::sin(gA) + eef_pO[2]*std::cos(gA);

    eefx_p[0] = eef_p[0];
    eefx_p[1] = -eef_p[1];
    eefx_p[2] = -eef_p[2];
    {
      double n = std::sqrt(eefx_p[0]*eefx_p[0] + eefx_p[1]*eefx_p[1] + eefx_p[2]*eefx_p[2]);
      eefx_p[0] = eefx_p[0]/n;
      eefx_p[1] = eefx_p[1]/n;
      eefx_p[2] = eefx_p[2]/n;
    }

    eefy_p[0] = (flipGrip)?(1):(-1);
    eefy_p[1] = 0;
    eefy_p[2] = 0;

    eefz_p[0] = eefx_p[1]*eefy_p[2] - eefx_p[2]*eefy_p[1];
    eefz_p[1] = eefx_p[2]*eefy_p[0] - eefx_p[0]*eefy_p[2];
    eefz_p[2] = eefx_p[0]*eefy_p[1] - eefx_p[1]*eefy_p[0];

    Eigen::Affine3d eefPose_p;
    eefPose_p(0, 0) = eefx_p[0]; eefPose_p(0, 1) = eefy_p[0]; eefPose_p(0, 2) = eefz_p[0]; eefPose_p(0, 3) = eef_p[0];
    eefPose_p(1, 0) = eefx_p[1]; eefPose_p(1, 1) = eefy_p[1]; eefPose_p(1, 2) = eefz_p[1]; eefPose_p(1, 3) = eef_p[1];
    eefPose_p(2, 0) = eefx_p[2]; eefPose_p(2, 1) = eefy_p[2]; eefPose_p(2, 2) = eefz_p[2]; eefPose_p(2, 3) = eef_p[2];
    eefPose_p(3, 0) =     0    ; eefPose_p(3, 1) =     0    ; eefPose_p(3, 2) =     0    ; eefPose_p(3, 3) =    1    ;
    gripPose = refPose*eefPose_p;
    puzzleToGripper = eefPose_p;
}

namespace planning_pipeline
{
  bool bring_arm(std::string armName, robot_state::RobotState &gstate, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res);
  bool tryGrip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint, robot_state::RobotState &gstate, robot_state::RobotState &pstate, Eigen::Affine3d &gripPose, Eigen::Affine3d &closePuzzleToGripper, const planning_scene::PlanningSceneConstPtr& planning_scene, robot_state::RobotState const& wstate, bool flipped, bool aligned, double incidenceAngle, double closeDist, double farDist, const robot_model::RobotModelConstPtr &kmodel);
  bool selectGrip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint, robot_state::RobotState &gstate, robot_state::RobotState &pstate, Eigen::Affine3d &gripPose, Eigen::Affine3d &closePuzzleToGripper, const planning_scene::PlanningSceneConstPtr& planning_scene, robot_state::RobotState const& wstate, const robot_model::RobotModelConstPtr &kmodel);
  bool arm_away(std::string const& armName, std::string const& awayName, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res);
  bool followPuzzle(std::string const& armName, std::string const& lastArmLink, Eigen::Affine3d const& closePuzzleToGripper, robot_state::RobotState const& pstate, robot_state::RobotState &wstate, const planning_scene::PlanningSceneConstPtr& planning_scene, planning_interface::MotionPlanResponse& res);
  bool grip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint, robot_state::RobotState &wstate, robot_state::RobotState &pstate, Eigen::Affine3d &closePuzzleToGripper, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, const robot_model::RobotModelConstPtr &kmodel);
  bool ungrip(std::string const& armName, std::string const& gripperName, std::string const& gripperJoint, std::string const& awayName, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res);
  void switchArms(std::string &crArm, std::string &crGripper, std::string &crGripperJoint, std::string &crArmLastLink, std::string &crArmAway);
  bool trySwitchingGrip(std::string &crArm, std::string &crGripper, std::string &crGripperJoint, std::string &crArmLastLink, std::string &crArmAway, robot_state::RobotState &wstate, robot_state::RobotState &pstate, Eigen::Affine3d &closePuzzleToGripper,
                                          boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain,
                                          planning_interface::PlannerPtr const& planner_instance,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res,
                                          std::vector<std::size_t> &adapter_added_state_index,
                                          const robot_model::RobotModelConstPtr &kmodel);
}

bool planning_pipeline::bring_arm(std::string armName, robot_state::RobotState &gstate, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
  std::cout << "ACTION: plan for arm " << armName << std::endl;
  robot_state::JointStateGroup *garm = gstate.getJointStateGroup(armName);
  robot_state::JointStateGroup *warm = wstate.getJointStateGroup(armName);
  planning_interface::MotionPlanRequest newReq;
  planning_interface::MotionPlanResponse newRes;
  newReq.group_name = armName;
  newReq.num_planning_attempts = 1;
  newReq.allowed_planning_time = 5;
  robot_state::robotStateToRobotStateMsg(wstate, newReq.start_state);
  newReq.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.x;
  newReq.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.y;
  newReq.workspace_parameters.min_corner.z = req.workspace_parameters.min_corner.z;
  newReq.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.x;
  newReq.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.y;
  newReq.workspace_parameters.max_corner.z = req.workspace_parameters.max_corner.z;
  newReq.goal_constraints.resize(1);
  newReq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(garm);
  bool solved = (adapter_chain)?(adapter_chain->adaptAndPlan(planner_instance, planning_scene, newReq, newRes, adapter_added_state_index)):(planner_instance->solve(planning_scene, newReq, newRes));
  unsigned int kMax = newRes.trajectory_->getWayPointCount();
  for(unsigned int k = 0; k < kMax; k++)
  {
    robot_state::RobotState tstate(newRes.trajectory_->getWayPoint(k));
    robot_state::JointStateGroup *tarm = tstate.getJointStateGroup(armName);
    std::vector<double> jointValues;
    tarm->getVariableValues(jointValues);
    warm->setVariableValues(jointValues);
    res.trajectory_->addSuffixWayPoint(wstate, 0.01);
  }
  return solved;
}

bool planning_pipeline::tryGrip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint, robot_state::RobotState &gstate, robot_state::RobotState &pstate, Eigen::Affine3d &gripPose, Eigen::Affine3d &closePuzzleToGripper, const planning_scene::PlanningSceneConstPtr& planning_scene, robot_state::RobotState const& wstate, bool flipped, bool aligned, double incidenceAngle, double closeDist, double farDist, const robot_model::RobotModelConstPtr &kmodel)
{
  std::cout << "  ACTION: try grip for arm " << armName << " with grip parameters ";
  (aligned)?(std::cout << "aligned "):(std::cout << "side ");
  (flipped)?(std::cout << "flipped "):(std::cout << "regular ");
  std::cout << "grip of " << incidenceAngle << " incidenceAngle." << std::endl;
  const double gN[] = {0.10, 0.15, 0.20, 0.25, 0.30};
  robot_state::RobotState tstateClose(wstate), tstateFar(wstate);
  robot_state::JointStateGroup *armClose = tstateClose.getJointStateGroup(armName);
  robot_state::JointStateGroup *armFar = tstateFar.getJointStateGroup(armName);
  Eigen::Affine3d closePose, farPose, puzzlePose, dummyPose;
  puzzlePose = wstate.getLinkState("ring_link")->getGlobalCollisionBodyTransform();
  if(aligned)
  {
    getAlignedGrip(incidenceAngle, closeDist, flipped, puzzlePose, closePose, closePuzzleToGripper);
    getAlignedGrip(incidenceAngle, farDist, flipped, puzzlePose, farPose, dummyPose);
  }
  else
  {
    getSideGrip(incidenceAngle, closeDist, flipped, puzzlePose, closePose, closePuzzleToGripper);
    getSideGrip(incidenceAngle, farDist, flipped, puzzlePose, farPose, dummyPose);
  }
  bool closeFound = armClose->setFromIK(closePose);
  if(!closeFound)
  {
    //std::cout << " IK fails to find solution for grip." << std::endl;
  }
  bool farFound = armFar->setFromIK(farPose);
  if(!farFound)
  {
    //std::cout << " IK fails to find solution for approach." << std::endl;
  }
  if(closeFound)
  {
    closeFound = planning_scene->isStateValid(tstateClose, "puzzle_bot", false);
    if(!closeFound)
    {
      //std::cout << " grip fail." << std::endl;
      planning_scene->isStateValid(tstateClose, "puzzle_bot", false);
    }
  }
  if(farFound)
  {
    farFound = planning_scene->isStateValid(tstateFar, "puzzle_bot", false);
    if(!farFound)
    {
      //std::cout << " gripper approach fail." << std::endl;
      planning_scene->isStateValid(tstateFar, "puzzle_bot", false);
    }
  }
  if(closeFound && farFound)
  {
    std::vector<robot_state::RobotStatePtr> traj;
    double completion = armFar->computeCartesianPath(traj, lastArmLink, closePose, true, 0.005, 1.8);
    std::cout << " completion at " << completion << std::endl;
    if(0.9 > completion)
    {
      //std::cout << " cartesian path jump." << std::endl;
      return false;
    }
    unsigned int maxK = traj.size();
    bool allValid = true;
    for(unsigned int k = 0; (k < maxK) && (allValid); k++)
    {
      allValid = planning_scene->isStateValid(*(traj[k].get()), "puzzle_bot", false);
      if(!allValid)
      {
        //std::cout << " cartesian path contains invalid state." << std::endl;
        return false;
      }
    }
    robot_state::JointStateGroup *gripper = tstateClose.getJointStateGroup(gripperName);
    std::map<std::string, double> gripperJointValues;
    gripper->getVariableValues(gripperJointValues);
    for(unsigned int k = 0; (k <sizeof(gN)/sizeof(gN[0])) && (allValid); k++)
    {
      gripperJointValues[gripperJoint] = gN[k];
      gripper->setVariableValues(gripperJointValues);
      allValid = planning_scene->isStateValid(tstateClose, "puzzle_bot", false);
      if(!allValid)
      {
        //std::cout << " gripper open/close contains invalid state." << std::endl;
        return false;
      }
    }
    if(allValid)
    {
      //std::cout << "  setting gstate and gripPose" << std::endl;
      planning_interface::MotionPlanResponse res2;
      res2.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel, "puzzle_bot"));
      res2.trajectory_->clear();
      res2.planning_time_ = 0.01;
      res2.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      if(planning_pipeline::followPuzzle(armName, lastArmLink, closePuzzleToGripper, pstate, tstateClose, planning_scene, res2))
      {
        gstate = tstateFar;
        gripPose = closePose;
      }
      else
      {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool planning_pipeline::selectGrip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint,  robot_state::RobotState &gstate, robot_state::RobotState &pstate, Eigen::Affine3d &gripPose, Eigen::Affine3d &closePuzzleToGripper, const planning_scene::PlanningSceneConstPtr& planning_scene, robot_state::RobotState const& wstate, const robot_model::RobotModelConstPtr &kmodel)
{
  const double pi = 3.14159;
  const double a[] = {pi/6.0, pi/2.0, 5*pi/6.0, 7*pi/6.0, 9*pi/6.0, 11*pi/6.0};
  const double dC[] = {0.265, 0.185};
  const double dF[] = {0.355, 0.280};//0.305, 0.230
  const bool flipped[] =          {false, false, false, false, false, false,  true,  true,  true,  true,  true,  true, false, false, false, false, false, false,  true,  true,  true,  true,  true,  true};
  const bool aligned[] =          {false, false, false, false, false, false, false, false, false, false, false, false,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true};
  const double incidenceAngle[] = { a[0],  a[1],  a[2],  a[3],  a[4],  a[5],  a[0],  a[1],  a[2],  a[3],  a[4],  a[5],  a[0],  a[1],  a[2],  a[3],  a[4],  a[5],  a[0],  a[1],  a[2],  a[3],  a[4],  a[5]};
  const double closeDist[] =      {dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[0], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1], dC[1]};
  const double farDist[] =        {dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[0], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1], dF[1]};
  const unsigned int idx[] =      {    0,     1,     2,     3,     4,     5,     6,     7,     8,     9,    10,    11,    12,    13,    14,    15,    16,    17,    18,    19,    20,    21,    22,    23};
  std::vector<unsigned int> idxV(idx, idx + sizeof(idx) / sizeof(idx[0]) );
  std::random_shuffle(idxV.begin(), idxV.end());
  bool foundGrip = false;
  std::cout << "ACTION: select grip for arm " << armName << std::endl;
  for(unsigned int k = 0; (k < sizeof(flipped)/sizeof(flipped[0])) && (!foundGrip); k++)
  {
    unsigned int j = idxV[k];
    foundGrip = tryGrip(armName, lastArmLink, gripperName, gripperJoint, gstate, pstate, gripPose, closePuzzleToGripper, planning_scene, wstate, flipped[j], aligned[j], incidenceAngle[j], closeDist[j], farDist[j], kmodel);
  }
  return foundGrip;
}

bool planning_pipeline::arm_away(std::string const& armName, std::string const& awayName, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
  robot_state::RobotState gstate(wstate);
  std::string armWithGripper = armName + "_with_gripper";
  std::cout << "ACTION: arm away for " << armWithGripper << std::endl;
  robot_state::JointStateGroup *garm = gstate.getJointStateGroup(armWithGripper);
  garm->setToDefaultState(awayName);
  return bring_arm(armWithGripper, gstate, wstate, adapter_chain, planner_instance, adapter_added_state_index, planning_scene, req, res);
}

bool planning_pipeline::grip(std::string const& armName, std::string const& lastArmLink, std::string const& gripperName, std::string const& gripperJoint, robot_state::RobotState &wstate, robot_state::RobotState &pstate, Eigen::Affine3d &closePuzzleToGripper, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, const robot_model::RobotModelConstPtr &kmodel)
{
  std::cout << "ACTION: grip for arm " << armName << std::endl;
  robot_state::RobotState gstate(wstate);
  Eigen::Affine3d gripPose;
  if(!selectGrip(armName, lastArmLink, gripperName, gripperJoint, gstate, pstate, gripPose, closePuzzleToGripper, planning_scene, wstate, kmodel))
  {
    std::cout << " select grip failed." << std::endl;
    return false;
  }
  if(!bring_arm(armName, gstate, wstate, adapter_chain, planner_instance, adapter_added_state_index, planning_scene, req, res))
  {
    std::cout << " bring arm failed." << std::endl;
    return false;
  }
  robot_state::JointStateGroup *arm = wstate.getJointStateGroup(armName);
  std::vector<double> gripJointValues;
  gstate.getJointStateGroup(armName)->getVariableValues(gripJointValues);
  arm->setVariableValues(gripJointValues);
  res.trajectory_->addSuffixWayPoint(gstate, 0.01);
  std::vector<robot_state::RobotStatePtr> traj;
  if(0.9 > arm->computeCartesianPath(traj, lastArmLink, gripPose, true, 0.005, 2.0))
  {
    std::cout << " cartesian path contains jump." << std::endl;
    return false;
  }
  unsigned int maxK = traj.size();
  for(unsigned int k = 0; k < maxK; k++)
  {
    robot_state::JointStateGroup *tarm = traj[k]->getJointStateGroup(armName);
    std::vector<double> jointValues;
    tarm->getVariableValues(jointValues);
    if(!planning_scene->isStateValid(*(traj[k].get()), "puzzle_bot", false))
    {
      std::cout << "  Failing approach to grip." << std::endl;
      return false;
    }
    arm->setVariableValues(jointValues);
    res.trajectory_->addSuffixWayPoint(wstate, 0.01);
  }
  robot_state::JointStateGroup *gripper = wstate.getJointStateGroup(gripperName);
  std::map<std::string, double> gripperJointValues, gripperJointValues_bk;
  gripper->getVariableValues(gripperJointValues);
  double nums[] = {0.25, 0.20, 0.15, 0.10};
  std::vector<double> moveVals(nums, nums + sizeof(nums) / sizeof(nums[0]) );
  for(unsigned int k; k < moveVals.size(); k++)
  {
    gripper->getVariableValues(gripperJointValues_bk);
    gripperJointValues[gripperJoint] = moveVals[k];
    gripper->setVariableValues(gripperJointValues);
    if(!planning_scene->isStateValid(wstate, "puzzle_bot", false))
    {
      gripper->setVariableValues(gripperJointValues_bk);
      std::cout << "  Failing to close grip." << std::endl;
      return false;
    }
    res.trajectory_->addSuffixWayPoint(wstate, 0.01);
  }
  return true;
}

bool planning_pipeline::ungrip(std::string const& armName, std::string const& gripperName, std::string const& gripperJoint, std::string const& awayName, robot_state::RobotState &wstate, boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain, planning_interface::PlannerPtr const& planner_instance, std::vector<std::size_t> &adapter_added_state_index, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
  std::cout << "ACTION: ungrip for arm " << armName << std::endl;

  robot_state::JointStateGroup *gripper = wstate.getJointStateGroup(gripperName);
  std::map<std::string, double> gripperJointValues;
  gripper->getVariableValues(gripperJointValues);
  if(gripperJointValues[gripperJoint] < 0.30)
  {
    //double nums[] = {0.15, 0.20, 0.25, 0.30};
    double nums[] = {0.15};
    std::vector<double> moveVals(nums, nums + sizeof(nums) / sizeof(nums[0]) );
    for(unsigned int k; k < moveVals.size(); k++)
    {
      gripperJointValues[gripperJoint] = moveVals[k];
      gripper->setVariableValues(gripperJointValues);
      if(!planning_scene->isStateValid(wstate, "puzzle_bot", false))
      {
        return false;
      }
      res.trajectory_->addSuffixWayPoint(wstate, 0.01);
    }
  }
  return arm_away(armName, awayName, wstate, adapter_chain, planner_instance, adapter_added_state_index, planning_scene, req, res);
}

bool planning_pipeline::followPuzzle(std::string const& armName, std::string const& lastArmLink, Eigen::Affine3d const& closePuzzleToGripper, robot_state::RobotState const& pstate, robot_state::RobotState &wstate, const planning_scene::PlanningSceneConstPtr& planning_scene, planning_interface::MotionPlanResponse& res)
{
  std::cout << "ACTION: follow puzzle for arm " << armName << std::endl;
  Eigen::Affine3d puzzlePose = pstate.getLinkState("ring_link")->getGlobalCollisionBodyTransform();
  Eigen::Affine3d gripperPose = puzzlePose*closePuzzleToGripper;
  robot_state::JointStateGroup *arm = wstate.getJointStateGroup(armName);
  std::vector<double> jointVariables, newJointVariables;
  arm->getVariableValues(jointVariables);
  std::vector<double> puzzleValues, puzzleValues_bk;
  pstate.getJointStateGroup("puzzle")->getVariableValues(puzzleValues);
  wstate.getJointStateGroup("puzzle")->getVariableValues(puzzleValues_bk);
  if(!arm->setFromIK(gripperPose))
  {
    std::cout << "  IK fail while follow." << std::endl;
    return false;
  }
  arm->getVariableValues(newJointVariables);
  double dist = 0.0;
  for(unsigned int k = 0; k < newJointVariables.size(); k++)
  {
    double d = newJointVariables[k] - jointVariables[k];
    dist += d*d;
  }
  dist = std::sqrt(dist);
  double pdist = 0.0;
  for(unsigned int k = 0; k < puzzleValues_bk.size(); k++)
  {
    double d = puzzleValues[k] - puzzleValues_bk[k];
    pdist += d*d;
  }
  pdist = std::sqrt(pdist);
  if(20.0*pdist < dist)
  {
    arm->setVariableValues(jointVariables);
    std::cout << "  IK produces jump while follow: " << pdist << " " << dist << std::endl;
    return false;
  }
  wstate.getJointStateGroup("puzzle")->setVariableValues(puzzleValues);
  if(!planning_scene->isStateValid(wstate, "puzzle_bot", false))
  {
    arm->setVariableValues(jointVariables);
    wstate.getJointStateGroup("puzzle")->setVariableValues(puzzleValues_bk);
    std::cout << "  Follow produces a collision." << std::endl;
    return false;
  }
  res.trajectory_->addSuffixWayPoint(wstate, 0.01);
  return true;
}

void planning_pipeline::switchArms(std::string &crArm, std::string &crGripper, std::string &crGripperJoint, std::string &crArmLastLink, std::string &crArmAway)
{
  if("right_arm" == crArm)
  {
    crArm = "left_arm";
    crGripper = "left_gripper";
    crGripperJoint = "l_gripper_l_finger_joint";
    crArmLastLink = "l_wrist_roll_link";
    crArmAway = "left_arm_away";
  }
  else
  {
    crArm = "right_arm";
    crGripper = "right_gripper";
    crGripperJoint = "r_gripper_l_finger_joint";
    crArmLastLink = "r_wrist_roll_link";
    crArmAway = "right_arm_away";
  }
}

bool planning_pipeline::trySwitchingGrip(std::string &crArm, std::string &crGripper, std::string &crGripperJoint, std::string &crArmLastLink, std::string &crArmAway, robot_state::RobotState &wstate, robot_state::RobotState &pstate, Eigen::Affine3d &closePuzzleToGripper,
                                          boost::scoped_ptr<planning_request_adapter::PlanningRequestAdapterChain> const& adapter_chain,
                                          planning_interface::PlannerPtr const& planner_instance,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res,
                                          std::vector<std::size_t> &adapter_added_state_index,
                                          const robot_model::RobotModelConstPtr &kmodel)
{
  std::cout << "ACTION: try switching grips" << std::endl;
  robot_state::RobotState tstate(wstate);
  //Example run: crArm starts at right_arm; on success, crArm must be left_arm; on failure, must be right_arm
  //start: crArm at right_arm
  planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
  //crArm now left_arm
  planning_interface::MotionPlanResponse res2;
  res2.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel, "puzzle_bot"));
  res2.trajectory_->clear();
  res2.planning_time_ = 0.01;
  res2.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  Eigen::Affine3d cPtG;
  bool canGrip = planning_pipeline::grip(crArm, crArmLastLink, crGripper, crGripperJoint, tstate, pstate, cPtG, adapter_chain, planner_instance, adapter_added_state_index, planning_scene, req, res2, kmodel);
  if(!canGrip)
  {
    planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
    //crArm now right_arm
    std::cout << "Failed to switch grips: cannot bring other arm close." << std::endl;
    return false;
  }
  else
  {
    planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
    //crArm now right_arm
    bool canUngrip = planning_pipeline::ungrip(crArm, crGripper, crGripperJoint, crArmAway, tstate, adapter_chain, planner_instance, adapter_added_state_index, planning_scene, req, res2);
    if(!canUngrip)
    {
      std::cout << "Failed to switch grips: cannot remove previous grip." << std::endl;
      return false;
    }
    closePuzzleToGripper = cPtG;
    res.trajectory_->append(*(res2.trajectory_.get()), 0.01);
    wstate = tstate;
    planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
    //crArm now left_arm
  }
  return true;
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       std::vector<std::size_t> &adapter_added_state_index) const
{
  // broadcast the request we are about to work on, if needed
  if (publish_received_requests_)
    received_request_publisher_.publish(req);
  adapter_added_state_index.clear();
  ROS_INFO("planning_pipeline::PlanningPipeline::generatePlan called ... ");
  std::cout << "    -planning for group name " << req.group_name << std::endl;

  planning_interface::MotionPlanRequest dummyReq;
  //dummyReq.allowed_planning_time;

  std::cout << "Received planning request: " << std::endl;
  std::cout << "    allowed_planning_time = " << req.allowed_planning_time << std::endl;
  std::cout << "    group_name = " << req.group_name << std::endl;
  std::cout << "    num_planning_attempts = " << req.num_planning_attempts << std::endl;
  std::cout << "    planner_id = " << req.planner_id << std::endl;
  std::cout << "    goal_constraints.size() = " << req.goal_constraints.size() << std::endl;
  std::cout << "    goal_constraints[0].name = " << req.goal_constraints[0].name << std::endl;
  std::cout << "    goal_constraints[0].joint_constraints[0].joint_name = " << req.goal_constraints[0].joint_constraints[0].joint_name << std::endl;
  std::cout << "    goal_constraints[0].joint_constraints[0].position = " << req.goal_constraints[0].joint_constraints[0].position << std::endl;
  std::cout << "    path_constraints.position_constraints.size() = " << req.path_constraints.position_constraints.size() << std::endl;
  std::cout << "    trajectory_constraints.constraints.size() = " << req.trajectory_constraints.constraints.size() << std::endl;

  if(req.group_name == "puzzle")
  {
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel_, "puzzle_bot"));
    res.trajectory_->clear();
    res.planning_time_ = 0.01;
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    Eigen::Affine3d closePuzzleToGripper;
    robot_state::RobotState wstate(planning_scene->getCurrentState());
    robot_state::robotStateMsgToRobotState(req.start_state, wstate);
    bool response = planning_pipeline::ungrip("right_arm", "right_gripper", "r_gripper_l_finger_joint", "right_arm_away", wstate, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
    std::string crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway;
    if(!response)
    {
      return false;
    }
    response = planning_pipeline::ungrip("left_arm", "left_gripper", "l_gripper_l_finger_joint", "left_arm_away", wstate, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
    if(!response)
    {
      return false;
    }
    planning_interface::MotionPlanRequest newReq;
    planning_interface::MotionPlanResponse newRes;
    newReq.group_name = "puzzle";
    newReq.num_planning_attempts = 1;
    newReq.allowed_planning_time = 5;
    robot_state::robotStateToRobotStateMsg(wstate, newReq.start_state);
    newReq.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.x;
    newReq.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.y;
    newReq.workspace_parameters.min_corner.z = req.workspace_parameters.min_corner.z;
    newReq.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.x;
    newReq.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.y;
    newReq.workspace_parameters.max_corner.z = req.workspace_parameters.max_corner.z;
    newReq.goal_constraints.resize(1);
    newReq.goal_constraints[0] = req.goal_constraints[0];
    std::cout << "ACTION: find puzzle plan." << std::endl;
    bool solved = (adapter_chain_)?(adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, newReq, newRes, adapter_added_state_index)):(planner_instance_->solve(planning_scene, newReq, newRes));
    if(!solved)
    {
      std::cout << "  failed to find puzzle plan." << std::endl;
      return false;
    }
    robot_state::RobotState pstate(newRes.trajectory_->getWayPoint(0));
    response = planning_pipeline::grip("right_arm", "r_wrist_roll_link", "right_gripper", "r_gripper_l_finger_joint", wstate, pstate, closePuzzleToGripper, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res, kmodel_);
    if(!response)
    {
      response = planning_pipeline::grip("left_arm", "l_wrist_roll_link", "left_gripper", "l_gripper_l_finger_joint", wstate, pstate, closePuzzleToGripper, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res, kmodel_);
      if(!response)
      {
        return false;
      }
      crArm = "left_arm";
      crGripper = "left_gripper";
      crGripperJoint = "l_gripper_l_finger_joint";
      crArmLastLink = "l_wrist_roll_link";
      crArmAway = "left_arm_away";
    }
    else
    {
      crArm = "right_arm";
      crGripper = "right_gripper";
      crGripperJoint = "r_gripper_l_finger_joint";
      crArmLastLink = "r_wrist_roll_link";
      crArmAway = "right_arm_away";
    }
    unsigned int kMax = newRes.trajectory_->getWayPointCount();
    unsigned int lastGripChange = 0;
    unsigned int k = 0;
    bool finished = false;
    while(!finished)
    {
      if(k < kMax)
      {
        robot_state::RobotState pstate(newRes.trajectory_->getWayPoint(k));
        bool canFollow = planning_pipeline::followPuzzle(crArm, crArmLastLink, closePuzzleToGripper, pstate, wstate, planning_scene, res);
        if(!canFollow)
        {
          bool c = true;
          while((!planning_pipeline::trySwitchingGrip(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway, wstate, pstate, closePuzzleToGripper, adapter_chain_, planner_instance_, planning_scene, req, res, adapter_added_state_index, kmodel_)) && (c))
          {
            if(lastGripChange < k)
            {
              unsigned int d;
              d = (unsigned int)(0.25*(k - lastGripChange));
              if(!d)
              {
                d = 1;
              }
              for(unsigned int j = 0; j < d; j++)
              {
                robot_state::RobotState bstate(newRes.trajectory_->getWayPoint(k - j - 1));
                planning_pipeline::followPuzzle(crArm, crArmLastLink, closePuzzleToGripper, bstate, wstate, planning_scene, res);
              }
              k = k - d;
            }
            else
            {
              c = false;
            }
          }
          if(!c)
          {
            std::cout << "Failure: Backtracked all the way with no solution found." << std::endl;
            return false;
          }
          lastGripChange = k;
        }
        else
        {
          k++;
        }
      }
      else
      {
        finished = planning_pipeline::ungrip(crArm, crGripper, crGripperJoint, crArmAway, wstate, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
        if(!finished)
        {
          k = kMax - 1;
          bool c = true;
          bool canFinish = false;
          while((c) && (!canFinish))
          {
            if(lastGripChange < k)
            {
              unsigned int d;
              d = (unsigned int)(0.85*(k - lastGripChange));
              if(!d)
              {
                d = 1;
              }
              for(unsigned int j = 0; j < d; j++)
              {
                robot_state::RobotState bstate(newRes.trajectory_->getWayPoint(k - j - 1));
                planning_pipeline::followPuzzle(crArm, crArmLastLink, closePuzzleToGripper, bstate, wstate, planning_scene, res);
              }
              k = k - d;
              robot_state::RobotState pstate(newRes.trajectory_->getWayPoint(k+1));
              canFinish = planning_pipeline::trySwitchingGrip(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway, wstate, pstate, closePuzzleToGripper, adapter_chain_, planner_instance_, planning_scene, req, res, adapter_added_state_index, kmodel_);
            }
            else
            {
              c = false;
            }
          }
          if(!c)
          {
            std::cout << "Failure: Backtracked all the way with no solution found." << std::endl;
            return false;
          }
          lastGripChange = k;
        }
      }
/*
      if(!canFollow)
      {
        planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
        bool canGrip = planning_pipeline::grip(crArm, crArmLastLink, crGripper, crGripperJoint, wstate, closePuzzleToGripper, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
        if(!canGrip)
        {
          std::cout << "Failed to switch grips: cannot bring other arm close." << std::endl;
          return false;
        }
        else
        {
          planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
          bool canUngrip = planning_pipeline::ungrip(crArm, crGripper, crGripperJoint, crArmAway, wstate, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
          if(!canUngrip)
          {
            std::cout << "Failed to switch grips: cannot remove previous grip." << std::endl;
            return false;
          }
          planning_pipeline::switchArms(crArm, crGripper, crGripperJoint, crArmLastLink, crArmAway);
        }
      }
      else
      {
        k++;
      }
*/
    }

    response = true;//planning_pipeline::ungrip(crArm, crGripper, crGripperJoint, crArmAway, wstate, adapter_chain_, planner_instance_, adapter_added_state_index, planning_scene, req, res);
    if(!response)
    {
      return false;
    }
    res.trajectory_->setGroupName("puzzle_bot");
    if (display_computed_motion_plans_)
    {
      moveit_msgs::DisplayTrajectory disp;
      disp.model_id = kmodel_->getName();
      disp.trajectory.resize(1);
      res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
      robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
      display_path_publisher_.publish(disp);
    }
    return true;
  }

/*
  if(req.group_name == "left_arm")
  {
    robot_state::RobotState wstate(planning_scene->getCurrentState());
    robot_state::JointStateGroup *arm = wstate.getJointStateGroup("left_arm");
    robot_state::JointStateGroup *dummy = wstate.getJointStateGroup("right_arm");
    robot_state::JointStateGroup *puzzle = wstate.getJointStateGroup("puzzle");
    Eigen::Affine3d puzzlePose, endEffectorPose;
    puzzlePose = wstate.getLinkState("ring_link")->getGlobalCollisionBodyTransform();
    //Eigen::Vector3d puzzlePosition;
    //puzzlePosition = puzzlePose.translation();

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(kmodel_, "left_arm"));
    res.trajectory_->clear();
    res.planning_time_ = 0.01;
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    planning_interface::MotionPlanRequest newReq;
    planning_interface::MotionPlanResponse newRes;
  newReq.group_name = "right_arm";
  newReq.num_planning_attempts = 1;
  newReq.allowed_planning_time = 5;
  robot_state::robotStateToRobotStateMsg(wstate, newReq.start_state);
  newReq.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.x;
  newReq.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.y;
  newReq.workspace_parameters.min_corner.z = req.workspace_parameters.min_corner.z;
  newReq.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.x;
  newReq.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.y;
  newReq.workspace_parameters.max_corner.z = req.workspace_parameters.max_corner.z;
  //getSideGrip(-3.14159/6.0, 0.255, false, puzzlePose, endEffectorPose);
  bool foundPoseC = dummy->setFromIK(endEffectorPose);
  //getSideGrip(-3.14159/6.0, 0.28, false, puzzlePose, endEffectorPose);
  bool foundPoseF = dummy->setFromIK(endEffectorPose);
  if (foundPoseF && foundPoseC)
  {
    newReq.goal_constraints.resize(1);
    newReq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(dummy);
    bool solved = (adapter_chain_)?(adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, newReq, newRes, adapter_added_state_index)):(planner_instance_->solve(planning_scene, newReq, newRes));

    unsigned int kMax = newRes.trajectory_->getWayPointCount();
    for(unsigned int k = 0; k < kMax; k++)
    {
      robot_state::RobotState tstate(newRes.trajectory_->getWayPoint(k));
      robot_state::JointStateGroup *right_arm = tstate.getJointStateGroup("right_arm");
      std::vector<double> jointValues;
      right_arm->getVariableValues(jointValues);
      dummy->setVariableValues(jointValues);
      res.trajectory_->addSuffixWayPoint(wstate, 0.01);
    }

    std::vector<robot_state::RobotStatePtr> traj;
    getSideGrip(-3.14159/6.0, 0.255, false, puzzlePose, endEffectorPose);
    dummy->computeCartesianPath(traj, "r_wrist_roll_link", endEffectorPose, true, 0.005, 1.2);
    unsigned int maxK = traj.size();
    for(unsigned int k = 0; k < maxK; k++)
    {
      res.trajectory_->addSuffixWayPoint(traj[k], 0.01);
    }
    maxK--;
    wstate = *(traj[maxK].get());
    arm = wstate.getJointStateGroup("left_arm");
    dummy = wstate.getJointStateGroup("right_arm");
    puzzle = wstate.getJointStateGroup("puzzle");

    robot_state::JointStateGroup *rightGripper = wstate.getJointStateGroup("right_gripper");
    std::map<std::string, double> gripperJointValues;
    rightGripper->getVariableValues(gripperJointValues);
    std::vector<double> haxx;
    rightGripper->getVariableValues(haxx);
    for(unsigned int k = 0; k < haxx.size(); k++)
    {
      std::cout << "  haxx[" << k << "] " << haxx[k] << std::endl;
    }
    std::cout << "  r_gripper_l_finger_joint " << gripperJointValues["r_gripper_l_finger_joint"] << std::endl;
    gripperJointValues["r_gripper_l_finger_joint"] = 0.2;
    haxx[2] = 0.2;
    std::cout << "  r_gripper_l_finger_joint " << gripperJointValues["r_gripper_l_finger_joint"] << std::endl;
    //rightGripper->setVariableValues(gripperJointValues);
    //rightGripper->setVariableValues(haxx);
    rightGripper->setToDefaultState("right_gripper_intermediate");
    std::cout << "Wnum " << res.trajectory_->getWayPointCount() << std::endl;
    arm->setToRandomValues();
    res.trajectory_->addSuffixWayPoint(wstate, 0.01);
    gripperJointValues["r_gripper_l_finger_joint"] = 0.1;
    haxx[2] = 0.1;
    std::cout << "  r_gripper_l_finger_joint " << gripperJointValues["r_gripper_l_finger_joint"] << std::endl;
    //rightGripper->setVariableValues(gripperJointValues);
    //rightGripper->setVariableValues(haxx);
    rightGripper->setToDefaultState("right_gripper_closed");
    arm->setToRandomValues();
    res.trajectory_->addSuffixWayPoint(wstate, 0.01);
    std::cout << "Wnum " << res.trajectory_->getWayPointCount() << std::endl;
    res.trajectory_->setGroupName("puzzle_bot");
    if (display_computed_motion_plans_ && solved)
    {
      moveit_msgs::DisplayTrajectory disp;
      disp.model_id = kmodel_->getName();
      disp.trajectory.resize(1);
      res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
      robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
      display_path_publisher_.publish(disp);
    }
    return solved;
  }
  return false;
*/
/*
    double pi = 3.14159;
    double angles[6] = {pi/6.0, pi/2.0, 5.0*pi/6.0, 7.0*pi/6.0, 9.0*pi/6.0, 11.0*pi/6.0};

    bool foundPoseF, foundPoseC;
    for(unsigned int k = 0; k < 6; k++)
    {
      getAlignedGrip(angles[k], 0.21, false, puzzlePose, endEffectorPose);
      foundPoseF = arm->setFromIK(endEffectorPose);
      getAlignedGrip(angles[k], 0.185, false, puzzlePose, endEffectorPose);
      foundPoseC = arm->setFromIK(endEffectorPose);
      if(foundPoseF && foundPoseC)
      {
        res.trajectory_->addSuffixWayPoint(wstate, 0.01);
      }
    }
    for(unsigned int k = 0; k < 6; k++)
    {
      getAlignedGrip(angles[k], 0.21, true, puzzlePose, endEffectorPose);
      foundPoseF = arm->setFromIK(endEffectorPose);
      getAlignedGrip(angles[k], 0.185, true, puzzlePose, endEffectorPose);
      foundPoseC = arm->setFromIK(endEffectorPose);
      if(foundPoseF && foundPoseC)
      {
        res.trajectory_->addSuffixWayPoint(wstate, 0.01);
      }
    }
    for(unsigned int k = 0; k < 6; k++)
    {
      getSideGrip(angles[k], 0.28, false, puzzlePose, endEffectorPose);
      foundPoseF = arm->setFromIK(endEffectorPose);
      getSideGrip(angles[k], 0.255, false, puzzlePose, endEffectorPose);
      foundPoseC = arm->setFromIK(endEffectorPose);
      if(foundPoseF && foundPoseC)
      {
        res.trajectory_->addSuffixWayPoint(wstate, 0.01);
      }
    }
    for(unsigned int k = 0; k < 6; k++)
    {
      getSideGrip(angles[k], 0.28, true, puzzlePose, endEffectorPose);
      foundPoseF = arm->setFromIK(endEffectorPose);
      getSideGrip(angles[k], 0.255, true, puzzlePose, endEffectorPose);
      foundPoseC = arm->setFromIK(endEffectorPose);
      if(foundPoseF && foundPoseC)
      {
        res.trajectory_->addSuffixWayPoint(wstate, 0.01);
      }
    }
  // display solution path if needed
    if (display_computed_motion_plans_)
    {
      moveit_msgs::DisplayTrajectory disp;
      disp.model_id = kmodel_->getName();
      disp.trajectory.resize(1);
      res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
      robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
      display_path_publisher_.publish(disp);
    }
    return true;
  }
*/

  if (!planner_instance_)
  {
    ROS_ERROR("No planning plugin loaded. Cannot plan.");
    return false;
  }
  
  bool solved = false;
  try
  {
    if (adapter_chain_)
    {
      solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
      if (!adapter_added_state_index.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0 ; i < adapter_added_state_index.size() ; ++i)
          ss << adapter_added_state_index[i] << " ";
        ROS_DEBUG("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
      }
    }
    else
      solved = planner_instance_->solve(planning_scene, req, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Exception caught: '%s'", ex.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR("Unknown exception thrown by planner");
    return false;
  }
  bool valid = true;
  
  if (solved && res.trajectory_)
  {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
    if (check_solution_paths_)
    {
      std::vector<std::size_t> index;
      if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
        bool problem = false;
        for (std::size_t i = 0 ; i < index.size() && !problem ; ++i)
        {
          bool found = false;
          for (std::size_t j = 0 ; j < adapter_added_state_index.size() ; ++j)
            if (index[i] == adapter_added_state_index[j])
            {
              found = true;
              break;
            }
          if (!found)
            problem = true;
        }
        if (problem)
        {          
          if (index.size() == 1 && index[0] == 0) // ignore cases when the robot starts at invalid location
            ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
          else
          {
            valid = false;
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;            
            
            // display error messages
            std::stringstream ss;
            for (std::size_t i = 0 ; i < index.size() ; ++i)
              ss << index[i] << " ";
            ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ " << ss.str() << "] out of " << state_count);
            
            // call validity checks in verbose mode for the problematic states
            visualization_msgs::MarkerArray arr;
            for (std::size_t i = 0 ; i < index.size() ; ++i)
            {
              // check validity with verbose on
              const robot_state::RobotState &kstate = res.trajectory_->getWayPoint(index[i]);
              planning_scene->isStateValid(kstate, req.group_name, false);
              
              // compute the contacts if any
              collision_detection::CollisionRequest c_req;
              collision_detection::CollisionResult c_res;
              c_req.contacts = true;
              c_req.max_contacts = 10;
              c_req.max_contacts_per_pair = 3;
              c_req.verbose = false;
              planning_scene->checkCollision(c_req, c_res, kstate);
              if (c_res.contact_count > 0)
              {
                visualization_msgs::MarkerArray arr_i;
                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(), c_res.contacts);
                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
              }
            }
            if (!arr.markers.empty())
              contacts_publisher_.publish(arr);
          }
        }
        else
          ROS_DEBUG("Planned path was found to be valid, except for states that were added by planning request adapters, but that is ok.");
      }
      else
        ROS_DEBUG("Planned path was found to be valid when rechecked");
    }
  }
  
  // display solution path if needed
  if (display_computed_motion_plans_ && solved)
  { 
    moveit_msgs::DisplayTrajectory disp;
    disp.model_id = kmodel_->getName();
    disp.trajectory.resize(1);
    res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
    robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
    display_path_publisher_.publish(disp);      
  }
  
  return solved && valid;
}

void planning_pipeline::PlanningPipeline::terminate() const
{
  if (planner_instance_)
    planner_instance_->terminate();
}
