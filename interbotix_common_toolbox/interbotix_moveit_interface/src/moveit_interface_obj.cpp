// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "interbotix_moveit_interface/moveit_interface_obj.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>

namespace interbotix
{

InterbotixMoveItInterface::InterbotixMoveItInterface(
  rclcpp::Node::SharedPtr & node)
: node_(node)
{
  // We spin up a MultiThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  std::thread([&executor]() {executor.spin();}).detach();

  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    PLANNING_GROUP);
  joint_model_group = move_group->getCurrentState(2.0)->getJointModelGroup(PLANNING_GROUP);

  planning_scene = std::make_shared<moveit::planning_interface::PlanningSceneInterface>(
    MOVEIT_NAMESPACE, // Namespace in which all MoveIt related topics and services are discovered
    true);            // wait for services

  srv_moveit_plan = node_->create_service<MoveItPlan>(
    "moveit_plan",
    std::bind(
      &interbotix::InterbotixMoveItInterface::moveit_planner,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  srv_clear_markers = node_->create_service<Empty>(
    "clear_markers",
    std::bind(
      &interbotix::InterbotixMoveItInterface::clear_markers,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    node_,
    move_group->getPlanningFrame(),
    "/moveit_visual_tools");
  visual_tools->deleteAllMarkers();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.770;
  visual_tools->publishText(
    text_pose,
    "InterbotixMoveItInterface",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->trigger();

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(
    node_->get_logger(),
    "Reference frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(
    node_->get_logger(),
    "End effector link: %s", move_group->getEndEffectorLink().c_str());
}

InterbotixMoveItInterface::~InterbotixMoveItInterface()
{
  delete joint_model_group;
}

bool InterbotixMoveItInterface::moveit_plan_joint_positions(
  const std::vector<double> joint_group_positions)
{
  visual_tools->deleteAllMarkers();
  move_group->setJointValueTarget(joint_group_positions);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishText(
    text_pose,
    "Joint Space Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::msg::Pose pose)
{
  visual_tools->deleteAllMarkers();
  move_group->setPoseTarget(pose);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Pose Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_position(double x, double y, double z)
{
  visual_tools->deleteAllMarkers();
  move_group->setPositionTarget(x, y, z);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  RCLCPP_INFO(
    node_->get_logger(),
    "Target: x, y, z: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);

  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);

  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Position Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_orientation(
  const geometry_msgs::msg::Quaternion quat)
{
  visual_tools->deleteAllMarkers();
  move_group->setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  geometry_msgs::msg::Pose pose;
  pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Orientation Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan successful: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose> waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);
  RCLCPP_INFO(
    node_->get_logger(),
    "Visualizing (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  visual_tools->deleteAllMarkers();
  visual_tools->publishText(
    text_pose,
    "Cartesian Path",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishPath(
    waypoints,
    rviz_visual_tools::LIME_GREEN,
    rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    visual_tools->publishAxisLabeled(
      waypoints[i],
      "pt" + std::to_string(i),
      rviz_visual_tools::SMALL);
  }
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  saved_plan = plan;
  saved_plan.trajectory_ = trajectory;

  // If a plan was found for over 90% of the waypoints...
  // consider that a successful planning attempt
  return (1.0 - fraction < 0.1) && (fraction != -1.0);
}

bool InterbotixMoveItInterface::moveit_execute_plan(void)
{
  return move_group->execute(saved_plan) == MoveItErrorCode::SUCCESS;
}

void InterbotixMoveItInterface::moveit_set_path_constraint(
  const std::string constrained_link,
  const std::string reference_link,
  const geometry_msgs::msg::Quaternion quat,
  const double tolerance)
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;

  // this parameter sets the importance of this constraint relative to other constraints that might
  // be present. Closer to '0' means less important.
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // Since there is a constraint, it might take the planner a lot longer to come up with a valid
  // plan - so give it some time
  move_group->setPlanningTime(30.0);
}

void InterbotixMoveItInterface::moveit_clear_path_constraints(void)
{
  move_group->clearPathConstraints();

  // Now that there are no constraints, reduce the planning time to the default
  move_group->setPlanningTime(5.0);
}

geometry_msgs::msg::Pose InterbotixMoveItInterface::moveit_get_ee_pose(void)
{
  return move_group->getCurrentPose().pose;
}

void InterbotixMoveItInterface::moveit_scale_ee_velocity(const double factor)
{
  move_group->setMaxVelocityScalingFactor(factor);
}

bool InterbotixMoveItInterface::moveit_planner(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<MoveItPlan::Request> req,
  std::shared_ptr<MoveItPlan::Response> res)
{
  (void)request_header;
  bool success = false;
  std::string service_type;
  if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSE) {
    success = moveit_plan_ee_pose(req->ee_pose);
    service_type = "Planning EE pose";
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSITION) {
    success = moveit_plan_ee_position(
      req->ee_pose.position.x,
      req->ee_pose.position.y,
      req->ee_pose.position.z);
    service_type = "Planning EE position";
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_ORIENTATION) {
    success = moveit_plan_ee_orientation(req->ee_pose.orientation);
    service_type = "Planning EE orientation";
  } else if (req->cmd == MoveItPlan::Request::CMD_EXECUTE) {
    success = moveit_execute_plan();
    service_type = "Execution";
  }
  res->success = success;
  if (success) {
    res->msg.data = service_type + " was successful!";
  } else {
    res->msg.data = service_type + " was not successful.";
  }

  return true;
}

bool InterbotixMoveItInterface::clear_markers(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<Empty::Request> req,
  std::shared_ptr<Empty::Response> res)
{
  (void)request_header;
  (void)req;
  (void)res;
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
  RCLCPP_DEBUG(node_->get_logger(), "Cleared markers.");
  return true;
}

bool InterbotixMoveItInterface::moveit_add_collision_object(
  const std::string object_id,
  const std::vector<float> dimensions,
  const std::vector<double> object_pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.id = object_id;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  RCLCPP_INFO(node_->get_logger(), "New collision object created with id: " + object_id.c_str());

  // define objects size and shape
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3); // TODO: adjust 
  primitive.dimensions[primitive.BOX_X] = dimensions[0];
  primitive.dimensions[primitive.BOX_Y] = dimensions[1];
  primitive.dimensions[primitive.BOX_Z] = dimensions[2];

  // set the position and orientation of object
  geometry_msgs::msg::Pose co_pose;
  co_pose.position.x = object_pose[0];
  co_pose.position.y = object_pose[1];
  co_pose.position.z = object_pose[2];
  co_pose.orientation.x = object_pose[3];
  co_pose.orientation.y = object_pose[4];
  co_pose.orientation.z = object_pose[5];
  co_pose.orientation.w = object_pose[6];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(co_pose);
  collision_object.operation = collision_object.ADD;

  // add collision object to collective list of objects
  collision_objects.push_back(collision_object);

  // the planning scene will update automatically through the diff_publisher 
  planning_scene.addCollisionObjects(collision_objects);
  RCLCPP_INFO(node_->get_logger(), "Adding new object into the world");
  return true;
}

bool InterbotixMoveItInterface::moveit_remove_collision_object(
  const std::string object_id
)
{
  const std::vector<std::string> id_of_object_to_remove;
  id_of_object_to_remove.push_back(object_id);
  std::map<std::string, moveit_msgs::CollisionObject> found_objects = planning_scene_interface.getObjects(id_of_object_to_remove);
  
  if(found_objects.find(object_id) != found_objects.end()) {
    moveit_msgs::msg::CollisionObject object_to_remove = found_objects[object_id]; // OR found_objects.find(object_id).second; //to get value from map  
    
    auto it = std::find(collision_objects.begin(), collision_objects.end(), object_to_remove);
    if(it != collision_objects.end()) {
      collision_objects.erase(it); // erase object from private list
    }
    planning_scene.removeCollisionObjects(id_of_object_to_remove);
    RCLCPP_INFO(node_->get_logger(), "Removing object from the world");
    return true;
  }
  RCLCPP_INFO(node_->get_logger(), "Could not find object with id: " + object_id.c_str());
  return false;
}

bool InterbotixMoveItInterface::moveit_attach_object_to_ee(
  const std::string object_id,
  const std::vector<std::string> touch_links)
{
  const std::vector<std::string> id_of_object_to_attach;
  id_of_object_to_attach.push_back(object_id);
  std::map<std::string, moveit_msgs::CollisionObject> found_objects = planning_scene.getObjects(id_of_object_to_attach);
  
  if(found_objects.find(object_id) != found_objects.end()) {
    RCLCPP_INFO(node_->get_logger(), "Removing object from world, before attaching it to end effector.");

    if(moveit_remove_collision_object(object_id)){
      moveit_msgs::msg::AttachedCollisionObject object_to_attach;
      object_to_attach.object = found_objects[object_id]; // OR found_objects.find(object_id).second; //to get value from map
      object_to_attach.link_name = move_group.getEndEffectorLink();
      object_to_attach.touch_links = touch_links; // The set of links that the attached object is allowed to touch
      object_to_attach.object.operation = attached_object.object.ADD;

      move_group.attachObject(object_to_attach.object.id, object_to_attach.link_name, touch_links);
      planning_scene.applyAttachedCollisionObject(object_to_attach);
      RCLCPP_INFO(node_->get_logger(), "Attaching object to end effector.");
      return true;
    }
    else {
      RCLCPP_INFO(node_->get_logger(), "Failed to remove object from the world.");
      return false;
    }
  } 
  else {
    RCLCPP_INFO(node_->get_logger(), "The object you are trying to add to the end effector does not exist.");
  }
  return false;
}

bool InterbotixMoveItInterface::moveit_detach_object_from_ee(
  const std::string object_id
  bool reintroduce = true)
{
  const std::vector<std::string> id_of_object_to_detach;
  id_of_object_to_detach.push_back(object_id);

  std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects;
  attached_objects = planning_scene.getAttachedObjects(id_of_object_to_detach);
  
  RCLCPP_INFO(node_->get_logger(), "Detaching the object from the end effector");
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object = attached_objects[object_id].object;
  if (move_group.detachObject(collision_object.id)){
    if(reintroduce){
      RCLCPP_INFO(node_->get_logger(), "Reintroducing detached object into the world.");
      collision_object.operation = collision_object.ADD;
      // update private list of objects
      collision_objects.push_back(collision_object);
      return  applyCollisionObject(collision_object);
    }
    else{
      RCLCPP_INFO(node_->get_logger(), "Removing detached object from the scene");
      moveit_msgs::msg::AttachedCollisionObject detach_object;
      detach_object = attached_objects[object_id];
      detach_object.object.operation = attached_object.object.REMOVE; 
      return applyAttachedCollisionObject(detach_object);
    }
  }
  else{
    RCLCPP_INFO(node_->get_logger(), "The specified object to detach could not be identified.");
    return false;
  }


}


}  // namespace interbotix
