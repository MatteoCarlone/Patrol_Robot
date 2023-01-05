#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "patrol_robot/MarkerRoutine.h"

int position=0;

bool reach(patrol_robot::MarkerRoutine::Request &req, patrol_robot::MarkerRoutine::Response &resp){
  
  if(req.pos == 1){
  	position=1;
  	resp.message = "MR_left";
  	}
  else if (req.pos == 2){ 
  	position=2;
  	resp.message = "home";
  	}
  else if (req.pos == 3){ 
    position=3;
    resp.message = "MR_right";
    }
  else if (req.pos == 4){ 
    position=4;
    resp.message = "MR_back";
    }
  else if (req.pos == 5){ 
    position=5;
    resp.message = "stop";
    }
  else{ 

    return false; 
  }

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  moveit::planning_interface::MoveGroupInterface group("arm");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  group.setStartStateToCurrentState();
  group.setGoalOrientationTolerance(1);
  group.setGoalPositionTolerance(1);
  
  
  while(ros::ok()){
  	 if (position==1){
  	  group.setNamedTarget("MR_left");
  	  group.move();
  	  position=0;
      }
  	 if (position==2){
  	  group.setNamedTarget("home");
  	  group.move();
  	  position=0;
      }
     if (position==3){
      group.setNamedTarget("MR_right");
      group.move();
      position=0;
      }
     if (position==4){
      group.setNamedTarget("MR_back");
      group.move();
      position=0;
      }
     if (position==5){
      group.setNamedTarget("stop");
      group.move();
      position=0;
      }

  }
  return(0);
}