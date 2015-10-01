#include <iostream>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>


using namespace std;


bool execute(moveit::planning_interface::MoveGroup group, vector <geometry_msgs::Pose> waypoints)
{
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory,
                                                false);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.plan(my_plan);
    my_plan.trajectory_ = trajectory;
    cout << "before" << endl;
    group.execute(my_plan);

    return true;
}

//moveit_msgs::RobotTrajectory trajectory;

void execute(moveit::planning_interface::MoveGroup group, geometry_msgs::Pose pose)
{
    cout << " first " << endl;
    vector <geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    //execute(group, waypoints);
    moveit_msgs::RobotTrajectory trajectory;
    cout << "somewhere" << endl;
    moveit_msgs::MoveItErrorCodes error_code;
    
    double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory,
                                               false,
                                                &error_code);

    //return;
    //cout << "middle" << endl;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.plan(my_plan);
    //    cout << "2nd" << endl;
    my_plan.trajectory_ = trajectory;
    //cout << "before" << endl;
    //for(int i=0;i<3;i++)    
        group.execute(my_plan);
    //cout << "after" << endl;
}

const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
robot_state::RobotState getCurrentRobotState(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
{
        // each time the current state is needed
        planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
        planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
        ps->getCurrentStateNonConst().update();
        robot_state::RobotState current_state = ps->getCurrentState();
        return current_state;
}



void moveTo(robot_state::RobotStatePtr state)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG1");

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();    
    
    vector <double> vals(5);

    
    kinematic_state->setJointGroupPositions("LEG1", vals);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("leg1link5");

    cout << *kinematic_state->getJointPositions("leg1joint1") << endl;

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    

  /*
  moveit::planning_interface::MoveGroup group("LEG1");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ =
    boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  geometry_msgs::Pose target_pose1 = group.getRandomPose().pose;  
  

  double diff = 0;
  while(ros::ok()){
  //group.setStartState(*group.getCurrentState());
  //group.setStartStateToCurrentState();

  group.setStartState(getCurrentRobotState(planning_scene_monitor_));

  double x,y,z;

  cout << "please enter" << endl;
  cin >> x >> y >> z;

    target_pose1.orientation.x = -0.357997;
    target_pose1.orientation.y = 0.0187744;
    target_pose1.orientation.z = -0.467386;
    target_pose1.orientation.w = 0.808106;
    
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;

    cout << target_pose1 << endl;

    execute(group, target_pose1);
    
    vector <geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);
    target_pose1.position.x += 0.1;  
    waypoints.push_back(target_pose1);
    target_pose1.position.x += 0.1;  
    waypoints.push_back(target_pose1);
    //execute(group, waypoints);
    exit(0);
  }
 */
  ros::shutdown();  
  return 0;
}
