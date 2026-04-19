#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <vector>
#include <cmath>

std::vector<geometry_msgs::Pose> generateTrefoilKnotWaypoints(int num_points = 300) {
    std::vector<geometry_msgs::Pose> waypoints;
    
    for (int i = 0; i < num_points; ++i) {
        double t = 2 * M_PI * i / num_points;
        
        double scale = 0.05;
        double offset_x = 0.4;
        double offset_y = 0.2;
        double offset_z = 0.5;
        // Equazione parametrica del Trefoil Knot
        double x = scale * (std::sin(t) + 2.0 * std::sin(2.0 * t)) + offset_x;
        double y = scale * (std::cos(t) - 2.0 * std::cos(2.0 * t)) + offset_y;
        double z = scale * (-std::sin(3.0 * t)) + offset_z;
        
        geometry_msgs::Pose waypoint;
        waypoint.position.x = x;
        waypoint.position.y = y;
        waypoint.position.z = z;
        waypoint.orientation.x = 0.0;
        waypoint.orientation.y = 0.0;
        waypoint.orientation.z = 0.0;
        waypoint.orientation.w = 1.0;
        
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "panda_trefoil_trajectory");
    ros::NodeHandle node_handle;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Avvio di MoveIt e Gazebo...");
    ros::Duration(1.0).sleep();

    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    
    //Aggiunta per far eseguire la traiettoria all'end effector e non al link 8
     move_group.setEndEffectorLink("panda_hand_tcp");
    
    move_group.setPlanningTime(50.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.3);

    ROS_INFO("Generazione traiettoria Trefoil Knot...");
    std::vector<geometry_msgs::Pose> waypoints = generateTrefoilKnotWaypoints(300);
    
    // Prendi il primo punto come posizione iniziale
    geometry_msgs::Pose start_pose = waypoints[0];
    
    ROS_INFO("Movimento al punto iniziale della traiettoria...");
    
    move_group.setPoseTarget(start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    moveit_msgs::MoveItErrorCodes error_code;
    bool success = (move_group.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.execute(start_plan);
        ROS_INFO("Posizione iniziale raggiunta");
    } else {
        ROS_WARN("Impossibile arriva al punto iniziale");
        return 1;
    }
    
    if (success) {
        ROS_INFO("Pianificazione traiettoria...");
        
        // Aspetta un momento dopo il movimento iniziale
        ros::Duration(1.0).sleep();
        
        double eef_step = 0.0005;
        moveit_msgs::RobotTrajectory trefoil_traj;
        moveit_msgs::MoveItErrorCodes error_code;
        
        // Compute Cartesian Path - escludi il primo punto (già raggiunto)
        std::vector<geometry_msgs::Pose> trajectory_waypoints(waypoints.begin() + 1, waypoints.end());
        double fraction = move_group.computeCartesianPath(trajectory_waypoints, eef_step, trefoil_traj, false, &error_code);
        ROS_INFO("Percentuale di successo della pianificazione %.2f%%", fraction * 100.0);

        if (fraction > 0.9 && error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            moveit::planning_interface::MoveGroupInterface::Plan plan_trefoil;
            plan_trefoil.trajectory_ = trefoil_traj;
            ROS_INFO("Esecuzione traiettoria Trefoil Knot...");
            move_group.execute(plan_trefoil);
            ROS_INFO("Traiettoria completata");
        } 
        else {
            ROS_WARN("Pianificazione non riuscita completamente, traiettoria parziale eseguita");
            if (fraction > 0.7) {
                moveit::planning_interface::MoveGroupInterface::Plan plan_trefoil;
                plan_trefoil.trajectory_ = trefoil_traj;
                move_group.execute(plan_trefoil);
            } 
            else {
                ROS_ERROR("Impossibile eseguire la traiettoria");
            }
        }
    } 
    else {
        ROS_ERROR("Non è possibile raggiungere il punto iniziale. Esecuzione traiettoria annullata.");
    }
    ros::waitForShutdown();
    return 0;
}

