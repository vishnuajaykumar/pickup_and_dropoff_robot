#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";

    while (ros::ok()) {
        // Define a position and orientation for the robot to reach pick up
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 2.595;
        goal.target_pose.pose.position.y = -1.450;
        goal.target_pose.pose.orientation.w = 1.0;

        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Sending pickup goal");
        ac.sendGoal(goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot reached the pickup zone, waiting...");
            ros::Duration(5.0).sleep(); // Simulate pickup time
        } else {
            ROS_WARN("Failed to reach the pickup zone");
            continue; 
        }

        // Define a position and orientation for the robot to reach drop off
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = -2.94;
        goal.target_pose.pose.position.y = -2.29;
        goal.target_pose.pose.orientation.w = 1.0;

        // Send the drof pff position and orientation for the robot to reach
        ROS_INFO("Sending drop-off goal");
        ac.sendGoal(goal);
        ac.waitForResult();

        // Check if the robot reached drop-off zone
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot reached the drop-off zone, waiting...");
            ros::Duration(5.0).sleep(); // Simulate drop-off time
        } else {
            ROS_WARN("Failed to reach the drop-off zone");
            continue; 
        }

        ROS_INFO("Restarting pickup and drop-off cycle...");
        ros::Duration(2.0).sleep(); 
    }

    return 0;
}
