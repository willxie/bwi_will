#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_search");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    double theta = M_PI;
    while (ros::ok())
    {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "level_mux/map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = -35.0;
        goal.target_pose.pose.position.y = -11.5;

        theta += 0.5;
        if (theta > 2.0 * M_PI) {
            theta -= 2.0 * M_PI;
        }
        goal.target_pose.pose.orientation.z = sin(theta / 2);
        goal.target_pose.pose.orientation.w = cos(theta / 2);

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Turn success");
        else
            ROS_INFO("Turn failed");

    }
}
