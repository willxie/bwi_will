#include <ros/ros.h>
// #include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_search");
    ros::NodeHandle *nh = new ros::NodeHandle();

    ros::Publisher simple_goal_pub = nh->advertise<geometry_msgs::PoseStamped>("/move_base_interruptable_simple/goal", 100);

    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::PoseStamped goal_msg;

        goal_msg.header.frame_id = "level_mux/map";
        goal_msg.pose.position.x = -40.0;
        goal_msg.pose.position.y = -11.3;
        goal_msg.pose.orientation.z   = 1;
        goal_msg.pose.orientation.w   = 0;


        ROS_INFO("Publishing new goal...");
        simple_goal_pub.publish(goal_msg);

        ros::Duration(10).sleep();
    }
}
