#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose current_position;
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_position = msg->pose.pose;
}

int main(int argc, char **argv)
{
    srand(time(NULL));

    // TODO: YAML
    std::vector<std::pair<double, double> > locations; // (x, y)
    locations.emplace_back(-35.0, -11.5);   // In front of my computer
    locations.emplace_back(-30.03, -4.73);  // Kitchen
    locations.emplace_back(-47.67, -7.75);  // Robot soccer field
    locations.emplace_back(-19.18, -4.95);  // Printer
    locations.emplace_back(-13.97, -11.90); // North front student desk intersection

    // const string locations_yaml_path = ros::package::getPath("bwi_object_search") +
    //     "yaml/locations.yaml";

    // YAML::Node locations = YAML::LoadFile(locations_yaml_path.c_str());

    // for (YAML::const_iterator it = locations.begin(); it != locations.end(); ++it)
    // {
    //     ROS_INFO("%S");
    // }

    // parser.GetNextDocument(doc);

    ros::init(argc, argv, "object_search");
    ros::NodeHandle nh;

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // ros::Subscriber odom_sub = nh.subscribe("odom", 10, chatterCallback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber amcl_pose_sub = nh.subscribe("amcl_pose", 100, amclPoseCallback);

    double theta = M_PI;
    int last_location_num = -1;
    while (ros::ok())
    {
        move_base_msgs::MoveBaseGoal goal;

        // Different location each time
        int location_num = rand() % locations.size();
        while (location_num == last_location_num) {
            location_num = rand() % locations.size();
        }
        last_location_num = location_num;

        std::pair<double, double>& new_location = locations[location_num];
        // std::pair<double, double>& new_location = locations[0];
        ROS_INFO("Location #%d", location_num);
        goal.target_pose.header.frame_id = "level_mux/map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = new_location.first;
        goal.target_pose.pose.position.y = new_location.second;

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
            ROS_INFO("Move success");
        else
            ROS_INFO("Move failed");

        ros::Duration(2).sleep();

        // Spin
        ROS_INFO("Spinning");
        geometry_msgs::Twist rotate;
        rotate.angular.z = 0.3;

        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(12.0); // Timeout of 2 seconds
        while(ros::Time::now() - start_time < timeout) {
            cmd_vel_pub.publish(rotate);
        }

        ROS_INFO("Done.");
        rotate.angular.z = 0.0;
        cmd_vel_pub.publish(rotate);
        cmd_vel_pub.publish(rotate);
        cmd_vel_pub.publish(rotate);
        ros::Duration(10).sleep();
        ros::spinOnce();
    }


}
