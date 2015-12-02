#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <map>
#include "map.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"


// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"

#include <termios.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose current_position;

std::vector<std::pair<int,int> > free_space_indices;

std::string map_frame = "level_mux/map";
// std::string map_frame = "map";

map_t* map_;
bool has_map = false;


// List of publisher and subscribers
// TODO: really need to class-ify this
ros::Publisher cmd_vel_pub;
ros::Publisher loc_visited_pub;
ros::Publisher loc_free_pub;
ros::Subscriber amcl_pose_sub;
ros::Subscriber map_sub;

void mySigintHandler(int sig)
{
  ROS_INFO("Shutting down...");
  // Do some custom action.
  // For example, publish a stop message to some other nodes

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

// Non-blocking keyboard press
// http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}



void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_position = msg->pose.pose;


}

map_t*
convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
    map_t* map = map_alloc();
    ROS_ASSERT(map);

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    // Convert to player format
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    ROS_ASSERT(map->cells);
    for(int i=0;i<map->size_x * map->size_y;i+=10000) // Down sampled
    {
        if(map_msg.data[i] == 0)
            map->cells[i].occ_state = -1; // Free
        else if(map_msg.data[i] == 100)
            map->cells[i].occ_state = +1; // Occupied
        else
            map->cells[i].occ_state = 0; // Unknown
    }

    return map;
}

void handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
    map_ = convertMap(msg);

    // Index of free space
    free_space_indices.resize(0);
    for(int i = 0; i < map_->size_x; i++)
        for(int j = 0; j < map_->size_y; j++)
            if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
                free_space_indices.push_back(std::make_pair(i,j));

}


void publishFreeSpace() {
    // Publish all free locations
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = map_frame;
    cloud_msg.poses.resize(free_space_indices.size());
    for (int i = 0; i < free_space_indices.size(); ++i) {
        auto& loc = free_space_indices[i];

        // ROS_INFO("(%d, %d)", loc.first, loc.second);
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(0),
                                 tf::Vector3(MAP_WXGX(map_, loc.first),
                                             MAP_WYGY(map_, loc.second),
                                             0)),
                        cloud_msg.poses[i]);
    }

    loc_free_pub.publish(cloud_msg);

    ROS_INFO("Free location size: %d", (int)free_space_indices.size());
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    ROS_INFO("Map received");

    // Only get map once
    if (has_map) {
        return;
    }

    handleMapMessage(*msg);

    publishFreeSpace();
    has_map = true;
}

int main(int argc, char **argv)
{
    srand(time(NULL));

    // TODO: YAML
    // Some predefined locations
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

    signal(SIGINT, mySigintHandler);

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // ros::Subscriber odom_sub = nh.subscribe("odom", 10, chatterCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    loc_free_pub = nh.advertise<geometry_msgs::PoseArray>("loc_free", 2, true);
    loc_visited_pub = nh.advertise<geometry_msgs::PoseArray>("loc_visited", 2, true);

    amcl_pose_sub = nh.subscribe("amcl_pose", 100, amclPoseCallback);
    map_sub = nh.subscribe(map_frame, 1, mapCallback);

    double theta = M_PI;
    int last_location_num = -1;
    bool keypress_exit = false;

    ROS_INFO("Start");

    while (!has_map) {
        ros::spinOnce();

        ros::Duration(1.0).sleep();
    }


    while (ros::ok())
    {
        // Different location each time
        int location_num = rand() % locations.size();
        while (location_num == last_location_num) {
            location_num = rand() % locations.size();
        }
        last_location_num = location_num;

        // std::pair<double, double>& new_location = locations[location_num];

        // Shuffle free locations
        std::random_shuffle ( free_space_indices.begin(), free_space_indices.end() );
        std::pair<int,int> fsc = free_space_indices.back();
        free_space_indices.pop_back();
        std::pair<double, double> new_location = std::make_pair<double,double>(MAP_WXGX(map_, fsc.first), MAP_WYGY(map_, fsc.second));

        ROS_INFO("Location #%d", location_num);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = map_frame;
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

        // while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        // }
        while (!ac.waitForResult(ros::Duration(1.0))) {
            ROS_INFO(".");
            // ROS_INFO("%d", ac.getState());
            // int c = getch();   // call your non-blocking input function
            // if (c == '\b') {
            //     ROS_INFO("Keyboard interrupt");
            //     ac.cancelGoal();
            //     ROS_INFO("Goal canceled")
            //     std::pair<double, double>& new_location = locations[0];
            //     goal.target_pose.header.frame_id = "level_mux/map";
            //     goal.target_pose.header.stamp = ros::Time::now();
            //     goal.target_pose.pose.position.x = new_location.first;
            //     goal.target_pose.pose.position.y = new_location.second;
            //     ac.sendGoal(goal);
            //     ROS_INFO("Going back home");
            // }
            // keypress_exit = true;
        }

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Move success");
        else
            ROS_INFO("Move failed");

        ros::Duration(2).sleep();

        // Spin
        // ROS_INFO("Spinning");
        // geometry_msgs::Twist rotate;
        // rotate.angular.z = 0.3;

        // ros::Time start_time = ros::Time::now();
        // ros::Duration timeout(12.0); // Timeout of 2 seconds
        // while(ros::Time::now() - start_time < timeout) {
        //     cmd_vel_pub.publish(rotate);
        // }

        // rotate.angular.z = 0.0;
        // cmd_vel_pub.publish(rotate);
        // cmd_vel_pub.publish(rotate);
        // cmd_vel_pub.publish(rotate);
        // ros::Duration(10).sleep();

        ROS_INFO("Done.");

        ros::spinOnce();

        publishFreeSpace();

    }

    ROS_INFO("Gracefully exiting");
    return 0;
}
