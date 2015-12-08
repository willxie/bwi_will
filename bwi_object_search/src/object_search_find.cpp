#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>


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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// #include <btQuaternion.h>

// This for transforming quanterion to RPY
// #include "tf/transform_datatypes.h"
// #include "tf/LinearMath/btMatrix3x3.h"

#include <tf/transform_listener.h>
#include <ar_pose/ARMarkers.h>
#include <ros/package.h>

#include <algorithm>

struct ObjectStamped {
    int id;                           // Name of object
    geometry_msgs::PoseStamped pose_stamped;  // Pose of object
    int count;                        // Number of time it appeared
};

ros::Publisher   ar_pose_trans_pub;

// This list is used for denoise
std::vector<ObjectStamped> temp_object_list;
std::string relation_data_path = ros::package::getPath("bwi_object_search") + "/relation_data.txt";

ros::Duration time_threshold (10);
double distance_threshold = 1.0;
int count_threshold = 10;
double range_threshold = 10.0;             // Limit the range the robot can detect objects
double z_upper_threshold = 1.29;
double z_lower_threshold = 0.88;

// Filtering possible positions
int map_sample_increment = 15;    // The following two variables are codepedent on each other
int dense_count_threshold = 50;
double dense_radius = 1.0;

std::vector<int> seen_id_list;
std::vector<double>  seen_id_x_list;
std::vector<double>  seen_id_y_list;

std::vector<double>    distance_vector;


#include <termios.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose current_position;

std::vector<std::pair<int,int> > free_space_indices;
std::vector<double> free_space_weights;
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
  // ros::shutdown();
  exit(0);
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

    ROS_INFO("Free location size: %d\tweights size: %d", (int)free_space_indices.size(),  (int)free_space_weights.size());
}


bool isInForbiddenCircle(double circle_x, double circle_y, double circle_radius, double target_x, double target_y) {
    return (sqrt(pow(target_x - circle_x, 2) + pow(target_y - circle_y, 2)) < circle_radius);
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    current_position = msg->pose.pose;

    // Only lower weight based on location every second
    static ros::Time last_amcl_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration amcl_wait (1.0);
    if (now - last_amcl_time > amcl_wait) {
        last_amcl_time = now;

        // Cast the forbidden circle
        // Find the circle in front of the robot
        tf::Quaternion q (current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double forbidden_circle_radius = 0.5;

        double forbidden_circle_x = current_position.position.x +  forbidden_circle_radius * cos(yaw);
        double forbidden_circle_y = current_position.position.y +  forbidden_circle_radius * sin(yaw);
        // ROS_INFO("\f(%f, %f, %f)", roll, pitch, yaw);
        // ROS_INFO("\f(%f, %f, %f)", current_position.position.x, current_position.position.y, current_position.position.z);

        // TODO Try erase first
        for (auto it = free_space_indices.begin(); it != free_space_indices.end();) {
            double loc_x = MAP_WXGX(map_, it->first);
            double loc_y =  MAP_WYGY(map_, it->second);

            if (isInForbiddenCircle(forbidden_circle_x, forbidden_circle_y,  forbidden_circle_radius, loc_x, loc_y)) {
                it = free_space_indices.erase(it);
            } else {
                ++it;
            }
        }
        publishFreeSpace();

        // // Make particles in the circle in front of the robot lower prob
        // for (int i = 0; i < free_space_indices.size(); ++i) {
        //     std::pair<int,int>& fsc = free_space_indices[i];
        //     double loc_x = MAP_WXGX(map_, fsc.first);
        //     double loc_y =  MAP_WYGY(map_, fsc.second);
        //     // If point is in circle, lower weight
        //     if (isInForbiddenenCircle(forbidden_circle_x, forbidden_circle_y, forbidden_circle_radius, loc_x, loc_y)) {
        //         free_space_weight[i] = free_space_weight_min;
        //     }
        // }
    }
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
    for(int i=0;i<map->size_x * map->size_y;i+=map_sample_increment) // Down sampled
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
    for(int i = 0; i < map_->size_x; i++) {
        for(int j = 0; j < map_->size_y; j++) {
            if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1) {
                free_space_indices.push_back(std::make_pair(i,j));
                // free_space_weights.push_back(1.0);
            }
        }
    }

    // Remove all regions not dense enough
    for (auto it = free_space_indices.begin(); it != free_space_indices.end();) {
        double self_x = MAP_WXGX(map_, it->first);
        double self_y =  MAP_WYGY(map_, it->second);

        int neighbor_count = 0;

        // Find the number of points within a radius
        for (auto iit = free_space_indices.begin(); iit != free_space_indices.end(); ++iit) {
            double loc_x = MAP_WXGX(map_, iit->first);
            double loc_y =  MAP_WYGY(map_, iit->second);
            if (loc_x < (self_x + dense_radius) &&
                loc_x > (self_x - dense_radius) &&
                loc_y < (self_y + dense_radius) &&
                loc_y > (self_y - dense_radius)) {
                neighbor_count++;
            }
        }

        // Remove ones that are too remote since they are likely to be outliers
        if (neighbor_count < dense_count_threshold) {
            it = free_space_indices.erase(it);
        } else {
            ++it;
        }
    }

    // Then add the weights
    for (auto it = free_space_indices.begin(); it != free_space_indices.end(); ++it) {
        free_space_weights.push_back(1.0);
    }
}


void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    ROS_INFO("Map received");

    // Only get map once
    if (has_map) {
        publishFreeSpace();
        return;
    }

    handleMapMessage(*msg);

    publishFreeSpace();
    has_map = true;
    ROS_INFO("Map processing done");
}

void processing (const ar_pose::ARMarkers::ConstPtr& msg) {
    ar_pose::ARMarker ar_pose_marker;

    for (int i=0; i < msg->markers.size(); i++) {
      ar_pose_marker = msg->markers.at(i);

      // ROS_INFO("frame_id: %s", ar_pose_marker.header.frame_id.c_str());
      // ROS_INFO("id: %d", ar_pose_marker.id);

      // std::cout<<"(" << ar_pose_marker.pose.pose.position.x;
      // std::cout<<", " << ar_pose_marker.pose.pose.position.y;
      // std::cout<<", " << ar_pose_marker.pose.pose.position.z;
      // std::cout << ")" << std::endl;

      static tf::TransformListener listener;

      std::string map_frame = "level_mux/map";

      try {
          geometry_msgs::PoseStamped pose_before;
          geometry_msgs::PoseStamped pose_transformed;
          pose_before.header = ar_pose_marker.header;
          pose_before.pose = ar_pose_marker.pose.pose;

          // Wait until the transform is ready
          // listener.waitForTransform(pose_before.header.frame_id, map_frame,
          //                           pose_before.header.stamp, ros::Duration(3.0));

          // listener.transformPose(map_frame,
          //                        pose_before,
          //                        pose_transformed);

          // This grabs the map's pose time 2 seconds before (rate at which
          // it publishes to tf. This is okay because map is essentially fixed
          // static time static_stamp = pose_before.header.stamp;
          listener.transformPose(map_frame,
                                 pose_before.header.stamp - ros::Duration(3),
                                 // static_stamp,
                                 pose_before,
                                 pose_before.header.frame_id,
                                 pose_transformed);

          // Set detection range
          if (pose_before.pose.position.z > range_threshold) {
              continue;
          }

          // Set global z threshold
          if (pose_transformed.pose.position.z > z_upper_threshold ||
              pose_transformed.pose.position.z < z_lower_threshold) {
              continue;
          }
          // ROS_INFO("\t(%f, %f, %f)", pose_transformed.pose.position.x,
          //          pose_transformed.pose.position.y,
          //          pose_transformed.pose.position.z);


          // Process the temp list to pick out both comfirmed objects and
          // remove stale objects
          for (auto it = temp_object_list.begin(); it != temp_object_list.end();) {
             if (pose_transformed.header.stamp - it->pose_stamped.header.stamp > time_threshold) {
                 // Only Save object loc when it becomes stale.
                 // This avoids the issue of bias towards object with more screen time
                 // higher weight
                 if (it->count > count_threshold) {
                     // TODO move element to data container
                     // std::time_t result = std::time(nullptr);
                     std::cout << "(" << it->id << ":" << it->count << ") Observed" << std::endl;
                     if (it->id == 0) {
                         ROS_INFO("Object found!");
                         exit(0);
                     }
                     if(std::find(seen_id_list.begin(), seen_id_list.end(), it->id) != seen_id_list.end()) {
                         /* v contains x */
                         std::cout << "Seen it" << std::endl;
                     } else {
                         /* v does not contain x */
                         seen_id_list.push_back(it->id);
                         seen_id_x_list.push_back(it->pose_stamped.pose.position.x);
                         seen_id_y_list.push_back(it->pose_stamped.pose.position.y);
                     }

                 }
                 it = temp_object_list.erase(it);
             } else {
                 ++it;
             }
          }

          // Search to see if the objec is already in list
          bool has_added = false;
          for (int j = 0; j < temp_object_list.size(); ++j) {
              ObjectStamped& os = temp_object_list[j];
              if (os.id == ar_pose_marker.id) {
                  // Check Euclidean distance distance
                  double distance = sqrt(pow(pose_transformed.pose.position.x - os.pose_stamped.pose.position.x, 2) +
                                         pow(pose_transformed.pose.position.y - os.pose_stamped.pose.position.y, 2) +
                                         pow(pose_transformed.pose.position.z - os.pose_stamped.pose.position.z, 2));
                  if (distance < distance_threshold) {
                      // Update timestamp and loc with header to account for moving object
                      os.pose_stamped = pose_transformed;
                      os.count++;
                      has_added = true;
                      break;
                  }
              }
          }
          // We don't have one that meets the requiements for association, add it
          // Note that two object can have the same id at different locations at once
          if (!has_added) {
              ObjectStamped os;
              os.id = ar_pose_marker.id;
              os.pose_stamped = pose_transformed;
              os.count = 0;
              temp_object_list.push_back(os);
          }

          // ROS_INFO("\t(%f, %f, %f)", pose_transformed.pose.position.x,
          //          pose_transformed.pose.position.y,
          //          pose_transformed.pose.position.z);

          // Debug
          for (auto& os : temp_object_list) {
              std::cout << "(" << os.id << ":" << os.count << ")-->";
          }
          std::cout << std::endl;
      } catch (tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
      }
    }
}



int main(int argc, char **argv)
{
    srand(time(NULL));

    std::mt19937 gen(std::time(0));

    // TODO: YAML
    // Some predefined locations
    std::vector<std::pair<double, double> > locations; // (x, y)
    // locations.emplace_back(-35.0, -11.5);   // In front of my computer
    // locations.emplace_back(-30.03, -4.73);  // Kitchen
    // locations.emplace_back(-47.67, -7.75);  // Robot soccer field
    // locations.emplace_back(-19.18, -4.95);  // Printer
    // locations.emplace_back(-13.97, -11.90); // North front student desk intersection
    locations.emplace_back(-30.22, -11.56);
    locations.emplace_back(-13.88, -11.88);
    locations.emplace_back(-8.21, -11.38);
    locations.emplace_back(-14.00, -9.05);
    locations.emplace_back(-14.00, -4.99);
    locations.emplace_back(-8.38, -6.07);
    locations.emplace_back(-16.76, -4.22);
    // locations.emplace_back(-14.19, -1.37);
    // locations.emplace_back(-14.19, -1.37);
    // locations.emplace_back(-14.19, -1.37);

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

    // Create a ROS subscriber
    // ros::Subscriber sub = nh.subscribe ("ar_pose_marker", 1, processing);

    // ros::spin();

    amcl_pose_sub = nh.subscribe("amcl_pose", 100, amclPoseCallback);
    map_sub = nh.subscribe(map_frame, 1, mapCallback);

    double theta = M_PI;
    int last_location_num = -1;
    bool keypress_exit = false;

    ROS_INFO("Start");

    do {
        ros::spinOnce();
        ROS_INFO("Waiting for map...");
        ros::Duration(1.0).sleep();
    } while (!has_map);

    ROS_INFO("Got map, running main loop");



    ros::spin();


    // Spin
    ROS_INFO("Spinning");
    geometry_msgs::Twist rotate;
    rotate.angular.z = 0.3;

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(15.0); // Timeout of 2 seconds
    while(ros::Time::now() - start_time < timeout) {
        cmd_vel_pub.publish(rotate);
        ros::spinOnce();
    }

    // Read file and build relation
    std::ifstream          file("/home/users/wxie/catkin_ws/src/bwi_will/bwi_object_search/relation_data_bak.txt");
    std::string   line;
    std::vector<std::vector<int> >     data;

    // Read one line at a time into the variable line:
    while(std::getline(file, line))
    {
        std::vector<int>   lineData;

        std::istringstream ss(line);
        std::string token;
        while(std::getline(ss, token, ' ')) {
            float token_f = stof(token);
            lineData.push_back(token_f);
        }

        data.push_back(lineData);

        // std::stringstream  lineStream(line);

        // int value;
        // // Read an integer at a time from the line
        // while(lineStream >> value)
        // {
        //     // Add the integers from a line to a 1D array (vector)
        //     lineData.push_back(value);
        // }
        // // When all the integers have been read, add the 1D array
        // // into a 2D array (as one line in the 2D array)

        // for (auto& butt : lineData) {
        //     std::cout << butt << std::endl;
        // }
        // std::cout  << std::endl;
    }

    distance_vector.push_back(0); // Dummy for the target
    double target_x = -12.3969;
    double target_y = -15.9015;
    for (int l = 1; l < 16; ++l) {
        double total_distance = 0;
        int count = 0;
        for(auto& kk : data) {
            // // Skip itself
            // if (kk[1] == 0)
            //     continue;
            if (kk[1] != l) {
                continue;
            }
            // 0 time, 1 id, 2 x, 3, y, 4 z
            double x = kk[2];
            double y = kk[3];
            double distance = sqrt(pow(x - target_x, 2) + pow(y - target_x, 2));
            total_distance += distance;
            count++;
            // for (auto&k : kk) {
            //     std::cout << k << " ";
            // }
            // std::cout << std::endl;
        }
        if (count == 0) {
            distance_vector.push_back(0);
        } else {
            distance_vector.push_back(total_distance / count);
        }
    }

    for (auto distance : distance_vector)
    {
        std::cout << distance << " ";
    }
    std::cout << std::endl;

    while (ros::ok()) {
        // Filter out target poses
        // std::vector<std::pair<int,int> > free_space_indices;
    // std::vector<double>    distance_vector;
// std::vector<int> seen_id_list
// std::vector<double>  seen_id_x_list;
// std::vector<double>  seen_id_y_list;
        double distance_bound = 3.0;
        for (int id_index = 0; id_index < seen_id_list.size(); ++id_index) {
            double target_distance = distance_vector[seen_id_list[id_index]];
            if (target_distance == 0) {
                continue;
            }
            ROS_INFO("Trying to eliminate target points...");
            double target_x = seen_id_x_list[id_index];
            double target_y = seen_id_y_list[id_index];

            for (auto it = free_space_indices.begin(); it != free_space_indices.end();) {
                double x = MAP_WXGX(map_, it->first);
                double y =  MAP_WYGY(map_, it->second);
                double distance = sqrt(pow(x - target_x, 2) + pow(y - target_x, 2));

                if (distance > target_distance + distance_bound ||
                    distance < target_distance - distance_bound) {
                    it = free_space_indices.erase(it);
                } else {
                    ++it;
                }
            }
        }
        std::cout << "seen_id_list size: " << seen_id_list.size() << std::endl;
        for (auto& id : seen_id_list) {
            std::cout << id << " ";
        }
        std::cout<<std::endl;
        // TODO Clear lists
        publishFreeSpace();
        ros::spinOnce();

////////////////////

        // Different location each time
        // int location_num = rand() % locations.size();
        // while (location_num == last_location_num) {
        //     location_num = rand() % locations.size();
        // }
        // last_location_num = location_num;

        // std::pair<double, double>& new_location = locations[location_num];
        // ROS_INFO("Destination: #%d", location_num);


        // Pick a random point based on weight
        std::discrete_distribution<int> distribution (free_space_weights.begin(), free_space_weights.end());
        int sampled_index = distribution(gen); // Sample without replacement
        std::pair<int,int> fsc = free_space_indices[sampled_index];
        double loc_x = MAP_WXGX(map_, fsc.first);
        double loc_y =  MAP_WYGY(map_, fsc.second);
        std::pair<double, double> new_location = std::make_pair<double,double>((double)loc_x, (double)loc_y);

        // // Pick a random location uniformly distributed on the map
        // // std::piecewise_constant_distribution<> d(free_space_indices.begin(), free_space_indices.end(), free_space_weights.begin());
        // std::random_shuffle ( free_space_indices.begin(), free_space_indices.end() );
        // std::pair<int,int> fsc = free_space_indices.back();
        // free_space_indices.pop_back(); // TODO replacement or not?
        // double loc_x = MAP_WXGX(map_, fsc.first);
        // double loc_y =  MAP_WYGY(map_, fsc.second);
        // std::pair<double, double> new_location = std::make_pair<double,double>((double)loc_x, (double)loc_y);
        // ROS_INFO("Destination: (%f, %f)", loc_x, loc_y);

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

        while (!ac.waitForResult(ros::Duration(0.01))) {
            ros::spinOnce();
        }


        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Move success");
        else
            ROS_INFO("Move failed");

        ros::Duration(1).sleep();

        // Spin
        ROS_INFO("Spinning");
        geometry_msgs::Twist rotate;
        rotate.angular.z = 0.3;

        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(5.0); // Timeout of 2 seconds
        while(ros::Time::now() - start_time < timeout) {
            cmd_vel_pub.publish(rotate);
            ros::spinOnce();
        }

        rotate.angular.z = 0.0;
        cmd_vel_pub.publish(rotate);
        cmd_vel_pub.publish(rotate);
        cmd_vel_pub.publish(rotate);
        ros::Duration(1).sleep();

        ROS_INFO("Done.");

        ros::spinOnce();
    }

    ROS_INFO("Gracefully exiting");
    return 0;
}
