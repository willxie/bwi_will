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

#include <boost/accumulators/accumulators.hpp>

#include <boost/accumulators/statistics.hpp>

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

struct ObjectDistribution {
    int id;
    double mean_x;
    double mean_y;
    double var_x;
    double var_y;
    double distance;
    bool seen;
};

ros::Publisher   ar_pose_trans_pub;

// This list is used for denoise
std::vector<ObjectStamped> temp_object_list;
std::string relation_data_path = ros::package::getPath("bwi_object_search") + "/relation_data_fixed.txt";

ros::Duration time_threshold (2);
double distance_threshold = 1.0;
int count_threshold = 20;
double range_max_threshold = 3.00;            // Limit the range the robot can detect objects
double range_min_threshold = 0.25;             // Limit the range the robot can detect objects
double z_upper_threshold = 1.29;
double z_lower_threshold = 0.88;

// Filtering possible positions
int map_sample_increment = 15;    // The following two variables are codepedent on each other
int dense_count_threshold = 50;
double dense_radius = 1.0;

// Weight adjustments
double free_space_weight_start = 1.0;
double free_space_weight_min = 0.1;

// Object data
double variance_avg_max_threshold  = 1.00;
double variance_avg_min_threshold  = 0.01;

double target_id = 0;
double target_x = 0;
double target_y = 0;
// double target_x = -12.3969;
// double target_y = -15.9015;


std::vector<ObjectDistribution> object_distribution_list;

std::vector<int> seen_id_list;
std::vector<double>  seen_id_x_list;
std::vector<double>  seen_id_y_list;
std::vector<double>    distance_vector;

clock_t begin, end;


#include <termios.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose current_position;
double current_distance = 0;

std::vector<std::pair<int,int> > free_space_indices;
std::vector<double> free_space_weights;
std::string map_frame = "level_mux/map";
// std::string map_frame = "map";

map_t* map_;
bool has_map = false;

// Keep track of runtime
ros::Time init_time;

bool searching = false;

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

double calculateGaussianValue(double m, double s, double x) {
    return ( 1 / ( s * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-m)/s, 2.0 ) );
}

void publishFreeSpace() {
    // Publish all free locations
    geometry_msgs::PoseArray cloud_msg, cloud_msg_2;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = map_frame;
    cloud_msg.poses.resize(free_space_indices.size());
    for (int i = 0; i < free_space_indices.size(); ++i) {
        // Don't visualize destinations with low / min prob
        if (free_space_weights[i] <= 2*free_space_weight_min) {
            continue;
        }
        auto& loc = free_space_indices[i];

        // ROS_INFO("(%d, %d)", loc.first, loc.second);
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(0),
                                 tf::Vector3(MAP_WXGX(map_, loc.first),
                                             MAP_WYGY(map_, loc.second),
                                             0)),
                        cloud_msg.poses[i]);
    }

    // This is for high weight only
    cloud_msg_2.header.stamp = ros::Time::now();
    cloud_msg_2.header.frame_id = map_frame;
    cloud_msg_2.poses.resize(free_space_indices.size());
    for (int i = 0; i < free_space_indices.size(); ++i) {
        // Don't visualize destinations with low / min prob
        if (free_space_weights[i] <= 1.5) {
            continue;
        }
        auto& loc = free_space_indices[i];

        // ROS_INFO("(%d, %d)", loc.first, loc.second);
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(0),
                                 tf::Vector3(MAP_WXGX(map_, loc.first),
                                             MAP_WYGY(map_, loc.second),
                                             0)),
                        cloud_msg_2.poses[i]);
    }

    loc_free_pub.publish(cloud_msg);
    loc_visited_pub.publish(cloud_msg_2);

    // ROS_INFO("Free location size: %d\tweights size: %d", (int)free_space_indices.size(),  (int)free_space_weights.size());
}


bool isInForbiddenCircle(double circle_x, double circle_y, double circle_radius, double target_x, double target_y) {
    return (sqrt(pow(target_x - circle_x, 2) + pow(target_y - circle_y, 2)) < circle_radius);
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    static bool first_time = true;
    // Keep track of distance traveled
    if (first_time) {
        first_time = false;
    } else{
        current_distance += sqrt(pow(msg->pose.pose.position.x - current_position.position.x, 2) + pow(msg->pose.pose.position.y - current_position.position.y, 2));

    }
    current_position = msg->pose.pose;
    // std::cout << "current_distance: " << current_distance << std::endl;

                          end = clock();

                          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

                          std::cout<< "elapsed_secs: " << elapsed_secs<<std::endl;

                          std::cout << "current_distance: " << current_distance << std::endl;



    // Only lower weight based on location every second
    static ros::Time last_amcl_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration amcl_wait (1.0);
    // Searching is a bool set by the behavior because we don't want the robot to eat up
    // all the position particles while driving in at the wrong direction
    if (now - last_amcl_time > amcl_wait) {
        last_amcl_time = now;

        // Cast the forbidden circle
        // Find the circle in front of the robot
        tf::Quaternion q (current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double forbidden_circle_radius = 1.0;

        double forbidden_circle_x = current_position.position.x +  forbidden_circle_radius * cos(yaw);
        double forbidden_circle_y = current_position.position.y +  forbidden_circle_radius * sin(yaw);
        // ROS_INFO("\f(%f, %f, %f)", roll, pitch, yaw);
        // ROS_INFO("\f(%f, %f, %f)", current_position.position.x, current_position.position.y, current_position.position.z);

        // // Try erase first
        // for (auto it = free_space_indices.begin(); it != free_space_indices.end();) {
        //     double loc_x = MAP_WXGX(map_, it->first);
        //     double loc_y =  MAP_WYGY(map_, it->second);

        //     if (isInForbiddenCircle(forbidden_circle_x, forbidden_circle_y,  forbidden_circle_radius, loc_x, loc_y)) {
        //         it = free_space_indices.erase(it);
        //     } else {
        //         ++it;
        //     }
        // }
        // publishFreeSpace();

        // Make particles in the circle in front of the robot lower prob
        for (int i = 0; i < free_space_indices.size(); ++i) {
            assert(free_space_indices.size() == free_space_weights.size());

            std::pair<int,int>& fsc = free_space_indices[i];

            double loc_x = MAP_WXGX(map_, fsc.first);
            double loc_y =  MAP_WYGY(map_, fsc.second);

            if (isInForbiddenCircle(forbidden_circle_x, forbidden_circle_y,  forbidden_circle_radius, loc_x, loc_y)) {
                // free_space_weights[i] = free_space_weight_min;
            }
        }
        publishFreeSpace();

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
        free_space_weights.push_back(free_space_weight_start);
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

    // publishFreeSpace();

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
          // Set detection range
          if (pose_before.pose.position.z > range_max_threshold ||
              pose_before.pose.position.z < range_min_threshold ) {
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
                      // Found the object, we are done
                      if (it->id == target_id) {
                          ROS_INFO("Object found!");

                          end = clock();

                          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

                          std::cout<< "elapsed_secs: " << elapsed_secs<<std::endl;

                          std::cout << "current_distance: " << current_distance << std::endl;

                         // ros::Time::now() - init_time);
                         exit(0);
                     }

                     // Found something else
                     for (auto& od :object_distribution_list) {
                         if (od.seen) {
                             continue;
                         }
                         if (od.id != it->id) {
                             continue;
                         }
                         ROS_INFO("Landmark found! ID = %d", it->id);
                         double stdev = sqrt((od.var_x + od.var_y) / 2);   // Average standard deviation
                         double v_gaussian = calculateGaussianValue(od.distance, stdev, od.distance);
                         for (int i = 0; i < free_space_indices.size(); ++i) {
                             assert(free_space_indices.size() == free_space_weights.size());

                             std::pair<int,int>& fsc = free_space_indices[i];

                             double loc_x = MAP_WXGX(map_, fsc.first);
                             double loc_y =  MAP_WYGY(map_, fsc.second);

                             // Add weight based on distance from the tag
                             // V is expected from relationship data, p is position particle distance from the beacon
                             // double p_distance = sqrt(pow(loc_x - od.mean_x, 2) + pow(loc_y - od.mean_y, 2));
                             double p_distance = sqrt(pow(loc_x - it->pose_stamped.pose.position.x, 2) + pow(loc_y - it->pose_stamped.pose.position.y, 2));
                             double p_gaussian = calculateGaussianValue(p_distance, stdev, od.distance);

                             //     free_space_weights[i] = free_space_weight_min;
                             // Score normalized to 1
                             double distance_weight = (v_gaussian - fabs(p_gaussian - v_gaussian)) / v_gaussian;
                             // Let's double that
                             distance_weight *= 10;

                             // free_space_weights[i] += distance_weight;
                         }
                         publishFreeSpace();
                     }
////////////////////////////////////////
                     // if (std::find(seen_id_list.begin(), seen_id_list.end(), it->id) != seen_id_list.end()) {
                     //     /* v contains x */
                     //     std::cout << "Seen it" << std::endl;
                     // } else {
                     //     /* v does not contain x */
                     //     seen_id_list.push_back(it->id);
                     //     seen_id_x_list.push_back(it->pose_stamped.pose.position.x);
                     //     seen_id_y_list.push_back(it->pose_stamped.pose.position.y);
                     // }

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

          // // Debug
          // for (auto& os : temp_object_list) {
          //     std::cout << "(" << os.id << ":" << os.count << ")-->";
          // }
          // std::cout << std::endl;
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

    std::vector<std::pair<double, double> > locations; // (x, y)

    ros::init(argc, argv, "object_search_find");
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
    ros::Subscriber sub = nh.subscribe ("ar_pose_marker", 1, processing);

    // ros::spin();

    amcl_pose_sub = nh.subscribe("amcl_pose", 100, amclPoseCallback);
    map_sub = nh.subscribe(map_frame, 1, mapCallback);

    double theta = M_PI;
    int last_location_num = -1;
    bool keypress_exit = false;

    ROS_INFO("Start");

    current_distance = 0;

    // Keep track of time
    begin = clock();


    // Read file and build relation
    std::ifstream          file(relation_data_path);
    std::string   line;
    std::vector<std::vector<int> >     data;

    // Read raw datat into vectors
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


// struct ObjectDistribution {
//     double mean_x;
//     double mean_y;
//     double var_x;
//     double var_y;
// };

    for (int id = 0; id < 16; ++id) {
        using namespace boost::accumulators;
        accumulator_set< double, stats<tag::variance> > acc_x;
        accumulator_set< double, stats<tag::variance> > acc_y;
        int count;
        // Calculate mean and variance for each id
        for(auto& lineData : data) {
            int line_id = lineData[1];
            if (id != line_id) {
                continue;
            }
            double x = lineData[2];
            double y = lineData[3];
            acc_x(x);
            acc_y(y);
            count++;
        }
        printf("ID: %d \t count: %d \t mean: (%.2f, %.2f) \t var: (%.2f, %.2f)\n", id, count, mean(acc_x), mean(acc_y), variance(acc_x), variance(acc_y));

        double avg_var = (variance(acc_x) + variance(acc_y)) / 2;
        if (avg_var < variance_avg_max_threshold &&
            avg_var > variance_avg_min_threshold) {
            ObjectDistribution od;
            od.id = id;
            od.mean_x = mean(acc_x);
            od.mean_y = mean(acc_y);
            od.var_x  = variance(acc_x);
            od.var_y  = variance(acc_y);
            od.seen = false;
            od.distance = -1;
            object_distribution_list.push_back(od);
        }
    }

    // Get values for target if target passes the variance test
    for (auto& od : object_distribution_list) {
        if (od.id == target_id) {
            target_x = od.mean_x;
            target_y = od.mean_y;
            break;
        }
    }
    std::cout << "Target id: " << target_id << "  = " <<target_x << ", " << target_y << std::endl;

    // Loop through each landmark we can use
    for (auto& od : object_distribution_list) {
        if (od.id == target_id) {
            continue;
        }
        od.distance = sqrt(pow(od.mean_x - target_x, 2) + pow(od.mean_y - target_x, 2));
        std::cout << od.id << ":" << od.distance << ", ";
    }
    std::cout << std::endl;


    // for (int id = 1; id < 16; ++l) {
    //     double total_distance = 0;
    //     int count = 0;
    //     // Search data for the id
    //     for(auto& kk : data) {

    //         if (kk[1] != id) {
    //             continue;
    //         }
    //         // 0 time, 1 id, 2 x, 3, y, 4 z
    //         double x = kk[2];
    //         double y = kk[3];
    //         double distance = sqrt(pow(x - target_x, 2) + pow(y - target_x, 2));
    //         total_distance += distance;
    //         count++;
    //         // for (auto&k : kk) {
    //         //     std::cout << k << " ";
    //         // }
    //         // std::cout << std::endl;
    //     }
    //     if (count == 0) {
    //         distance_vector.push_back(0);
    //     } else {
    //         distance_vector.push_back(total_distance / count);
    //     }
    // }

    // for (auto distance : distance_vector)
    // {
    //     std::cout << distance << " ";
    // }
    // std::cout << std::endl;

    // ros::spin();

    do {
        ros::spinOnce();
        ROS_INFO("Waiting for map...");
        ros::Duration(1.0).sleep();
    } while (!has_map);

    ROS_INFO("Got map, running main loop");

    // // Spin
    // ROS_INFO("Spinning");
    // geometry_msgs::Twist rotate;
    // rotate.angular.z = 0.3;

    // ros::Time start_time = ros::Time::now();
    // ros::Duration timeout(15.0); // Timeout of 2 seconds
    // while(ros::Time::now() - start_time < timeout) {
    //     cmd_vel_pub.publish(rotate);
    //     ros::spinOnce();
    // }

    while (ros::ok()) {
        // Filter out target poses
        // std::vector<std::pair<int,int> > free_space_indices;
    // std::vector<double>    distance_vector;
// std::vector<int> seen_id_list
// std::vector<double>  seen_id_x_list;
// std::vector<double>  seen_id_y_list;

        // // Remove positions that don't fit in the distance vector
        // double distance_bound = 3.0;
        // for (int id_index = 0; id_index < seen_id_list.size(); ++id_index) {
        //     double target_distance = distance_vector[seen_id_list[id_index]];
        //     if (target_distance == 0) {
        //         continue;
        //     }
        //     ROS_INFO("Trying to eliminate target points...");
        //     double target_x = seen_id_x_list[id_index];
        //     double target_y = seen_id_y_list[id_index];

        //     for (auto it = free_space_indices.begin(); it != free_space_indices.end();) {
        //         double x = MAP_WXGX(map_, it->first);
        //         double y =  MAP_WYGY(map_, it->second);
        //         double distance = sqrt(pow(x - target_x, 2) + pow(y - target_x, 2));

        //         if (distance > target_distance + distance_bound ||
        //             distance < target_distance - distance_bound) {
        //             it = free_space_indices.erase(it);
        //         } else {
        //             ++it;
        //         }
        //     }
        // }
        // std::cout << "seen_id_list size: " << seen_id_list.size() << std::endl;
        // for (auto& id : seen_id_list) {
        //     std::cout << id << " ";
        // }
        // std::cout<<std::endl;
        // // TODO Clear lists
        // publishFreeSpace();
        // ros::spinOnce();

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

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Move success");
        } else{
            ROS_INFO("Move failed");
            // Wait a bit
            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(5.0); // Timeout of 2 seconds
            while(ros::Time::now() - start_time < timeout) {

                ros::spinOnce();
            }
            continue;
        }

        ros::Duration(1).sleep();

        searching = true;

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

        searching = false;
    }

    ROS_INFO("Gracefully exiting");
    return 0;
}
