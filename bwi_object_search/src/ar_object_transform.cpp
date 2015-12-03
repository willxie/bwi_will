#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>
#include <ar_pose/ARMarkers.h>
#include <ros/package.h>

struct ObjectStamped {
    int id;                           // Name of object
    geometry_msgs::PoseStamped pose_stamped;  // Pose of object
    int count;                        // Number of time it appeared
};



// This list is used for denoise
std::vector<ObjectStamped> temp_object_list;
std::string relation_data_path = ros::package::getPath("bwi_object_search") + "/relation_data.txt";

ros::Duration time_threshold (5);
double distance_threshold = 0.5;
int count_threshold = 20;
double range_threshold = 2.0;             // Limit the range the robot can detect objects
double z_upper_threshold = 1.00;
double z_lower_threshold = 0.55;


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
                                 pose_before.header.stamp - ros::Duration(5),
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
                     std::cout << "(" << it->id << ":" << it->count << ") saved" << std::endl;
                     std::ofstream relation_data;
                     relation_data.open(relation_data_path, std::ios::in | std::ios::app);
                     relation_data << it->pose_stamped.header.stamp << " ";    // Time
                     relation_data << it->id << " ";
                     relation_data << it->pose_stamped.pose.position.x << " ";
                     relation_data << it->pose_stamped.pose.position.y << " ";
                     relation_data << it->pose_stamped.pose.position.z << " ";
                     relation_data << "\n";
                     relation_data.close();
                     ROS_INFO("Saved to %s", relation_data_path.c_str());
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


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ar_object_transform");
  ros::NodeHandle nh;

  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe ("ar_pose_marker", 1, processing);

  ros::spin();
}
