#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>
#include <ar_pose/ARMarkers.h>
#include <time.h>

void processing (const ar_pose::ARMarkers::ConstPtr& msg){
    ar_pose::ARMarker ar_pose_marker;

    for (int i=0; i < msg->markers.size(); i++) {
      ar_pose_marker = msg->markers.at(i);

      // ROS_INFO("frame_id: %s", ar_pose_marker.header.frame_id.c_str());
      // ROS_INFO("id: %d", ar_pose_marker.id);

      std::cout<<"(" << ar_pose_marker.pose.pose.position.x;
      std::cout<<", " << ar_pose_marker.pose.pose.position.y;
      std::cout<<", " << ar_pose_marker.pose.pose.position.z;
      std::cout << ")" << std::endl;

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
          // it publishes to tf
          listener.transformPose(map_frame,
                                 pose_before.header.stamp - ros::Duration(3),
                                 pose_before,
                                 pose_before.header.frame_id,
                                 pose_transformed);

          std::cout<<"\t(" << pose_transformed.pose.position.x;
          std::cout<<", " << pose_transformed.pose.position.y;
          std::cout<<", " << pose_transformed.pose.position.z;
          std::cout << ")" << std::endl;

      } catch (tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
      }

      // tf::StampedTransform transform;


    //   try {
    //       listener.lookupTransform("/turtle2", map_frame,
    //                                ros::Time(0), transform);
    //   } catch (tf::TransformException ex){
    //       ROS_ERROR("%s", ex.what());
    //       ros::Duration(1.0).sleep();
    //   }
    }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ar_object_transform");
  ros::NodeHandle nh;


  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe ("ar_pose_marker", 1, processing);

  ros::spin ();
}
