#include <ros/ros.h>
#include <ar_pose/ARMarkers.h>

void processing (const ar_pose::ARMarkers::ConstPtr& msg){

    ar_pose::ARMarker ar_pose_marker;

    for(int i=0; i < msg->markers.size();i++){
      ar_pose_marker = msg->markers.at(i);
      std::cout<<"x " << ar_pose_marker.pose.pose.position.x<<std::endl;
      std::cout<<"y " << ar_pose_marker.pose.pose.position.y<<std::endl;
      std::cout<<"z " << ar_pose_marker.pose.pose.position.z<<std::endl;

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
