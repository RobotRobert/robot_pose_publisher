/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;

  nh_priv.param<std::string>("map_frame", map_frame, "map"); // Removed "/" in frame name
  nh_priv.param<std::string>("base_frame", base_frame, "base_link"); // Removed "/" in frame name
  nh_priv.param<double>("publish_frequency", publish_frequency, 10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  else 
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

  // create the listener
  tf::TransformListener listener;
  cout << "Waiting for transform between " << map_frame << " and " << base_frame << endl;
  try
  {
    listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      // cout << "Looking up transform between " << map_frame << " and " << base_frame << endl;
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform); // Time=0 get's the latest

      // cout << "Pulbishing pose \n";
      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped);
      else
        p_pub.publish(pose_stamped.pose);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
      cout << "robot_pose_publisher tf::TransformException &ex" << endl;
      cout << ex.what() << endl;;
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
