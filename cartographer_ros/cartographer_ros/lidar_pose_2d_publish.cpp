#include <ros/ros.h>

#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "lcm_msg/position/pose_2d.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv) {
  lcm::LCM lcm;
  if (!lcm.good()) {
    return 1;
  }

  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer, node);

  ros::Rate rate(20.0);

  while (ros::ok()) {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform(
          "odom", "horizontal_laser_link", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      rate.sleep();
      continue;
    }
    position::pose_2d pose;
    {
      // get pose.heading
      tf2::Quaternion tf2_q(transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y,
                            transformStamped.transform.rotation.z,
                            transformStamped.transform.rotation.w);
      double roll, picth;
      tf2::Matrix3x3 pose_matrix(tf2_q);
      pose_matrix.getEulerYPR(pose.heading, picth, roll);
      pose.heading = pose.heading / 3.14 * 180.0;
    }

    pose.timestamp = ros::Time::now().toSec();
    pose.x = transformStamped.transform.translation.x;
    pose.y = transformStamped.transform.translation.y;
    pose.confidence_x = 100;
    pose.confidence_y = 100;
    pose.confidence_heading = 100;

    lcm.publish("lidar_pose_2d", &pose);
    std::cout << "lidar_pose_2d: tx,ty,rz: " << pose.x << ", " << pose.y << ", "
              << pose.heading << std::endl;
    rate.sleep();
  }
  return 0;
}