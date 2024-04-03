#include <cmath>
#include <iostream>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

static const int64_t kNanoSecondsInSecond = 1000000000;
bool sim_running = false;

void callback(const sensor_msgs::ImuPtr& msg) {
  sim_running = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  // Wait for simulation to become ready...
  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  ros::Duration(10).sleep();

  ROS_INFO("Start publishing waypoints.");

  ros::Rate loop_rate(10); // Rate of publishing messages

  int64_t time_from_start_ns = 0;

  while (ros::ok()) {
    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    msg->header.stamp = ros::Time::now();
    msg->points.resize(1); // Just one point in this circular trajectory
    msg->joint_names.push_back("base_link");

    double x = 3 * sin(ros::Time::now().toSec()); // X coordinate as sine of time
    double y = 3 * cos(ros::Time::now().toSec()); // Y coordinate as cosine of time
    double z = 2.0; // Constant Z coordinate

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = Eigen::Vector3d(x, y, z);
    trajectory_point.setFromYaw(0.0); // Yaw is 0 for simplicity
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(0.1 * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[0]);

    wp_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}