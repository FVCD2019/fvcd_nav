#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double c_x = 0.0;
  double c_y = 0.0;
  double c_th = 0.0;
  double p_x = 0.0;
  double p_y = 0.0;
  double p_th = 0.0;

  double vx = 0.0;
  double vy = -0.0;
  double vth = 0.0;

  int nu = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    if (nu < 100){
		c_x +=2.0;
	    c_y += 2.0;
	    c_th = 0.5;
		nu++;
	}
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    vth = atan2((c_x - p_x),(c_y - p_y));
    vx = sqrt(pow((c_x - p_x),2) + pow((c_y - p_y),2)) * sin(vth);
    vy = sqrt(pow((c_x - p_x),2) + pow((c_y - p_y),2)) * cos(vth);

    p_x = c_x;
    p_y = c_y;
    p_th = c_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(c_th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = c_x;
    odom_trans.transform.translation.y = c_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = c_x;
    odom.pose.pose.position.y = c_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
