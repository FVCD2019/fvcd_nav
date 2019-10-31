#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


class Odom
{
public:
	Odom();
	void cmdCB(const geometry_msgs::Twist& msg);
	void run();
private:
	ros::NodeHandle n;
	ros::Publisher odom_pub;
//for sim
	ros::Subscriber cmd_vel;
	nav_msgs::Odometry odom;
	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;
};

Odom::Odom()
{
	ros::NodeHandle n;
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	cmd_vel = n.subscribe("/cmd_vel", 1, &Odom::cmdCB, this);

}
void Odom::cmdCB(const geometry_msgs::Twist& msg)
{
  double cx = msg.linear.x;
  double cy = msg.linear.y;
  double ang = msg.angular.z;

    x += cx;
    y += cy;
    th += ang;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    //odom.twist.twist.linear.x = vx;
    //odom.twist.twist.linear.y = vy;
    //odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

}

void Odom::run()
{
/*
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
*/

}
int main(int argc, char** argv){
	ros::init(argc, argv, "odometry_publisher");
	Odom Odom;

	ros::Rate rate(1);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;


}
