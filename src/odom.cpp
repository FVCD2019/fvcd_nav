#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

float init_pose_x = 5.5;
float init_pose_y = 1.0;
bool first_chk = false;
int count = 0;

class Odom
{
public:
	Odom();
	void poseCB(const std_msgs::Float32MultiArray::ConstPtr& msg);
	nav_msgs::Odometry getOdom();
	void run();
private:
	ros::NodeHandle n;
	ros::Publisher odom_pub;
//for sim
	ros::Subscriber v_pose;
	nav_msgs::Odometry odom;
	ros::Time current_time, last_time;
	float pose_x = 0.0;
	float pose_y = 0.0;
	float pose_heading = 0.0;
};

Odom::Odom()
{
	ros::NodeHandle n;
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	v_pose = n.subscribe("/detector/pose", 1, &Odom::poseCB, this);
	odom.pose.pose.orientation.x = 0;
	odom.pose.pose.orientation.y = 0;
	odom.pose.pose.orientation.z = 0;
	odom.pose.pose.orientation.w = 1;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	first_chk = false;
}

nav_msgs::Odometry Odom::getOdom()
{
  return odom;
}

void Odom::poseCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  	if(count < 5){
		count++;
	}
	else if(count == 5){
		init_pose_x = msg->data[0];
  		init_pose_y = msg->data[1];
		first_chk = true;
  	}
	else{
		pose_x = msg->data[0];
  		pose_y = msg->data[1];
		pose_heading = msg->data[2];
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_heading);

		//next, we'll publish the odometry message over ROS
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = pose_x;
		odom.pose.pose.position.y = pose_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//publish the message
		odom_pub.publish(odom);
		last_time = current_time;
	}
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    Odom Odom;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate rate(10);
    while(ros::ok()){
		// map to odom tf
		transform.setOrigin( tf::Vector3(init_pose_x, init_pose_y, 0.0) );
		transform.setRotation( tf::Quaternion(0, 0, 1, 1) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

		nav_msgs::Odometry current_odom = Odom.getOdom();

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = current_odom.pose.pose.position.x;
		odom_trans.transform.translation.y = current_odom.pose.pose.position.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = current_odom.pose.pose.orientation;

    	odom_broadcaster.sendTransform(odom_trans);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
