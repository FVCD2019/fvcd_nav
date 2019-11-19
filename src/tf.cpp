#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

float pose_x = 5.5;
float pose_y = 1.0;


void poseCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  pose_x = msg->data[0];
  pose_y = msg->data[1];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_broadcaster");
  ros::NodeHandle node;
  //ros::Subscriber sub = n.subscribe("/detector/pose", 1, poseCB);
  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(pose_x, pose_y, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 1, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "map"));
    rate.sleep();
  }
  return 0;
};
