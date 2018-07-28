#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "connecting_tf");
  ros::NodeHandle node;
  

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.01, 0.01, 0.01) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"vicon/zed_boseong/zed_boseong", "zed_center" ));
    rate.sleep();
  }
  return 0;
};


