#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
int main(int argc, char** argv){
  ros::init(argc, argv, "connecting_tf");
  ros::NodeHandle node("~");
  std::string frame1,frame2;

  if(not(node.hasParam("frame1_id") and node.hasParam("frame2_id"))){
    ROS_ERROR("frame id was not provided");
	exit(-1);
  }

  node.getParam("frame1_id",frame1);
  node.getParam("frame2_id",frame2);



  ROS_INFO_STREAM("tf connecting from "<<frame1<<" to "<<frame2);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(100);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.01) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),frame1, frame2 ));
    rate.sleep();
  }
  return 0;
};


