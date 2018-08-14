#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <string>
int main(int argc, char** argv){
  ros::init(argc, argv, "connecting_tf");
  ros::NodeHandle node("~");
  std::string frame1,frame2;
  // transformation from frame1 to frame2
  float x,y,z,R,P,Y;


  if(not(node.hasParam("frame1_id") and node.hasParam("frame2_id"))){
    ROS_ERROR("frame id was not provided");
	exit(-1);
  }

  node.getParam("frame1_id",frame1);
  node.getParam("frame2_id",frame2);
  node.getParam("x",x);
  node.getParam("y",y);
  node.getParam("z",z);
  node.getParam("R",R);
  node.getParam("P",P);
  node.getParam("Y",Y);



  ROS_INFO_STREAM("tf connecting from "<<frame1<<" to "<<frame2);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(100);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(x, y, z) );
	tf::Quaternion q=tf::createQuaternionFromRPY(R,P,Y);
	transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),frame1, frame2 ));
    rate.sleep();
  }
  return 0;
};


