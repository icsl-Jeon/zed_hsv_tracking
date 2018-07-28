
/*
 * This code is for testing kcf tracker and position estimation with zed camera
 * (1) ZED sends rgb and depth image in ros (just for checking verification)
 * (2) KCF tracker gives the bounding box
 * (3) we return the position of target and compare it with Vicon
 */


#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>


using namespace cv;

// global variables


const char* src_window = "original image";

const char* mask_window = "mask image";

struct ThresHSV{

    int iLowH; //= 0;
    int iHighH; // = 179;

    int iLowS; //= 0;
    int iHighS; //= 255;

    int iLowV; //= 0;
    int iHighV; //= 255;
};

ThresHSV thres_daloka;

class TrackingManager{
private:

    const char* camera_frame;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;


    float x,y,z; //position of target w.r.t local frame of camera
    geometry_msgs::Point target_position; // target position in world frame


    // ROS
    ros::NodeHandle nh;
    ros::Publisher pos_pub;
    ros::Publisher marker_pub;
    ros::Subscriber sub_pcl2;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_rgb;

public:

    Mat rgb_mat; //RGBA
    Mat nonZeroCoordinates; // detected region (index)
    Mat imgHSV; //D
    bool callback;
    Point point1,point2; // lower left corner / upper right corner

    TrackingManager():it(nh) {
        // KCF tracker params
        cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pos_pub = nh.advertise<geometry_msgs::PointStamped>("target_position",10);
        marker_pub = nh.advertise<visualization_msgs::Marker>("target_BB",10);
        sub_rgb = it.subscribe("/zed/rgb/image_rect_color", 10,&TrackingManager::img_rgb_callback, this);
        sub_pcl2 = nh.subscribe("/zed/point_cloud/cloud_registered", 10,&TrackingManager::pcl_callback,this);

        namedWindow(src_window,CV_WINDOW_AUTOSIZE);

        // upper left corner / lower right corner
        camera_frame = "zed_left_camera";

    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& );
    void img_rgb_callback(const sensor_msgs::ImageConstPtr& );
    void get_target_pos();
    bool isReceived(){return rgb_mat.cols!=0;};
    void cur_imshow() {imshow(src_window, rgb_mat); waitKey(3);};
    void tracking_update(ThresHSV); // update rectangle by thresholding HSV (Bounding Box)
    void result_imshow();
    void result_publish();
};


int main(int argc, char **argv) {

    //ros
    ros::init(argc,argv,"kcf_tracking_node");
    ros::Time::init();
    ros::Rate rate(50);
    TrackingManager tracking_manager;


    ros::NodeHandle nh("~");

    int H_max,H_min,S_max,S_min,V_max,V_min;

    nh.getParam("H_max",H_max);
    nh.getParam("H_min",H_min);
    nh.getParam("S_max",S_max);
    nh.getParam("S_min",S_min);
    nh.getParam("V_max",V_max);
    nh.getParam("V_min",V_min);

    //
    thres_daloka.iHighH=H_max;
    thres_daloka.iLowH=H_min;

    thres_daloka.iHighS=S_max;
    thres_daloka.iLowS=S_min;

    thres_daloka.iHighV=V_max;
    thres_daloka.iLowV=V_min;


    while(ros::ok()) {

        if(tracking_manager.isReceived()) {
            tracking_manager.tracking_update(thres_daloka);
            tracking_manager.result_imshow();
            tracking_manager.get_target_pos();
            tracking_manager.result_publish();
        }
        else
            ROS_INFO_STREAM("please subscribe to img topic");
//        ros::spin(); // important !


        ros::spinOnce(); // must use spin once
        rate.sleep();
    }
    return 0;
}


void TrackingManager::tracking_update(ThresHSV thresHSV){

    // update rectangle by thresholding HSV (Bounding Box)

    cvtColor(rgb_mat, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    Mat imgThresholded;

    inRange(imgHSV, Scalar(thresHSV.iLowH, thresHSV.iLowS, thresHSV.iLowV),
            Scalar(thresHSV.iHighH, thresHSV.iHighS, thresHSV.iHighV),
            imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    waitKey(3);


    // bounding box

    Mat nonZeroCoordinates;

    findNonZero(imgThresholded, nonZeroCoordinates);
    int min_x = rgb_mat.cols+1, min_y = rgb_mat.rows+1, max_x = -1, max_y = -1;

    for (int i = 0; i < nonZeroCoordinates.total(); i++) {

        int cur_x = nonZeroCoordinates.at<Point>(i).x;
        int cur_y = nonZeroCoordinates.at<Point>(i).y;

        // extract bounding box of detected red region in this case
        if (cur_x < min_x)
            min_x = cur_x;
        if (cur_y < min_y)
            min_y = cur_y;
        if (cur_x > max_x)
            max_x = cur_x;
        if (cur_y > max_y)
            max_y = cur_y;
    }

    point1.x = min_x;
    point1.y = min_y;
    point2.x = max_x;
    point2.y = max_y;

};



// callback function for getting point cloud
void TrackingManager::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // Procedure:sensor_msgs->PCL2->PCL->extraction
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);
//    pcl::PointXYZ p1 = depth.at(x, y);

}



// retreive msg
void TrackingManager::img_rgb_callback(const sensor_msgs::ImageConstPtr& img_rgb_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
//        cv_ptr = cv_bridge::toCvCopy(img_rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(img_rgb_msg);

    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    rgb_mat = cv_ptr->image.clone();
//    std::cout<<"current size: "<<(rgb_mat.cols)<<std::endl;

}

// if we get any image after initialization, get the position of target
void TrackingManager::get_target_pos(){

    if(cloud_ptr->height==rgb_mat.rows) {

        float sum_x = 0, sum_y = 0, sum_z = 0;
        int N_pixel = 0; // num of non-nan number

        for (int j = point1.y; j < point2.y; j++)
            for (int i = point1.x; i < point2.x; i++) {
                pcl::PointXYZRGB p = cloud_ptr->at(i, j);
                if (p.x == p.x && p.y == p.y && p.z == p.z) {
                    sum_x += p.x;
                    sum_y += p.y;
                    sum_z += p.z;
                    N_pixel++;
                }
            }

        // from local coordinate
        x = sum_x / N_pixel;
        y = sum_y / N_pixel;
        z = sum_z / N_pixel;

        if (x == x && y == y && z == z) {
            // world 2 zed
            tf::StampedTransform w2c;
            listener.lookupTransform("map", camera_frame, ros::Time(0), w2c);
            tf::Vector3 target_pos = w2c * tf::Vector3(x, y, z);

            target_position.x=target_pos.x();
            target_position.y=target_pos.y();
            target_position.z=target_pos.z();

            // global coordinates
            printf("pos of the target : [%.4f, %.4f ,%.4f] [m]\n", target_pos.x(), target_pos.y(), target_pos.z());
        }
    }

    else
        ROS_WARN("pcl not recieved");
}


void TrackingManager::result_imshow() {
    Mat rgb_mat_rect = rgb_mat.clone();
    rectangle(rgb_mat_rect,point1, point2,Scalar(0, 0, 255), 2, 8);
    imshow(src_window, rgb_mat_rect);
    waitKey(3);
}

void TrackingManager::result_publish() {

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,z));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),camera_frame,"target_pos_estimation"));

    visualization_msgs::Marker BBMarker;

    // Bounded Box around target
    BBMarker.header.frame_id="map";
    BBMarker.header.stamp=ros::Time::now();
    BBMarker.ns="targetBB";
    BBMarker.action=visualization_msgs::Marker::ADD;
    BBMarker.id=0;
    BBMarker.type=visualization_msgs::Marker::CUBE;

    BBMarker.pose.orientation.x = 0.0;
    BBMarker.pose.orientation.y = 0.0;
    BBMarker.pose.orientation.z = 0.0;
    BBMarker.pose.orientation.w = 1.0;

    BBMarker.scale.x = 0.3;
    BBMarker.scale.y = BBMarker.scale.x;
    BBMarker.scale.z= 0.3;

    BBMarker.color.r=1.0;
    BBMarker.color.a=0.5;

    BBMarker.pose.position=target_position;
    marker_pub.publish(BBMarker);

    geometry_msgs::PointStamped pointStamped;
    pointStamped.header.stamp=ros::Time::now();
    pointStamped.header.frame_id="map";
    pointStamped.point=target_position;

    pos_pub.publish(pointStamped);

}
