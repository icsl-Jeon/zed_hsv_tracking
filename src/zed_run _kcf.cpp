
/*
 * This code is for testing kcf tracker and position estimation with zed camera
 * (1) ZED sends rgb and depth image in ros (just for checking verification)
 * (2) KCF tracker gives the bounding box
 * (3) we return the position of target and compare it with Vicon
 */

#include <opencv2/opencv.hpp>

#include "zed_node/kcftracker.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/core/core.hpp>

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <image_transport/image_transport.h>


using namespace cv;

// global variables


const char* src_window = "Select ROI";

void mouseHandler(int , int , int , int , void* );

class TrackingManager{
private:

    const char* camera_frame;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;


    //KCF tracker result
    Rect result;
    float x,y,z; //position of target


    // ROS
    ros::NodeHandle nh;

    ros::Subscriber sub_pcl2;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_rgb;
    image_transport::Subscriber sub_d;

public:

    bool getInit; // did we get initial bounding box ?
    // KCF tracker initialization
    int drag,select_flag;
    // frame object
    Mat rgb_mat; //RGBA
    Mat d_mat; //D
    bool callback;
    KCFTracker tracker;
    Point point1,point2; // upper left corner / lower right corner

    TrackingManager():it(nh) {
        // KCF tracker params
        bool HOG = false;
        bool FIXEDWINDOW = false;
        bool MULTISCALE = true;
        bool SILENT = true;
        bool LAB = false;
        tracker = KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
        cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        sub_rgb = it.subscribe("/zed/rgb/image_rect_color", 10,&TrackingManager::img_rgb_callback, this);
        sub_d = it.subscribe("zed/depth/depth_registered", 10, &TrackingManager::img_d_callback, this);
        sub_pcl2 = nh.subscribe("/zed/point_cloud/cloud_registered", 10,&TrackingManager::pcl_callback,this);

        namedWindow(src_window,CV_WINDOW_AUTOSIZE);

        drag = 0, select_flag = 0;
        // upper left corner / lower right corner
        camera_frame = "zed_left_camera";
        callback = false;
        getInit = false; // did we get initial bounding box ?


    }

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& );
    void img_rgb_callback(const sensor_msgs::ImageConstPtr& );
    void img_d_callback(const sensor_msgs::ImageConstPtr& );
    void get_target_pos();
    bool isReceived(){return rgb_mat.cols!=0;};
    void cur_imshow() {imshow(src_window, rgb_mat); waitKey(3);};
    void result_imshow();
    void gettingROI();

};


int main(int argc, char **argv) {

    //ros
    ros::init(argc,argv,"kcf_tracking_node");
    ros::Time::init();
    ros::MultiThreadedSpinner spinner(2);
    ros::Rate rate(30);
    TrackingManager tracking_manager;
    tracking_manager.gettingROI();
    setMouseCallback(src_window, mouseHandler, &tracking_manager);

    while(ros::ok()) {

        ROS_INFO_STREAM(tracking_manager.isReceived());

        if(tracking_manager.isReceived()) {


            if (!tracking_manager.getInit) {
                // still didn't get any initial regions
                // nothing to be done
                tracking_manager.cur_imshow();
            } else {

                ROS_INFO_STREAM("result show");

                tracking_manager.result_imshow();
                tracking_manager.get_target_pos();
            }
        }
        else
            ROS_INFO_STREAM("please subscribe to img topic");
//        ros::spin(); // important !


        ros::spinOnce(); // must use spin once
        rate.sleep();
    }
    return 0;
}





// callback function for getting pointcloud
void TrackingManager::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // Procedure:sensor_msgs->PCL2->PCL->extraction
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);
//    pcl::PointXYZ p1 = depth.at(x, y);
}



// retreive msg
void TrackingManager::img_rgb_callback(const sensor_msgs::ImageConstPtr& img_rgb_msg){

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
//        cv_ptr = cv_bridge::toCvCopy(img_rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(img_rgb_msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    rgb_mat=cv_ptr->image.clone();
//    std::cout<<"current size: "<<(rgb_mat.cols)<<std::endl;

}


void TrackingManager::img_d_callback(const sensor_msgs::ImageConstPtr& img_d_msg){

    // if openni mode = 1,2 -> then encoding TYPE_16UC1 / mode = 0 -> encoding TYPE_32FC1
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
//        cv_ptr = cv_bridge::toCvCopy(img_d_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr = cv_bridge::toCvCopy(img_d_msg);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    d_mat=(cv_ptr->image).clone();

}

// if we get any image after initialization, get the position of target
void TrackingManager::get_target_pos(){
    float sum_x=0,sum_y=0,sum_z=0;
    int N_pixel=0; // num of non-nan number

    for(int i=result.x; i< result.x+result.width;i++)
        for(int j=result.y;j<result.y+result.height;j++) {
            pcl::PointXYZRGB p = cloud_ptr->at(i, j);
            if (p.x == p.x && p.y == p.y && p.z == p.z) {
                sum_x += p.x;
                sum_y += p.y;
                sum_z += p.z;
                N_pixel++;
            }
        }
    x=sum_x/N_pixel; y=sum_y/N_pixel; z=sum_z/N_pixel;
    printf("pos of the target : [%.4f, %.4f ,%.4f] [m]\n",x,y,z);
}


void mouseHandler(int event, int xx, int yy, int flags, void* param)
{


    TrackingManager *trackingManagerPtr=(TrackingManager *) param;

    Mat rgb_mat = (trackingManagerPtr->rgb_mat).clone();
    if(trackingManagerPtr->isReceived()) {
        if (event == CV_EVENT_LBUTTONDOWN && !trackingManagerPtr->drag && !trackingManagerPtr->select_flag) {
            /* left button clicked. ROI selection begins */
            trackingManagerPtr->point1 = Point(xx, yy);
            trackingManagerPtr->drag = 1;
        }

        if (event == CV_EVENT_MOUSEMOVE && trackingManagerPtr->drag && !trackingManagerPtr->select_flag) {
            /* mouse dragged. ROI being selected */
            Mat img1 = rgb_mat.clone();
            trackingManagerPtr->point2 = Point(xx, yy);
            rectangle(img1, trackingManagerPtr->point1, trackingManagerPtr->point2, CV_RGB(255, 0, 0), 3, 8, 0);
            imshow(src_window, img1);
            waitKey(3);
        }

        if (event == CV_EVENT_LBUTTONUP && trackingManagerPtr->drag && !trackingManagerPtr->select_flag) {
            Mat img2 = rgb_mat.clone();
            trackingManagerPtr->point2 = Point(xx, yy);
            trackingManagerPtr->drag = 0;
            imshow(src_window, img2);
            waitKey(3);
            trackingManagerPtr->callback = true;
            trackingManagerPtr->getInit = true; // got initial bounding box

            ROS_INFO("ROI selected: pnt1: [%d, %d]/ pnt2: [%d, %d]\n", trackingManagerPtr->point1.x,
                   trackingManagerPtr->point1.y,trackingManagerPtr->point2.x,trackingManagerPtr->point2.y);

            int xMin = std::min(trackingManagerPtr->point1.x,trackingManagerPtr->point2.x);
            int xMax = std::max(trackingManagerPtr->point1.x,trackingManagerPtr->point2.x);
            int yMin = std::min(trackingManagerPtr->point1.y,trackingManagerPtr->point2.y);
            int yMax = std::max(trackingManagerPtr->point1.y,trackingManagerPtr->point2.y);
            int width = xMax - xMin;
            int height = yMax - yMin;

            // tracker initialization
            trackingManagerPtr->tracker.init(Rect(xMin, yMin, width, height), rgb_mat);
            ROS_INFO_STREAM("tracker init!\n");
        }
    }
}

void TrackingManager::result_imshow() {
    result = tracker.update(rgb_mat); //always pure
    Mat rgb_mat_rect = rgb_mat.clone();
    rectangle(rgb_mat_rect, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height),
              Scalar(255, 0, 0), 1, 8);
    imshow(src_window, rgb_mat_rect);
    waitKey(3);

}

void TrackingManager::gettingROI() {

}
