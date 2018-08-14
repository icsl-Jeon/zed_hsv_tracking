//
// Created by jbs on 18. 8. 10.
//

/*
 * ZED MANAGER
 *
 * ROS COMM
 *
 * (1) publish point cloud information to Octomap server and bounding boxed image to GCS
 * (2) subscribe tf information from GCS
 *
 *
 * FUNCTIONS
 *
 * (1) retrieve pcl, RGB image from ZED camera
 * (2) HSV tracking
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
#include <sl/Camera.hpp>
#include <string>

using namespace cv;

// global variables


struct ThresHSV{

    int iLowH; //= 0;
    int iHighH; // = 179;

    int iLowS; //= 0;
    int iHighS; //= 255;

    int iLowV; //= 0;
    int iHighV; //= 255;
};


/** DALOKA
H: 31-76

S: 116-255

V: 0-255
**/


/**
* Conversion function between sl::Mat and  Mat
**/
Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since  Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    //  Mat and sl::Mat will share a single memory structure
    return  Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

/**
* Conversion function between cv::Mat to ROSmsg
**/

sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}



/**
 * ZED camera manager
 */

class ZEDManager{
private:


    // frame
    std::string camera_frame_id;
    std::string world_frame_id;
    std::string target_frame_id;


    // DATA
    sl::Camera ZED;
    int width;
    int height;

    sensor_msgs::PointCloud2 pc2;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    // ZED type
    sl::InitParameters init_params;
    sl::RuntimeParameters run_params;
    sl::Mat image;
    sl::Mat cloud;

    float min_depth;
    float max_depth;
    
    // opencv type
    Mat rgb_raw; //RGBA
    Mat HSV;
    Mat rgb_detected; // detected region
    Mat nonZeroCoordinates; // detected region (index)
    bool callback;
    bool view_thres;

    // Target
    float x,y,z; //position of target w.r.t local frame of camera
    geometry_msgs::Point target_position; // target position in world frame
    Point point1,point2; // lower left corner / upper right corner of detected region
    visualization_msgs::Marker BBMarker;


    // ROS
    ros::NodeHandle nh;
    ros::Publisher target_pos_pub; // publish position (x,y,z) of target
    ros::Publisher marker_pub; // bounding box for visualization


    image_transport::ImageTransport it;
    image_transport::Publisher pub_raw_rgb;
    image_transport::Publisher pub_detected_rgb;
    ros::Publisher pub_cloud;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;




public:

    // parameters
    double rate;
    ThresHSV thres_daloka;
    bool HSV_tuning;

    ZEDManager() ;
    void retrieve();
    void publishPointCloud();
    void publishRGB();
    void get_target_pos();
    void tracking_update(); // update rectangle by thresholding HSV (Bounding Box)
    void publishMarker(); // detected target position publish

};


const char * window_name="HSV tracker";

int main(int argc, char ** argv){

    ros::init(argc,argv,"ZED_manager");
    ZEDManager ZED_manager;
    ros::Rate rate(ZED_manager.rate);

    if(ZED_manager.HSV_tuning){
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);


    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", window_name, &(ZED_manager.thres_daloka.iLowH), 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", window_name, &(ZED_manager.thres_daloka.iHighH), 179);

    cvCreateTrackbar("LowS", window_name, &(ZED_manager.thres_daloka.iLowS), 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", window_name, &(ZED_manager.thres_daloka.iHighS), 255);

    cvCreateTrackbar("LowV", window_name, &(ZED_manager.thres_daloka.iLowV), 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", window_name,&(ZED_manager.thres_daloka.iHighV), 255);
    }



    while(ros::ok()){
   
	ros::Time t0,t1;
 
	t0=ros::Time::now();
        ZED_manager.retrieve(); 
	t1=ros::Time::now();

	t0=ros::Time::now();
	ZED_manager.tracking_update(); 
	t1=ros::Time::now();

	t0=ros::Time::now();
	ZED_manager.get_target_pos(); 
	t1=ros::Time::now();
	ZED_manager.publishRGB();
        ZED_manager.publishPointCloud();
        ZED_manager.publishMarker();

//        ROS_INFO("PCL published");
    	rate.sleep();

    }

   }

/**
* Function definition
**/



ZEDManager::ZEDManager():nh("~"),it(nh) {

    // parameter parsing
    nh.getParam("world_frame_id",world_frame_id);
    nh.getParam("camera_frame_id",camera_frame_id);
    nh.getParam("target_frame_id",target_frame_id);

    nh.getParam("loop_rate",rate);
    nh.getParam("max_sensing_depth",max_depth);
    nh.getParam("min_sensing_depth",min_depth);

    nh.getParam("HSV_tuning",HSV_tuning);

    nh.getParam("H_max",thres_daloka.iHighH);
    nh.getParam("H_min",thres_daloka.iLowH);
    nh.getParam("S_max",thres_daloka.iHighS);
    nh.getParam("S_min",thres_daloka.iLowS);
    nh.getParam("V_max",thres_daloka.iHighV);
    nh.getParam("V_min",thres_daloka.iLowV);

    nh.getParam("view_threshold_img",view_thres);
    // advertise  register
    target_pos_pub = nh.advertise<geometry_msgs::PointStamped>("target_position",10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("target_BB",10);

    pub_cloud=nh.advertise<sensor_msgs::PointCloud2>("point_cloud",1);
    pub_detected_rgb=it.advertise("detected_rgb",1);
    pub_raw_rgb=it.advertise("raw_rgb",1);


    // ZED_params
    init_params.sdk_verbose=true;
    init_params.camera_resolution=sl::RESOLUTION_VGA;
    init_params.camera_fps=rate;
    init_params.coordinate_units=sl::UNIT_METER;
    //init_params.depth_minimum_distance=min_depth; // if we lower this limit, computation increases
    init_params.depth_mode=sl::DEPTH_MODE_PERFORMANCE; // ULTRA >> QUALITY >> MEDIUM >> PERFROMANCE 
    run_params.sensing_mode=sl::SENSING_MODE_STANDARD;

    ZED.setDepthMaxRangeValue(max_depth);
    ZED.disableSpatialMapping();
    ZED.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, -1, true);


    // bounding box marker


    // Bounded Box around target
    BBMarker.header.frame_id=world_frame_id;
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


    sl::ERROR_CODE err = ZED.open(init_params);

    if (err != sl::SUCCESS)
    {
        printf("zed camera opening error! please run Diagnotics");
        exit(-1);
    }


    width=ZED.getResolution().width;
    height=ZED.getResolution().height;

    cv::Size cvSize(width, height);
    rgb_raw=cv::Mat(cvSize,CV_8UC3);


}

void ZEDManager::retrieve() {

    // RGB img retrieve
    if (ZED.grab()==sl::SUCCESS){
        ZED.retrieveImage(image, sl::VIEW_LEFT); // get the left image (rgba)
        rgb_raw=slMat2cvMat(image);
        cv::cvtColor(rgb_raw, rgb_raw, CV_RGBA2RGB);
        // PCL
        ZED.retrieveMeasure(cloud,sl::MEASURE_XYZ);
    }
    else
        ROS_WARN("zed grab error");
};

void ZEDManager::publishPointCloud() {

    point_cloud.width = width;
    point_cloud.height = height;
    int size = width * height;
    point_cloud.points.resize(size);
//    ROS_INFO_STREAM("size of pcl: "<<point_cloud.points.size());

    sl::Vector4<float> *cpu_cloud = cloud.getPtr<sl::float4>();
    for (int i = 0; i < size; i++) {
        point_cloud.points[i].x = cpu_cloud[i][2];
        point_cloud.points[i].y = -cpu_cloud[i][0];
        point_cloud.points[i].z = -cpu_cloud[i][1];
    }

    pcl::toROSMsg(point_cloud, pc2); // Convert the point cloud to a ROS message
    pc2.header.frame_id = camera_frame_id; // Set the header values of the ROS message
    pc2.header.stamp = ros::Time::now();
    pc2.height = height;
    pc2.width = width;
    pc2.is_bigendian = false;
    pc2.is_dense = false;
    pub_cloud.publish(pc2);
}


void ZEDManager::publishRGB() {
    pub_raw_rgb.publish(imageToROSmsg(rgb_raw, sensor_msgs::image_encodings::BGR8, camera_frame_id, ros::Time::now()));
    pub_detected_rgb.publish(imageToROSmsg(rgb_detected, sensor_msgs::image_encodings::BGR8, camera_frame_id, ros::Time::now()));
}


void ZEDManager::tracking_update(){

    // update rectangle by thresholding HSV (Bounding Box)

    ThresHSV thresHSV=this->thres_daloka;

    cvtColor(rgb_raw, HSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    Mat imgThresholded;

    inRange(HSV, Scalar(thresHSV.iLowH, thresHSV.iLowS, thresHSV.iLowV),
            Scalar(thresHSV.iHighH, thresHSV.iHighS, thresHSV.iHighV),
            imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
 /** 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
**/
/**
    //morphological closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
**/
    if(HSV_tuning){
    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    waitKey(1);
   	}

    // bounding box

    Mat nonZeroCoordinates;

    findNonZero(imgThresholded, nonZeroCoordinates);
    int min_x = rgb_raw.cols+1, min_y = rgb_raw.rows+1, max_x = -1, max_y = -1;

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


    // margin
    point1.x = min_x+10;
    point1.y = min_y+10;
    point2.x = max_x-10;
    point2.y = max_y-10;


    // applying rectangle
    rgb_detected=rgb_raw.clone();
    rectangle(rgb_detected,point1,point2,Scalar(0, 0, 255), 2, 8);
};


// if we get any image after initialization, get the position of target
void ZEDManager::get_target_pos(){

    if(point_cloud.height==rgb_raw.rows) {

        float sum_x = 0, sum_y = 0, sum_z = 0;
        int N_pixel = 0; // num of non-nan number

        for (int j = point1.y; j < point2.y; j++)
            for (int i = point1.x; i < point2.x; i++) {
                pcl::PointXYZ p = point_cloud.at(i, j);
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

            listener.lookupTransform(world_frame_id, camera_frame_id, ros::Time(0), w2c);
            tf::Vector3 target_pos = w2c * tf::Vector3(x, y, z);

            target_position.x=target_pos.x();
            target_position.y=target_pos.y();
            target_position.z=target_pos.z();


            tf::Transform transform;
            transform.setOrigin( tf::Vector3(target_pos.x(), target_pos.y(), target_pos.z()));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),world_frame_id, target_frame_id ));
            // global coordinates
        }
    }
    else
        ROS_WARN("pcl not recieved");

}

void ZEDManager::publishMarker() {
    BBMarker.pose.position=target_position;
    marker_pub.publish(BBMarker);

}





