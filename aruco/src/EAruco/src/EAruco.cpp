#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include <memory>
#include <iomanip>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <iostream>
#include <vector>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

geometry_msgs::PoseStamped Aruco_pose_realsense;
static double fx, fy, cx, cy; //focal length and principal point (for camera frame)
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
double CamParameters[4];

bool Aruco_init = false;
bool Aruco_large = true;    //placeholder as we will have two aruco marker later
bool Aruco_large_found = false;
//bool Aruco_small_found = false;

//large: 7x7_100_ID28; small 5x5_100_ID40
cv::Ptr<cv::aruco::Dictionary> dictionary_large = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
//cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;
    CamParameters[0] = fx;
    CamParameters[1] = fy;
    CamParameters[2] = cx;
    CamParameters[3] = cy;
}

void Aruco_PosePub(double Aruco_xyz[3]){
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.pose.position.x = Aruco_xyz[0];
    Aruco_pose_realsense.pose.position.y = Aruco_xyz[1];
    Aruco_pose_realsense.pose.position.z = Aruco_xyz[2];
}

void Aruco_process(Mat image_rgb){
    cv::Mat ArucoOutput = image_rgb.clone();
    std::vector<int> ids_large;
    //std::vector<int> ids_small;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    //cv::Vec3d rvec, tvec;
    rvecs.clear();tvecs.clear();
    
    cv::aruco::detectMarkers(image_rgb, dictionary_large, corners, ids_large);
    if(ids_large.size()>0){
        Aruco_init=true;
        Aruco_large_found=true;
        cv::aruco::drawDetectedMarkers(ArucoOutput, corners, ids_large);
        cv::aruco::estimatePoseSingleMarkers(corners, 17.55, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(unsigned int i=0; i<ids_large.size();i++){
             cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
    }else{Aruco_large_found=false;}
    double temp[3];
    for (int k = 0; k<3; k++){
        //temp[k] = tvecs[k];
        ;
    }
    Aruco_PosePub(temp);
    cv::imshow("Aruco", ArucoOutput);
    cv::waitKey(1);
}

void camera_rgb_cb(const sensor_msgs::CompressedImageConstPtr &rgb){
    /* Image initialize */
    cv::Mat image_rgb;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Aruco_process(image_rgb);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "EAruco");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucoPose",1);

    while(ros::ok()){
        ros::spinOnce();
        ArucoPose_pub.publish(Aruco_pose_realsense);
    }
    return 0;
}