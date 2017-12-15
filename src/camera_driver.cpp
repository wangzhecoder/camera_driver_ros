//////////////////////////////////////////////
//* This is ros driver for camera(wifi and usb)
//* Author: ZheWang
//* Email:wangzhe936@outlook.com
//////////////////////////////////////////////


#include<ros/ros.h>
#include <ros/package.h>
#include<iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/opencv.hpp>

#include<std_msgs/Header.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>


using namespace std;

class CameraDriver{
private:
    bool imageSizeInited;
    cv::Size newSize;
    ros::NodeHandle h;
public:
    ros::Publisher imagePub_origin;
    ros::Publisher imagePub_gray;
    ros::Publisher imagePub_origin_undistored;
    ros::Publisher imagePub_gray_undistored;
    cv::VideoCapture vcap;
public:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Matx33d K2;
    cv::Mat new_intrinsic_mat;
    
public:
    CameraDriver();
    ~CameraDriver();
    bool LoadCamImage(cv::Mat & image);
    void imgToMsg(ros::Publisher pub, cv::Mat & image,int type);
    bool OpenCam(string videoSreamAddress);//use to open web camera
    bool OpenCam(int camID);//use to open usb camera
    void undistortCameraImage(cv::Mat & img);
    cv::Mat convert2Gray(cv::Mat image);
};

CameraDriver::CameraDriver()
    :imageSizeInited(false),
    cameraMatrix(cv::Mat::eye(3, 3, CV_64F)),
    distCoeffs(cv::Mat::zeros(4, 1, CV_64F))
{
    imagePub_origin = h.advertise<sensor_msgs::Image>("/camera/image_raw/origin",1);
    imagePub_gray = h.advertise<sensor_msgs::Image>("/camera/image_raw/gray",1);
    imagePub_origin_undistored = h.advertise<sensor_msgs::Image>("/camera/image_raw/origin_undistored",1);
    imagePub_gray_undistored = h.advertise<sensor_msgs::Image>("/camera/image_raw/gray_undistored",1);

    // Load Settings and Check
    string strSettingsFile = ros::package::getPath("camera_driver") +"/"+"config/config.yaml";
    
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to omni_slam package directory.");
        ros::shutdown();
    }
    // load parameters
    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];

    cameraMatrix.at<double>(0, 0) = fx;
    cameraMatrix.at<double>(0, 2) = cx;
    cameraMatrix.at<double>(1, 1) = fy;
    cameraMatrix.at<double>(1, 2) = cy;

    float k1 = fsSettings["Camera.k1"];
    float k2 = fsSettings["Camera.k2"];
    float p1 = fsSettings["Camera.p1"];
    float p2 = fsSettings["Camera.p2"];

    distCoeffs.at<double>(0, 0) = k1;
    distCoeffs.at<double>(1, 0) = k2;
    distCoeffs.at<double>(2, 0) = p1;
    distCoeffs.at<double>(3, 0) = p2;

    int width = fsSettings["Image.width"];
    int height = fsSettings["Image.height"];

    cameraMatrix.copyTo(new_intrinsic_mat);
    new_intrinsic_mat.at<double>(0, 0) *= 1;
    new_intrinsic_mat.at<double>(1, 1) *= 1;
    new_intrinsic_mat.at<double>(0, 2) = 0.5 * width;
    new_intrinsic_mat.at<double>(1, 2) = 0.5 * height;

}

CameraDriver::~CameraDriver()
{
    vcap.release();
}

bool CameraDriver::OpenCam(string videoSreamAddress)
{
    if(!vcap.open(videoSreamAddress))
    {
        cout << "Error opening video stream" << endl;
        return false;
    }
    return true;
}

bool CameraDriver::OpenCam(int camID)
{
    if(!vcap.open(camID))
    {
        cout << "Error opening video stream" << endl;
        return false;
    }
    return true;
}

bool CameraDriver::LoadCamImage(cv::Mat & outputImage)
{
    cv::Mat imageTemp;
    if(!vcap.read(imageTemp))
    {
        cout<<"No Frame"<<endl;
        return false;
    }
    if(!imageSizeInited)
    {
        newSize.width = 640;
        newSize.height = imageTemp.rows*640/imageTemp.cols;
        imageSizeInited = true;
    }
    // cv::resize(imageTemp,outputImage,newSize,0,0,cv::INTER_AREA);
    outputImage = imageTemp.clone();
    imageTemp.release();
    return true;
}

void CameraDriver::imgToMsg(ros::Publisher pub, cv::Mat & image,int type)
{
    sensor_msgs::ImagePtr imgMsg;
    switch(type)
    {
        case 1:
            imgMsg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,image).toImageMsg();
            break;
        case 2:
            imgMsg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::MONO8,image).toImageMsg();
            break;
    }
    imgMsg->header.stamp = ros::Time::now();
    imgMsg->header.frame_id = "world";
    pub.publish(imgMsg);
}

void CameraDriver::undistortCameraImage(cv::Mat & img)
{
    cv::fisheye::undistortImage(img, img, cameraMatrix, distCoeffs, new_intrinsic_mat);
}

cv::Mat CameraDriver::convert2Gray(cv::Mat image)
{
    cv::Mat img;
    if(image.channels()==3)
    {
        cv::cvtColor(image,img,CV_BGR2GRAY);
    }
    return img;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"camera_driver");
    ros::NodeHandle nh("~");
    CameraDriver camera_driver;
    std::string camera_address;
    if(!nh.getParam("camera_address",camera_address))
        camera_address = "/dev/video0";
    if(!camera_driver.OpenCam(camera_address))
    {
        ROS_ERROR("Can't open camera ! Program exit !");
        return 0;
    }

    else
        cout<<"Open camera successfull! Publishing camera image!"<<endl;
    ros::Rate loopRate(30);

    while(ros::ok())
    {
        cv::Mat image;
        if(camera_driver.LoadCamImage(image))
        {
            camera_driver.imgToMsg(camera_driver.imagePub_origin,image,1);
            cv::Mat gray = camera_driver.convert2Gray(image);
            camera_driver.imgToMsg(camera_driver.imagePub_gray,gray,2);
            camera_driver.undistortCameraImage(image);
            camera_driver.imgToMsg(camera_driver.imagePub_origin_undistored,image,1);
            camera_driver.undistortCameraImage(gray);
            camera_driver.imgToMsg(camera_driver.imagePub_gray_undistored,gray,2);
            gray.release();
            image.release();
        }
        else
            cout<<"Load camera image failed !!"<<endl;
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}



