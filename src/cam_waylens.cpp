/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<ros/ros.h>
#include<iostream>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 

#include<std_msgs/Header.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>


using namespace std;

class WaylensCam{
private:
    bool imageSizeInited;
    cv::Size newSize;
    ros::Publisher imagePub;
    ros::NodeHandle h;
    cv::VideoCapture vcap;
public:
    WaylensCam();
    ~WaylensCam();
    bool LoadCamImage(cv::Mat & image);
    void imgToMsg(cv::Mat & image);
    bool OpenCam(string videoSreamAddress);
};

WaylensCam::WaylensCam()
    :imageSizeInited(false)
{
    imagePub = h.advertise<sensor_msgs::Image>("/cam0/image_raw",100);
}
WaylensCam::~WaylensCam()
{
    vcap.release();
}

bool WaylensCam::OpenCam(string videoSreamAddress)
{
    if(!vcap.open(videoSreamAddress))
    {
        cout << "Error opening video stream" << endl;
        return false;
    }
    return true;
}

bool WaylensCam::LoadCamImage(cv::Mat & outputImage)
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
    cv::resize(imageTemp,outputImage,newSize,0,0,cv::INTER_AREA);
    imageTemp.release();
    return true;
}

void WaylensCam::imgToMsg(cv::Mat & image)
{
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8,image).toImageMsg();
    imgMsg->header.stamp = ros::Time::now();
    imgMsg->header.frame_id = "map";
    imagePub.publish(imgMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"cam_waylens");
    WaylensCam waylenscam;
    if(!waylenscam.OpenCam("http://192.168.110.1:8081/cgi/mjpg/mjpg.cgi?.mjpg"))
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
        if(waylenscam.LoadCamImage(image))
        {
            waylenscam.imgToMsg(image);
            image.release();
        }
        else
            cout<<"Load camera image failed !!"<<endl;
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}

