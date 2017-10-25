/*
 * imageConverter.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Zhe Wang
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <vector>
#include <boost/thread.hpp>
#include <multi_robot_slam/Scenenode.h>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport depth_it_;
  image_transport::Subscriber depth_image_sub;
  ros::Subscriber mark_sub;
  ros::Publisher scene_node_pub;
  bool detSwitch;
  int sceneFrames;
  std::vector<cv::Mat> voc_src;
  cv::Mat vocab;
  cv::Mat mask;

  typedef struct
  {
	  double x,y;
	  std::vector<cv::KeyPoint> keyPoints;
  }sceneNode;
  sceneNode s_node;

public:
  ImageConverter();
  ~ImageConverter();
  void orbDetect(cv_bridge::CvImagePtr &cv_ptr,sceneNode node);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void markArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  void getdepthMask(const sensor_msgs::ImageConstPtr& msg);
};

ImageConverter::ImageConverter()
    : it_(nh_),
	  sceneFrames(0),
	  depth_it_(nh_)
{
    // Subscrive to input video feed and publish output video feed
	detSwitch = false;
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    mark_sub = nh_.subscribe("visualization_marker_array",1,&ImageConverter::markArrayCallback,this);
    scene_node_pub = nh_.advertise<multi_robot_slam::Scenenode>("/image_converter/scene_node",1);
    depth_image_sub = depth_it_.subscribe("/camera/depth_registered/image_raw",1,&ImageConverter::getdepthMask,this);
	//read vocabular from file
    cv::FileStorage fs;
	fs.open("/home/turtlebot/catkin_ws/src/multi_robot_slam/src/vocab.yml", cv::FileStorage::READ);
	fs["Vocabulary"] >> vocab;
	if (vocab.empty()) {
		ROS_INFO("vocab Invaild");
	}
	fs.release();
}

ImageConverter::~ImageConverter()
  {
  }

//*******************************************************************
//detect ORB features and descriptors and computer BOW data
//*******************************************************************
void ImageConverter::orbDetect(cv_bridge::CvImagePtr &cv_ptr,sceneNode node)//cv::Mat & img
{
	cv::Ptr<cv::FeatureDetector> orb_detector =
				cv::FeatureDetector::create("ORB");
	ROS_INFO("orb created");
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptors;
	multi_robot_slam::Scenenode msg;
	sensor_msgs::Image img;
	cv_bridge::CvImage des_ptr;//(cv_ptr->header,sensor_msgs::image_encodings::MONO8,descriptors);
	cv_bridge::CvImagePtr des_ptrr;
	des_ptr.header=cv_ptr->header;
	des_ptr.encoding=sensor_msgs::image_encodings::TYPE_32FC1;
	ROS_INFO("before orb created");
	orb_detector->detect(cv_ptr->image, keyPoints,mask);
	ROS_INFO("orb_detector detected");
	if(!keyPoints.empty())
	{
		//extract the descriptors
		cv::Ptr<cv::DescriptorExtractor> descriptor_extractor =
					cv::DescriptorExtractor::create("ORB");
		descriptor_extractor->compute(cv_ptr->image,keyPoints,descriptors);

		cv::Mat data;
		descriptors.convertTo(data, CV_32F);
		voc_src.push_back(data);
		if(sceneFrames%1==0&&sceneFrames!=0)
		{
//			cv::BOWKMeansTrainer voc_trainer(100);//100
//			std::vector<cv::Mat>::iterator v;
//			for(v = voc_src.begin();v!=voc_src.end();v++)
//				voc_trainer.add(*v);
//			ROS_INFO("voc added");
//			cv::Mat vocab = voc_trainer.cluster();

			//change vocab format from CV_32F to CV_8U
			cv::Mat vocab_uchar;
			float min=1.0,max=1.0;
			for(int m = 0; m < vocab.rows; m++)
			{
				for(int l = 0; l < vocab.cols; l++)
				{
					if(vocab.at<float>(m,l)<min)
						min=vocab.at<float>(m,l);
					if(vocab.at<float>(m,l)>max)
						max=vocab.at<float>(m,l);
				}
			}
			if(min!=max)
				vocab.convertTo(vocab_uchar, CV_8U,255.0/(max-min),-255.0*min/(max-min));

			ROS_INFO("voc clustered");
		
			//compute training data which needed by FabMap
			cv::Ptr<cv::DescriptorExtractor> extractor =
					cv::DescriptorExtractor::create("ORB");
			ROS_INFO("ORB created");
			cv::Ptr<cv::DescriptorMatcher> matcher =
				cv::DescriptorMatcher::create("BruteForce-Hamming");
			ROS_INFO("matcher created");
			cv::BOWImgDescriptorExtractor bide(extractor, matcher);
			bide.setVocabulary(vocab_uchar);
			ROS_INFO("bide vocab setted");
			bide.compute(cv_ptr->image, keyPoints, des_ptr.image);

			//drawKeypoints
			cv::Mat kepointImg;
			cv::drawKeypoints(cv_ptr->image,keyPoints,kepointImg,cv::Scalar(0,255,255),4);
			cv::imshow("keypoints", kepointImg);

			//transform 
			des_ptr.toImageMsg(img);
			ROS_INFO("des_ptr->image SIZE:[%d,%d]",des_ptr.image.rows,des_ptr.image.cols);
			msg.image=img;
			msg.px=node.x;
			msg.py=node.y;
			scene_node_pub.publish(msg);
			ROS_INFO("published");

			voc_src.clear();
			cv::FileStorage fs("vocabular.yml",cv::FileStorage::WRITE);
			fs<<"vocab"<<vocab;
			fs.release();

		}
		sceneFrames++;
	}
}

//*******************************************************************
//get image from ROS msg and change to OpenCV image format  
//*******************************************************************
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	if(detSwitch)
	{
		orbDetect(cv_ptr,s_node);
		ROS_INFO("Add SceneNode!Position[%lf,%lf]",s_node.x,s_node.y);
		detSwitch= false;
	}
	// Update GUI Window
	cv::waitKey(33);

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}

//*******************************************************************
//give pose value to msg
//*******************************************************************
void ImageConverter::markArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	ROS_INFO("Marker:[%lf,%lf]",msg->markers[msg->markers.size()-2].pose.position.x,msg->markers[msg->markers.size()-2].pose.position.y);

	if(msg->markers.size()>=2)
	{
		s_node.x=msg->markers[msg->markers.size()-2].pose.position.x;
		s_node.y=msg->markers[msg->markers.size()-2].pose.position.y;
	}
	else
	{
		s_node.x=msg->markers.back().pose.position.x;
		s_node.y=msg->markers.back().pose.position.y;
	}
	detSwitch = true;

}

//*******************************************************************
//Set distance depth image to mask
//*******************************************************************
void ImageConverter::getdepthMask(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	cv::Mat msk,msk1,msk2;
	int min=1,max=1;
	for(int m = 0; m < cv_ptr->image.rows; m++)
	{
		for(int l = 0; l < cv_ptr->image.cols; l++)
		{
			if(cv_ptr->image.at<ushort>(m,l)<min)
				min=cv_ptr->image.at<ushort>(m,l);
			if(cv_ptr->image.at<ushort>(m,l)>max)
				max=cv_ptr->image.at<ushort>(m,l);
		}
	}
	if(min!=max)
		cv_ptr->image.convertTo(msk, CV_8U,255.0/(max-min),-255.0*min/(max-min));

	cv::threshold(msk, msk1, 2.0, 255, cv::THRESH_BINARY);
	cv::threshold(msk, msk2, 150, 255, cv::THRESH_BINARY_INV);
	cv::bitwise_and(msk1, msk2, mask, cv::Mat());
	// cv::imshow("mask", mask);
	// cv::waitKey(33);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}



