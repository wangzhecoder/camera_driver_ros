/*
 * multiRobWorkstation.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Zhe Wang
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "multi_robot_slam/Scenenode.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class multiRobWorkstation
{
    private:
	ros::NodeHandle n;
	ros::Subscriber sceneNode_sub1;
	ros::Subscriber sceneNode_sub2;
	ros::Subscriber sceneNode_sub3;
	bool emptyTree;
	int scene_frames1,scene_frames2,scene_frames3;
	std::vector<std::vector<cv::Mat> > traindatas;
	std::vector<cv::Mat> traindatas1;
	std::vector<cv::Mat> traindatas2;
	std::vector<cv::Mat> traindatas3;
	ros::Publisher vis_pub1;
	ros::Publisher vis_pub2;
	ros::Publisher vis_pub3;

	typedef struct sceneNode{
		float x,y;
	};

	std::vector<sceneNode> sceneNodes;

    public:
	multiRobWorkstation();
	~multiRobWorkstation();
    void sceneNodeCallback1(const multi_robot_slam::Scenenode::ConstPtr& msg);
	void sceneNodeCallback2(const multi_robot_slam::Scenenode::ConstPtr& msg);
	void sceneNodeCallback3(const multi_robot_slam::Scenenode::ConstPtr& msg);
	void drawBOWHistgram(const cv::Mat bow);
	void publishSceneMatchMarker(cv::of2::IMatch match,std::vector<sceneNode> sceneNodes,float r,float g,float b,int matchSceneNum);
	void sceneMatch(std::vector<std::vector<cv::Mat> > traindatas_,cv::Mat data,int sceneNum);
	cv::Ptr<cv::of2::FabMap> createFabmapObject(std::vector<cv::Mat> traindatas);
	void fabmapCompare(cv::Ptr<cv::of2::FabMap> fabmap[],cv::Mat data,int sceneNum);
};

multiRobWorkstation::multiRobWorkstation()
	:emptyTree(true),
	 scene_frames1(0),
	 scene_frames2(0),
	 scene_frames3(0)
{
	sceneNode_sub1 = n.subscribe("/image_converter/Robot1_scene", 1, &multiRobWorkstation::sceneNodeCallback1,this);
	sceneNode_sub2 = n.subscribe("/image_converter/Robot2_scene", 1, &multiRobWorkstation::sceneNodeCallback2,this);
	sceneNode_sub3 = n.subscribe("/image_converter/Robot3_scene", 1, &multiRobWorkstation::sceneNodeCallback3,this);
	vis_pub1 = n.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 );
	vis_pub2 = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );
	vis_pub3 = n.advertise<visualization_msgs::Marker>( "visualization_marker3", 0 );
}

multiRobWorkstation::~multiRobWorkstation()
{

}

//*******************************************************************
//draw histgram according to bow data 
//*******************************************************************
void multiRobWorkstation::drawBOWHistgram(const cv::Mat bow)
{
	cv::Mat data = bow;
	cv::Mat result_display = cv::Mat::zeros(500, 500,CV_8UC3);
	for(int l = 0; l < data.cols; l++)
	{
		cv::line(result_display, cv::Point(0,250), cv::Point(500,250),cv::Scalar(0,255,255), 1, CV_AA, 0);
		cv::line(result_display, cv::Point(l,500-data.at<float>(l)*500), cv::Point(l,500.0),cv::Scalar(255,0,0), 1, CV_AA, 0);
	}
	cv::imshow("Histgram", result_display);
	data.release();
}

//*******************************************************************
//publish matched scene Marker 
//*******************************************************************
void multiRobWorkstation::publishSceneMatchMarker(cv::of2::IMatch match,std::vector<sceneNode> sceneNodes,float r,float g,float b,int matchSceneNum)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	char str[20];
	std::sprintf(str, "%.3lf",match.match);
	marker.text=str;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = sceneNodes[match.imgIdx].x;
	marker.pose.position.y = sceneNodes[match.imgIdx].y;
	marker.pose.position.z = 0.5;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	switch(matchSceneNum)
	{
		case 1:
			vis_pub1.publish(marker);
			break;
		case 2:
			vis_pub2.publish(marker);
			break;
		case 3:
			vis_pub3.publish(marker);
			break;
	}
}

cv::Ptr<cv::of2::FabMap> multiRobWorkstation::createFabmapObject(std::vector<cv::Mat> traindatas)
{
	cv::of2::ChowLiuTree treeBuilder;
	cv::Mat tree;
	cv::Ptr<cv::of2::FabMap> fabmap;
	cv::of2::IMatch maxMatch;
	treeBuilder.add(traindatas);
	tree = treeBuilder.make();
	fabmap = new cv::of2::FabMap2(tree,0.39,0,cv::of2::FabMap::SAMPLED | cv::of2::FabMap::CHOW_LIU);
	fabmap->addTraining(traindatas);
	fabmap->add(traindatas);
	return fabmap;
}

//***********************************************************************
//compute rescent scene match probility and match to whitch robot's scene
//***********************************************************************
void multiRobWorkstation::fabmapCompare(cv::Ptr<cv::of2::FabMap> fabmap[],cv::Mat data,int sceneNum)
{
	cv::of2::IMatch maxMatch;
	std::vector<cv::of2::IMatch> matches_;
	std::vector<std::vector<cv::of2::IMatch> > matches;
	for(int i=0;i<3;i++)
	{
		if(i==sceneNum)
			continue;
		fabmap[i]->compare(data,matches_,false);
		matches.push_back(matches_);
	}
	std::vector<std::vector<cv::of2::IMatch> >::iterator n;
	n=matches.begin();
	std::vector<cv::of2::IMatch>::iterator m;
	// cv::of2::IMatch maxMatch;
	int matchNum,i=0;
	maxMatch.match=0;
	for(n=matches.begin();n!=matches.end();n++)
	{
		for(m=(*n).begin();m!=(*n).end();m++)
		{
			if(m->match>maxMatch.match)
			{
				maxMatch=*m;
				matchNum = i;
			}
		}
		i++;
	}
	if(matchNum==1)
	{
		switch(sceneNum)
		{
			case 1:
				matchNum=2;
				break;
			default:
				matchNum=1;
		}
	}
	if(matchNum==2)
	{
		switch(sceneNum)
		{
			case 3:
				matchNum=2;
				break;
			default:
				matchNum=3;
		}
	}
	if(maxMatch.match>0.7 &&maxMatch.imgIdx!=-1)
		publishSceneMatchMarker(maxMatch,sceneNodes,0.0,1.0,0.0,matchNum);
	else
		publishSceneMatchMarker(maxMatch,sceneNodes,1.0,1.0,0.0,matchNum);
}

//*******************************************************************
//match scene which hava been visited 
//*******************************************************************
void multiRobWorkstation::sceneMatch(std::vector<std::vector<cv::Mat> > traindatas_,cv::Mat data,int sceneNum)
{
	cv::Ptr<cv::of2::FabMap> fabmap[3];
	std::vector<std::vector<cv::Mat> >::iterator t;
	int tn=0;
	//define three switches to control whether it is ready to run compare
	bool sw[3]={false,false,false};
	for(t=traindatas_.begin();t!=traindatas_.end();t++)
	{
		if(!(*t).empty())
		{
			ROS_INFO("Scene[%d] is not empty!",tn);
			fabmap[tn] = createFabmapObject(*t);
			sw[tn]=true;
			tn++;
		}
	}
	//Only three robots are all ready,compare can work
	//Here hasn't be completed !!!
	if(sw[0]&&sw[1]&&sw[2])
	{
		switch(sceneNum)
		{
			case 1:
				fabmapCompare(fabmap,data,1);
				break;
			case 2:
				fabmapCompare(fabmap,data,2);
				break;
			case 3:
				fabmapCompare(fabmap,data,3);
				break;
		}
	}
	else
		ROS_INFO("Error!Compare denied! Make sure there are three robot and all have been already!");
}

//*******************************************************************
//scene recognize using FabMap
//*******************************************************************
void multiRobWorkstation::sceneNodeCallback1(const multi_robot_slam::Scenenode::ConstPtr& msg)
{
  	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg->image,sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	drawBOWHistgram(cv_ptr->image);

	if((scene_frames1%2)==0&&scene_frames1!=0)
	{
		traindatas.push_back(traindatas1);
		traindatas.push_back(traindatas2);
		traindatas.push_back(traindatas3);
		sceneMatch(traindatas,cv_ptr->image,1);
		traindatas.clear();
	}
	if(scene_frames1%1==0)
	{
		traindatas1.push_back(cv_ptr->image);

		sceneNode sn;
		sn.x=msg->px;
		sn.y=msg->py;
		sceneNodes.push_back(sn);
	}
	scene_frames1++;
	// cv::waitKey(33);
}

void multiRobWorkstation::sceneNodeCallback2(const multi_robot_slam::Scenenode::ConstPtr& msg)
{
  	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg->image,sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	drawBOWHistgram(cv_ptr->image);

	if((scene_frames2%2)==0&&scene_frames2!=0)
	{
		traindatas.push_back(traindatas1);
		traindatas.push_back(traindatas2);
		traindatas.push_back(traindatas3);
		sceneMatch(traindatas,cv_ptr->image,2);
		traindatas.clear();
	}
	if(scene_frames2%1==0)
	{
		traindatas2.push_back(cv_ptr->image);

		sceneNode sn;
		sn.x=msg->px;
		sn.y=msg->py;
		sceneNodes.push_back(sn);
	}
	scene_frames2++;
	// cv::waitKey(33);
}

void multiRobWorkstation::sceneNodeCallback3(const multi_robot_slam::Scenenode::ConstPtr& msg)
{
  	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg->image,sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	drawBOWHistgram(cv_ptr->image);

	if((scene_frames3%2)==0&&scene_frames3!=0)
	{
		traindatas.push_back(traindatas1);
		traindatas.push_back(traindatas2);
		traindatas.push_back(traindatas3);
		sceneMatch(traindatas,cv_ptr->image,3);
		traindatas.clear();
	}
	if(scene_frames3%1==0)
	{
		traindatas3.push_back(cv_ptr->image);

		sceneNode sn;
		sn.x=msg->px;
		sn.y=msg->py;
		sceneNodes.push_back(sn);
	}
	scene_frames3++;
	// cv::waitKey(33);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worstation");
  multiRobWorkstation mk;
  ros::spin();
  return 0;
}