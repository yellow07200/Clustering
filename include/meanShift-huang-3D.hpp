//#ifndef MEANSHIFT_MEANSHIFT_HPP
//#define MEANSHIFT_MEANSHIFT_HPP
#ifndef MEANSHIFT_H_
#define MEANSHIFT_H_

#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include "Point.hpp"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#define MAX_ITERATIONS 100


#define DVSW 240
#define DVSH 180


namespace dvs_test
{

class Meanshift {
public:
  Meanshift(ros::NodeHandle & nh);//, ros::NodeHandle nh_private);
  //virtual ~Meanshift();

private:
  ros::NodeHandle nh_;

  void eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg);
  void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info);
  void PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_info);
  void CvImageCallback(const sensor_msgs::Image::ConstPtr& depth);
  //void CvImageCallback(const cv_bridge::CvImage::ConstPtr& depth);

  ros::Subscriber event_sub_;
  ros::Subscriber cam_info_subs_;
  ros::Subscriber pcl_sub_;
  ros::Subscriber depth_pub_;
  ros::Publisher center_orien_pub;

	int height,width;
	double timestamp;
	int events_size;
	std::vector<double> point;
	std::vector<double> ts;
	std::vector<Point> points;
	std::vector<int> cluster_ID;
	std::vector<Point> cluster_center;	
	std::vector<cv::Point> points_vector;
	std::vector<cv::Point> points_ID0;
	std::vector<std::vector<cv::Point> > points_cluster;

	std::vector<Point > points3d;
	cv::Mat sparseMatrix;
	//std::vector<int> events_num;
	std::vector<bool> p;

  	int numRows, numCols, maxIterNum;
	float timeDivider;
	std::vector<Point> pointsp;

	double angle;
	double grasping_angle;

	//frame of times
	cv::Mat BGAFframe;
	
};

	//std::vector<Cluster> meanShift(const std::vector<Point> &points, float bandwidth);//, int events_num);



}//namespace


#endif // MEANSHIFT_H_

