#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>

#include "meanShift-huang-3D.hpp"
#include "meanShift-Gaussian-3D.hpp"
#include "Point.hpp"
#include "pca.hpp"
#include "FindFarestPoints.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include "opencv2/imgproc/imgproc_c.h"
#include <opencv/highgui.h>
//#include "opencv2/imgproc/imgproc.hpp"

#include <complex>
#include <omp.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include<std_msgs/Float32MultiArray.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>

#include "pcl_ros/point_cloud.h"



#define min(x,y) ((x)<(y)?(x):(y))

#define Num_Cluster 7

#define bandwidth 0.05//0.25

int iter=0;
uint64_t first_timestamp;
double final_timestamp;

bool firstevent = true;
double firsttimestamp;

std::vector<Point> centroid;
std::vector<Point> centroid_filter;
std::vector<Point> centroid_filter_prev;

std::vector<cv::Point> cluster_points;
std::vector<std::vector<cv::Point> > clusters_test;

std::vector<Point> centroid_final;
std::vector<Point> centroid_prev;
std::vector<std::vector<int> > centroid_traj_x;
std::vector<std::vector<int> > centroid_traj_y;
std::vector<std::vector<Point> > centroid_traj;
std::vector<Point> original_centroid;

std::vector<int> events_num;

std::vector<int> a;

std::vector<Point> cluster_center_final;
std::vector<int> cluster_center_final_x;
std::vector<int> cluster_center_final_y;
std::vector<cv::Point> cv_cluster_center_final;

std::vector<Point> pc_points;

int last_size;

float elapsedTime;
int t_count;	

	int LBI;
	int TP;
	int GT;

	int pcl_sign=0;
	int pcl_sign_prev=0;

int depth_sign=0;

cv::Mat depth_image;

cv::Mat data=cv::Mat(4, 186, CV_64F, cv::Scalar::all(0.));

namespace dvs_test {

Meanshift::Meanshift(ros::NodeHandle & nh) : nh_(nh) //, ros::NodeHandle nh_private//
{
	numRows=DVSH; numCols=DVSW;
  	//cam_info_subs_ = nh_.subscribe("/dvs/camera_info", 1, &Meanshift::CameraInfoCallback, this);
	pcl_sub_=nh_.subscribe("/point_cloud_Clustering", 1, &Meanshift::PointCloud2Callback, this);
	event_sub_=nh_.subscribe("/dvs/events",1,&Meanshift::eventsCallback_simple,this);///dvs/events ///feature_events
	depth_pub_=nh_.subscribe("/depth_map",1,&Meanshift::CvImageCallback,this);

	center_orien_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_info", 1);
}
void Meanshift::CvImageCallback(const sensor_msgs::Image::ConstPtr& depth)
{
	ROS_INFO("subscribed depth_image");
	depth_image = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::RGB8)->image;//_<uint8_t>
	int height=depth_image.rows; //180. y
	int width=depth_image.cols; //240 x
	ROS_INFO("height=%d,width=%d",height,width);

	for (int i=0; i<height;i++)
	{
		for (int j=0; j<width;j++)
		{
			int depth_val=depth_image.at<int>(i, j);
		}
	}

	int mid_p=(int)depth_image.at<uchar>(0,0);
	ROS_INFO("x_middle=%d",mid_p);//depth_image.ptr<int>(0,0));	


	cv::namedWindow("image_yz", CV_WINDOW_AUTOSIZE);
	cv::imshow("image_yz", depth_image);
	cv::waitKey(0);

	depth_sign=1;
}
void Meanshift::PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_info)
{
	pcl_sign++;
	int pc_size=pcl_info->data.size();
	ROS_INFO("PCl Info Size=%d",pc_size);

	//cv_bridge::CvImage cv_image_pc;

	/* cv::Mat pc_clusters=cv::Mat( 3, pc_size/3, CV_64F, cv::Scalar::all(0));

	//cv::Mat contourImageRange(src.size(), CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat cv_image_pc=cv::Mat( 255, 255, CV_8UC3, cv::Scalar::all(0));
	cv::Mat cv_image_pc1=cv::Mat( 255, 255, CV_8UC3, cv::Scalar::all(0));
	cv::Mat cv_image_pc2=cv::Mat( 255, 255, CV_8UC3, cv::Scalar::all(0));
	cv::Mat cv_image_pc3=cv::Mat( 255, 255, CV_8UC3, cv::Scalar::all(0));
	
	for (int i = 0; i < pc_size/3; ++i)
	{
        int pc_x = pcl_info->data[i*3];
        int pc_y = pcl_info->data[i*3+1];
		int pc_z = pcl_info->data[i*3+2];

		cv_image_pc.at<double>(cv::Point(pc_x, pc_y))= (255,255,255);
		cv_image_pc1.at<double>(cv::Point(pc_y, pc_x))= (255,255,255);
		cv_image_pc2.at<double>(cv::Point(pc_z, pc_x))= (255,255,255);
		cv_image_pc3.at<double>(cv::Point(pc_z, pc_y))= (255,255,255);

	
		pc_clusters.at<int>(cv::Point(i, 0))= pc_x;
		pc_clusters.at<int>(cv::Point(i, 1))= pc_y;
		pc_clusters.at<int>(cv::Point(i, 2))= pc_z;

	}    

	std::cout<<"finish scanning"<<std::endl;
	//namedWindow("image_xyz", cv::WINDOW_NORMAL);
	// cv::namedWindow("image_xy", CV_WINDOW_AUTOSIZE);
	// cv::imshow("image_xy", cv_image_pc);
	// cv::waitKey(0);

	// cv::namedWindow("image_yx", CV_WINDOW_AUTOSIZE);
	// cv::imshow("image_yx", cv_image_pc1);
	// cv::waitKey(0);

	// cv::namedWindow("image_xz", CV_WINDOW_AUTOSIZE);
	// cv::imshow("image_xz", cv_image_pc2);
	// cv::waitKey(0);
 
	// cv::namedWindow("image_yz", CV_WINDOW_AUTOSIZE);
	// cv::imshow("image_yz", cv_image_pc3);
	// cv::waitKey(0);

	// read from pcd file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yellow/3DClustering/Voxel_holes_inertial_.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}
	// std::cout << "Loaded "
	// 			<< cloud->width * cloud->height
	// 			<< " data points from test_pcd.pcd with the following fields: "
	// 			<< std::endl;

	cv::Mat cv_image_pcd=cv::Mat( 240, 240, CV_8UC3, cv::Scalar::all(0));
	cv::Mat cv_image=cv::Mat( 240, 240, CV_8UC3, cv::Scalar::all(0));
	float x_max=0.126043; float x_min=-0.269949;
	float y_max=-0.326809; float y_min=-0.699874;
	float z_max=0.200383; float z_min=-0.136344;
	//int pcd_x, pcd_y, pcd_z;

	int counterOut = 0;
	for (const auto& point: *cloud) //186
    {
		//std::cout << "    " << point.x << " "<< point.y << " "<< point.z << std::endl;
		//scale 
		int pcd_x = (point.x-x_min)/1.2*240;//(x_max-x_min)*240;
		int pcd_y = (point.y-y_min)/1.2*240;//(y_max-y_min)*240;
		int pcd_z = (point.z-z_min)/1.2*240;//(z_max-z_min)*240;
		//scale= 127/ std::max(-lapmin,lapmax);
		// if (point.x>x_max) x_max=point.x; 
		// if (oinpt.x<x_min) x_min=point.x; 
		// if (point.y>y_max) y_max=point.y; 
		// if (point.y<y_min) y_min=point.y;  
		// if (point.z>z_max) z_max=point.z; 
		// if (point.z<z_min) z_min=point.z;
		cv_image_pcd.at<double>(cv::Point(pcd_y, pcd_x))= (255,255,255);

		//data.at<double>(cv::Point(counter, 2))= event_timestamp/final_timestamp;//normalized
		//data.at<double>(cv::Point(counterOut, 0))= event_timestamp/final_timestamp;//normalized
		data.at<double>(cv::Point(counterOut, 1))= (double)pcd_x/240;
		data.at<double>(cv::Point(counterOut, 2))= (double)pcd_x/240;
		data.at<double>(cv::Point(counterOut, 3))= (double)pcd_x/240;
		data.at<double>(cv::Point(counterOut, 0))=0;//0.1*counterOut;//
		double tau = 10000;
		//data.at<double>(cv::Point(counterOut, 2))= exp(-(final_timestamp-event_timestamp)/tau);//normalized

		counterOut++;
	}
	// std::cout<<"x_min="<<x_min<<",x_max="<<x_max<<std::endl;
	// std::cout<<"y_min="<<y_min<<",y_max="<<y_max<<std::endl; 
	// std::cout<<"z_min="<<z_min<<",z_max="<<z_max<<std::endl;
	// cv::namedWindow("image_pcd_xy", CV_WINDOW_AUTOSIZE);
	// cv::imshow("image_pcd_xy", cv_image_pcd);
	// cv::waitKey(0);

	cv::Mat clusterCenters;
	std::vector<double> clustCentX, clustCentY, clustCentZ, clustCentT;
	//std::vector<double> prev_clustCentX, prev_clustCentY;

	std::vector<int> point2Clusters;
	std::vector<int> positionClusterColor;
	std::vector<int> assign_matches;

	cv::Mat TclusterVotes;

	std::cout << "start" <<std::endl;
	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	meanshiftCluster_Gaussian(data, &clustCentT, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth, TclusterVotes);
	std::chrono::high_resolution_clock::time_point end =std::chrono::high_resolution_clock::now();
	elapsedTime += std::chrono::duration_cast<std::chrono::duration<float>>(end - start).count();
	t_count++;
	std::cout << "Elapsed time: " << elapsedTime << " s, count=" <<t_count<< std::endl;

	std::cout<<"point_cloud rostopic finished"<<std::endl;

	//-------------------------------
	// merge close centroids of all data
	int ClusterID=0;
		int sign=0;
		if (clustCentX.size()>0)
		{
			for(int i=0;i<clustCentX.size();i++)
			{
				int x1=clustCentX[i];
				int y1=clustCentY[i];
				std::cout<<"x1="<<x1<<",y1="<<y1<<std::endl;
				for(int j=i;j<clustCentX.size();j++)
				{
					int x2=clustCentX[j];
					int y2=clustCentY[j];
					int dis=pow((x1-x2),2)+pow((y1-y2),2);
		
					if (dis<(bandwidth/2)*(bandwidth/2)*240*240)// && x1!=0 && y1!=0)//800)// 400 last working //i!=j && 
					{
						x1=	(x1+clustCentX[j])/2;	//x1=centroid_filter[i][0];
						y1=	(y1+clustCentY[j])/2;	//y1=centroid_filter[i][1];
						clustCentX[j]=0;clustCentY[j]=0;	
						//cv::circle(cv_image.image, cv::Point(x1,y1), 1, cv::Scalar(0,255,0), 5, 8, 0);
						sign++;
					}
				}
				if (x1!=0 && y1!=0)
				{
					ClusterID++;
					centroid_final.push_back({x1,y1});//ClusterID,<<centroid_final[1][1]
					cv::circle(cv_image, cv::Point(x1,y1), 1, cv::Scalar(255,255,255), 5, 8, 0);//------------------
				}
			}
		}
		std::cout<<"centroid_final.size="<<centroid_final.size()<<",ClusterID="<<ClusterID<<std::endl;
		cv::namedWindow("cv_image", CV_WINDOW_AUTOSIZE);
		cv::imshow("cv_image", cv_image);
		cv::waitKey(0);

		int cs=centroid_final.size(); */

	
	
}
/* void Meanshift::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
	ROS_INFO("Camera Info Received");        
} */

void Meanshift::eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg)
{
	ROS_INFO("hao");

	//cv::Mat clusters=cv::Mat( 2, events_size, CV_64F, cv::Scalar::all(0.));

	height=msg->height; //180
	width=msg->width; //240
	events_size=msg->events.size();

	cv_bridge::CvImage cv_image;
    cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);//CV_8U-->mono
    cv_image.image = cv::Scalar(128, 128, 128);//cv::Scalar(128);

	// double event_timestamp =  (1E-6*(double)(msg->events[counterIn].ts.toNSec()-first_timestamp));//now in usecs 
	// ts = (1E-6*(double)(msg->events[i].ts.toNSec())) - firsttimestamp;

	// if(pcl_sign!=pcl_sign_prev) // obtained data from point cloud
	// {
		ROS_INFO("subscribed point_cloud");

		for (int i = 0; i < events_size; ++i)
		{
			const int x = msg->events[i].x;
			const int y = msg->events[i].y;

			cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = ( \
			msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));

			// clusters.at<double>(cv::Point(i, 0))= x;
			// clusters.at<double>(cv::Point(i, 1))= y;
		}
		std::cout<<"subscribed from point cloud"<<std::endl;
		cv::namedWindow("image", CV_WINDOW_AUTOSIZE);//CV_WINDOW_AUTOSIZE
		cv::imshow("image", cv_image.image);
		cv::waitKey(1);

	/* if (depth_sign==1)
	{
		cv::namedWindow("image_events", CV_WINDOW_AUTOSIZE);//CV_WINDOW_AUTOSIZE
		cv::imshow("image_events", cv_image.image);
		cv::waitKey(0);

		// cv::namedWindow("image_depth", CV_WINDOW_AUTOSIZE);
		// cv::imshow("image_depth", depth_image);
		// cv::waitKey(0);
	} */

		/* if(firstevent)
		{
			firsttimestamp = (1E-6*(double)(msg->events[10].ts.toNSec()));
			firstevent = false;
		}

		//double last_timestamp =  (1E-6*(double)(msg->events[counterIn-1].ts.toNSec()));//now in usecs
		cv::Mat clusterCenters;
		// cv::Mat segmentation=cv::Mat(numRows, numCols, CV_8UC3);
		// segmentation = cv::Scalar(128,128,128);

		// cv::Mat traj = cv::Mat(numRows, numCols, CV_8UC3);
		// traj = cv::Scalar(128,128,128);

		std::vector<double> clustCentX, clustCentY, clustCentZ, clustCentT;
		//std::vector<double> prev_clustCentX, prev_clustCentY;

		std::vector<int> point2Clusters;
		std::vector<int> positionClusterColor;
		std::vector<int> assign_matches;

		cv::Mat TclusterVotes;

		std::cout << "start-111" <<std::endl;
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		meanshiftCluster_Gaussian(data, &clustCentT, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth, TclusterVotes);
		std::chrono::high_resolution_clock::time_point end =std::chrono::high_resolution_clock::now();
		elapsedTime += std::chrono::duration_cast<std::chrono::duration<float>>(end - start).count();
		t_count++;
		std::cout << "start-112" <<std::endl;
		std::cout << "Elapsed time: " << elapsedTime << " s, count=" <<t_count<< std::endl; */

	// } // end-if(pcl_sign!=pcl_sign_prev) 


	pcl_sign_prev = pcl_sign; 
	depth_sign=0;
} //end-void


} //namespace
