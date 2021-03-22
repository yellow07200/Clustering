#include "meanShift-huang-3D.hpp"


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dvs_test");

	ros::NodeHandle nh;
	//ros::NodeHandle nh_private("~");

	dvs_test::Meanshift meanshift(nh);//, nh_private

	ros::spin();

	return 0;
}
