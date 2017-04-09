#include <ros/ros.h>    
#include <image_transport/image_transport.h>    
#include <cv_bridge/cv_bridge.h>    
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/core/core.hpp> 
#include <opencv2/opencv.hpp>       
#include <iostream>        
#include <string>        
#include <sstream>      
#include <opencv2/highgui/highgui.hpp>    
#include "opencv2/video/tracking.hpp"    
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>    
#include <std_msgs/UInt8.h>    

using namespace cv;    
using namespace std;    
namespace enc = sensor_msgs::image_encodings;    

static string ImgInfo = "~/Upan.png";
cv::Point *PointList;

cv::Point* MTemplate(Mat src);  
void imageCallback(const sensor_msgs::ImageConstPtr& original_image) {    
	cv_bridge::CvImagePtr cv_ptr;    
	try{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);    
	}    
	catch (cv_bridge::Exception& e) {    
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());    
		exit(-1);    
	} 
	ROS_INFO("Next step MTemplate");  
	PointList = MTemplate(cv_bridge::toCvShare(original_image, "bgr8")->image);

}    

cv::Point* MTemplate(Mat src) {  
	Mat templ;      
	Mat result;    
	int match_method = 0;    
	templ = imread(ImgInfo,1);   
	int result_cols = src.cols - templ.cols + 1;    
	int result_rows = src.rows - templ.rows + 1; 
	ROS_INFO("%d %d",templ.cols,templ.rows);   
	result.create( result_cols, result_rows, CV_32FC1 ); 
	ROS_INFO("Next step matchTemplate");   
	matchTemplate( src, templ, result, match_method );    
	normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );    

	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;    
	Point matchLoc;    
	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );    

	/// 对于方法 SQDIFF 和 SQDIFF_NORMED, 越小的数值代表更高的匹配结果. 而对于其他方法, 数值越大匹配越好    
	if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ) {   
		matchLoc = minLoc;   
	}    
	else {  
	matchLoc = maxLoc;   
	} 
	static cv::Point Point_List[2];
	Point_List[0] = cv::Point( matchLoc.x, matchLoc.y);
	Point_List[1] = cv::Point( matchLoc.x + templ.cols, matchLoc.y + templ.rows );
	return(Point_List);
}  

int main(int argc, char **argv) {    

	ros::init(argc, argv, "Shift");    
	ROS_INFO("-----------------");    

	ros::NodeHandle nh;    
	image_transport::ImageTransport it(nh);    

	image_transport::Subscriber sub = it.subscribe("colorimage", 1, imageCallback);
	ros::Publisher pub_lefttop = nh.advertise<geometry_msgs::Point>("geometry_msgs/Point_lefttop", 1000);
	ros::Publisher pub_rightbottom = nh.advertise<geometry_msgs::Point>("geometry_msgs/Point_rightbottom", 1000);
	ros::Rate loop_rate(10);
	while ( ros::ok() ){
		geometry_msgs::Point lefttop;
		lefttop.x = PointList[0].x;
		lefttop.y = PointList[0].y;
		lefttop.z = 1.0;
		geometry_msgs::Point rightbottom;
		rightbottom.x = PointList[0].x;
		rightbottom.y = PointList[0].y;
		rightbottom.z = 1.0;
		pub_lefttop.publish(lefttop);
		pub_lefttop.publish(rightbottom);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}    
