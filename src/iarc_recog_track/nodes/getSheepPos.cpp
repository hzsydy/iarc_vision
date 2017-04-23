#include <opencv2/opencv.hpp>  
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>  
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream> 
#include <cstring>
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings; 

const int MAXINTERVAL = 2;
const int MAXBLOCK = 300;
const int MINBLOCK = 10;

class ImageMatcher 
{
public:
	ImageMatcher(ros::NodeHandle& nn)
    {
        image_transport::ImageTransport it(nn); 
    	img_pub = it.advertise("/iarc/sheep/delay_img",1);
		pub_lefttop = nn.advertise<geometry_msgs::Point>("Point_lefttop", 20);
		pub_rightbottom = nn.advertise<geometry_msgs::Point>("Point_rightbottom", 20);
    	subI = it.subscribe("/iarc/realsense/color", 1, &ImageMatcher::imageCallback, this);

    }
	void imageCallback(const sensor_msgs::ImageConstPtr& original_image) {    
    	cv_bridge::CvImagePtr cv_ptr;    
    	try{
        	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);    
    	}    
    	catch (cv_bridge::Exception& e) {    
       	 	ROS_ERROR("ROSOpenCV::getSheepPos.cpp::cv_bridge exception: %s", e.what());    
        	exit(-1);    
    	}	 
    	ROS_INFO("Get Next Picture.");
    	cv::Mat tmp = cv_bridge::toCvShare(original_image, "bgr8")->image;
    	if (tmp.rows == 0 || tmp.cols == 0) return;
    	Mat image = tmp.clone();
    	trackonce(image);
	}
	void trackonce(Mat &image)
	{
		int s, t;
		Mat blank = image.clone();
		Mat sk = image.clone();
		Mat hsvimage;
	
		cout << "Size:" << image.size() << endl;
		cout << "Type:" << image.type() << endl;
		cvtColor(image, hsvimage, CV_BGR2HSV);

		cout << (int)hsvimage.at<Vec3b>(0, 0).type << endl;
		int ma = 0, mi = 360;
		for (int i = 0; i < image.rows; ++i)
			for (int j = 0; j < image.cols; ++j)
			{
				if (checkRed(hsvimage,i,j))
				{
					blank.at<Vec3b>(i, j).val[0] = 0;
					blank.at<Vec3b>(i, j).val[1] = 0;
					blank.at<Vec3b>(i, j).val[2] = 0;
				}
				else
				{
					blank.at<Vec3b>(i, j).val[0] = 255;
					blank.at<Vec3b>(i, j).val[1] = 255;
					blank.at<Vec3b>(i, j).val[2] = 255;
				}
				sk.at<Vec3b>(i, j).val[0] = 0;
				sk.at<Vec3b>(i, j).val[1] = 0;
				sk.at<Vec3b>(i, j).val[2] = 0;
			}
		for (int i = 0; i < image.rows; ++i)
			for (int j = 0; j < image.cols; ++j)
				if (sk.at<Vec3b>(i, j).val[0] == 0 && checkRed(hsvimage, i, j))
				{
					int num = 1, le = j, ri = j, to = i, bo = i;
					s = 0; t = 1; q1[1] = i; q2[1] = j;
					sk.at<Vec3b>(i, j).val[0] = 1;
					sk.at<Vec3b>(i, j).val[1] = 1;
					sk.at<Vec3b>(i, j).val[2] = 1;
					while (s < t)
					{
						++s;	
						int ii = q1[s], jj = q2[s];
						for (int pt = -MAXINTERVAL; pt <= MAXINTERVAL; ++pt)
							for (int qt = -MAXINTERVAL; qt <= MAXINTERVAL; ++qt)
								if (ii + pt >= 0 && ii + pt < image.rows && jj + qt >= 0 && jj + qt < image.cols)
								{
									if (sk.at<Vec3b>(ii + pt, jj + qt).val[0] == 0 && checkRed(hsvimage, ii + pt, jj + qt))
									{
										sk.at<Vec3b>(ii + pt, jj + qt).val[0] = 1;
										sk.at<Vec3b>(ii + pt, jj + qt).val[1] = 1;
										sk.at<Vec3b>(ii + pt, jj + qt).val[2] = 1;
										++t;
										q1[t] = ii + pt;
										q2[t] = jj + qt;
										++num;
										if (le > jj + qt) le = jj + qt;
										if (ri < jj + qt) ri = jj + qt;
										if (to > ii + pt) to = ii + pt;
										if (bo < ii + pt) bo = ii + pt;
									}
								}
					}
					if (num >= MINBLOCK && num <= MAXBLOCK)
					{
						geometry_msgs::Point lefttop;
						lefttop.x = le;
						lefttop.y = to;
						lefttop.z = 1;
						geometry_msgs::Point rightbottom;
						rightbottom.x = ri;
						rightbottom.y = bo;
						rightbottom.z = 1.0;
						sensor_msgs::ImagePtr Imsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

						pub_lefttop.publish(lefttop);
						pub_rightbottom.publish(rightbottom);
						img_pub.publish(Imsg);
						ROS_INFO("Get Pos Once");
						//cv::rectangle(image, Point2i(le, to), Point2i(ri, bo), Scalar(255, 0, 0), 2);
					}
				}
	}
private:
	image_transport::Publisher img_pub;
	ros::Publisher pub_lefttop;
	ros::Publisher pub_rightbottom;
	image_transport::Subscriber subI;
	int q1[50000], q2[50000];

	bool checkRed(Mat a, int i, int j)
	{	
		if (((int)a.at<Vec3b>(i, j).val[0] <= 12.5 || (int)a.at<Vec3b>(i, j).val[0] >= 167.5) && (int)a.at<Vec3b>(i, j).val[1] >= 77)
			return true;
		return false;
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "getSheepPos");
    ros::NodeHandle nn;
    ImageMatcher im(nn);
	ros::spin();
	return 0;
}
