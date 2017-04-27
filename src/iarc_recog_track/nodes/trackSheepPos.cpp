#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "highgui.h"
#include "colotracker.h"
#include "region.h"
#include <string>
#include "iarc_recog_track/my_Point.h"
#include <boost/thread/mutex.hpp>
using namespace cv;
namespace enc = sensor_msgs::image_encodings; 

class ImageTracker
{
public:
    ImageTracker(ros::NodeHandle& nn)
    {
        image_transport::ImageTransport it(nn); 
        chatter_pub = nn.advertise<geometry_msgs::Point>("/iarc/sheep/center",20);
        img_pub = it.advertise("/iarc/sheep/timg",1);
        subp = nn.subscribe("my_Point",1,&ImageTracker::pointCallback, this);
        subI = it.subscribe("color", 1, &ImageTracker::imageCallback, this);
        subdI = it.subscribe("/iarc/sheep/delay_img", 1, &ImageTracker::delay_imgCallback, this);
        g_trackerInitialized = false;
    }
    void pointCallback(const iarc_recog_track::my_Point::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock_(mutex_);
        ROS_INFO("Hearing my_Point :%lf, %lf, %lf, %lf",msg->mle,msg->mto, msg->mri, msg->mbo);
        Mat img_t = img.clone();
        g_tracker.init(img_t, msg->mle,msg->mto, msg->mri, msg->mbo);
        g_trackerInitialized = true;
    }
    void delay_imgCallback(const sensor_msgs::ImageConstPtr& original_image)
    {
        boost::mutex::scoped_lock lock_(mutex_);
        cv_bridge::CvImagePtr cv_ptr;    
            try{
            cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);    
        }    
        catch (cv_bridge::Exception& e) {    
            ROS_ERROR("cv_bridge exception: %s", e.what());    
            exit(-1);    
        } 
        ROS_INFO("Restarting tracking");  
        cv::Mat tmp = cv_bridge::toCvShare(original_image, "bgr8")->image;
        if (tmp.rows == 0 || tmp.cols == 0) return;
        img = tmp.clone();
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& original_image) 
    {    
        boost::mutex::scoped_lock lock_(mutex_);
        cv_bridge::CvImagePtr cv_ptr;    
            try{
            cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);    
        }    
        catch (cv_bridge::Exception& e) {    
            ROS_ERROR("cv_bridge exception: %s", e.what());    
            exit(-1);    
        }  
        cv::Mat tmp = cv_bridge::toCvShare(original_image, "bgr8")->image;
        if (tmp.rows == 0 || tmp.cols == 0) return;
        img = tmp.clone();
        if (g_trackerInitialized){
            img_k = img.clone();
            bb = g_tracker.track(img_k);
        }
        if (bb != NULL){
            cv::rectangle(img_k, Point2i(bb->x, bb->y), Point2i(bb->x + bb->width, bb->y + bb->height), Scalar(255, 0, 0), 3);
            if (ros::ok())
            {
                geometry_msgs::Point pt;
                pt.x=bb->x+bb->width/2;
                pt.y=bb->y+bb->height/2;
                ROS_INFO("Track center : %lf, %lf", pt.x,pt.y);
                chatter_pub.publish(pt);
                sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_k).toImageMsg();
                img_pub.publish(imgmsg);
            }
            delete bb;
            bb = NULL;
        }
    } 
private:
    ros::Publisher chatter_pub;
    image_transport::Publisher img_pub;
    ros::Subscriber subp;
    image_transport::Subscriber subI;
    image_transport::Subscriber subdI;
    cv::Mat img;
    cv::Mat img_k;
    bool g_trackerInitialized;
    ColorTracker g_tracker;
    cv::Point *PointList;
    BBox *bb = NULL;

    boost::mutex mutex_;
};



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "trackSheepPos");
    ros::NodeHandle nn;
    ImageTracker itn(nn);
    ros::spin();
    return 0;
}
