#include <ros/ros.h>

#include <iostream>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


class RealsenseImgParser
{
public:
    RealsenseImgParser(const ros::NodeHandle& nh)
    {
        image_transport::ImageTransport it(nh);
        sub_color_ = it.subscribe("/camera/color/image_raw", 1, &RealsenseImgParser::colorImgCB, this, std::string("raw"));
        sub_depth_ = it.subscribe("/camera/depth/image_raw", 1, &RealsenseImgParser::depthImgCB, this, std::string("raw"));
    }
    
    ~RealsenseImgParser() { }
    
    void colorImgCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
        
        cv::imshow("color", cv_ptr->image);
    }
    
    void depthImgCB(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat depth_img = cv_ptr->image; //CV_16UC1, 480*360
                
        cv::imshow("depth", depth_img);
        cv::waitKey(33);
    }


private:

    image_transport::Subscriber sub_color_;
    image_transport::Subscriber sub_depth_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_simple_test", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    
    RealsenseImgParser rip(n);
    
    ros::spin();

    return 0;
}
