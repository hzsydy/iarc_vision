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

using namespace cv;
namespace enc = sensor_msgs::image_encodings; 

cv::Point *PointList;
cv::Point t_topLeft(0,0);
cv::Point t_botRight(0,0);
cv::Point g_topLeft(0,0);
cv::Point g_botRight(0,0);
cv::Point g_botRight_tmp(0,0);
cv::Mat img;
cv::Mat img_k;
bool plot = false;
bool g_trackerInitialized = false;
ColorTracker * g_tracker = NULL;

static void onMouse( int event, int x, int y, int, void* param)
{
    cv::Mat img = ((cv::Mat *)param)->clone();
    if( event == cv::EVENT_LBUTTONDOWN && !g_trackerInitialized){
        std::cout << "DOWN " << std::endl;
        g_topLeft = Point(x,y);
        plot = true;
    }else if (event == cv::EVENT_LBUTTONUP && !g_trackerInitialized){
        std::cout << "UP " << std::endl;
        g_botRight = Point(x,y);
        plot = false;
        if (g_tracker != NULL)
            delete g_tracker;
        g_tracker = new ColorTracker();
        g_tracker->init(*(cv::Mat *)param, g_topLeft.x, g_topLeft.y, g_botRight.x, g_botRight.y);
        g_trackerInitialized = true;
    }else if (event == cv::EVENT_MOUSEMOVE && !g_trackerInitialized){
        //plot bbox
        g_botRight_tmp = Point(x,y);
        // if (plot){
        //     cv::rectangle(img, g_topLeft, current, cv::Scalar(0,255,0), 2);
        //     imshow("output", img);
        // }
    }
}

void pointCallback1(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO("Hearing geo/Pt_lt:%lf, %lf",msg->x,msg->y);
    t_topLeft.x = msg->x;
    t_topLeft.y = msg->y;
}

void pointCallback2(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO("Hearing geo/Pt_rb:%lf, %lf",msg->x,msg->y);
    t_botRight.x = msg->x;
    t_botRight.y = msg->y;
    g_topLeft = cv::Point(0,0);
    g_botRight_tmp = cv::Point(0,0);
    if (g_tracker != NULL)
        delete g_tracker;
    g_tracker = new ColorTracker();
    g_tracker->init(img, t_topLeft.x, t_topLeft.y, t_botRight.x, t_botRight.y);
    g_trackerInitialized = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image) {    
    cv_bridge::CvImagePtr cv_ptr;    
    try{
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);    
    }    
    catch (cv_bridge::Exception& e) {    
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());    
        exit(-1);    
    } 
    ROS_INFO("Get Next Picture.");  
    cv::Mat tmp = cv_bridge::toCvShare(original_image, "bgr8")->image;
    if (tmp.rows == 0 || tmp.cols == 0) return;
    img = tmp;
} 


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "trackSheepPos");
    ros::NodeHandle nn;
    image_transport::ImageTransport it(nn); 
    ros::Publisher chatter_pub = nn.advertise<geometry_msgs::Point>("/iarc/sheep/center",20);
    image_transport::Publisher img_pub = it.advertise("/iarc/sheep/timg",1);
    
    BBox *bb = NULL;
    
    
    /*int captureDevice = 0;
    if (argc > 1)
        captureDevice = atoi(argv[1]);

    cv::VideoCapture webcam = cv::VideoCapture(captureDevice);
    webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!webcam.isOpened()){
        webcam.release();
        std::cerr << "Error during opening capture device!" << std::endl;
        return 1;
    }*/

    cv::namedWindow( "output", 0 );
    cv::setMouseCallback( "output", onMouse, &img);
    img = cv::Mat(640,480,CV_8UC3,cv::Scalar(0,0,0));
    
    
    ros::Subscriber sub1 = nn.subscribe("Point_lefttop", 1, pointCallback1);
    ros::Subscriber sub2 = nn.subscribe("Point_rightbottom", 1, pointCallback2);
    image_transport::Subscriber subI = it.subscribe("/iarc/realsense/color", 1, imageCallback);
            
    ros::Rate loop_rate(10);
    for(;ros::ok();){
        //webcam >>img;
        
            
        int c = waitKey(10);
        if( (c & 255) == 27 ) {
            std::cout << "Exiting ..." << std::endl;
            break;
        }
        //some control
        switch( (char)c ){
        case 'i':
            g_trackerInitialized = false;
            g_topLeft = cv::Point(0,0);
            g_botRight_tmp = cv::Point(0,0);
            break;
        default:;

        }
        if (img.rows == 0 || img.cols == 0) continue; 
        if (g_trackerInitialized && g_tracker != NULL){
            img_k = img.clone();
            bb = g_tracker->track(img_k);
        }
        if (!g_trackerInitialized && plot && g_botRight_tmp.x > 0){
            cv::rectangle(img, g_topLeft, g_botRight_tmp, cv::Scalar(0,255,0), 2);
        }
        if (bb != NULL){
            cv::rectangle(img_k, Point2i(bb->x, bb->y), Point2i(bb->x + bb->width, bb->y + bb->height), Scalar(255, 0, 0), 3);
            if (ros::ok())
            {
                geometry_msgs::Point pt;
                pt.x=bb->x+bb->width/2;
                pt.y=bb->y+bb->height/2;
                ROS_INFO("%lf %lf", pt.x,pt.y);
                chatter_pub.publish(pt);
                sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_k).toImageMsg();
                img_pub.publish(imgmsg);
            }
            
            delete bb;
            bb = NULL;
        }
        ros::spinOnce();
        if (!(img_k.rows == 0||img_k.cols == 0))
            cv::imshow("output", img_k);

    }

    if (g_tracker != NULL)
        delete g_tracker;
    //ros::spin();
    return 0;
}
