#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <sstream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>



using namespace ros;
pcl::PointCloud<pcl::PointXYZ> _cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&_cloud);

class calculate_altitude_publish
{
public:
	calculate_altitude_publish()
	{
		sub = nh.subscribe ("/iarc/realsense/points", 1, &calculate_altitude_publish::cloud_cb,this);
    	pub=nh.advertise<std_msgs::Float32>("/iarc/realsense/altitude",1000);

    	subimg=nh.subscribe("/camera/color/image_raw",1,&calculate_altitude_publish::image_cb,this);
    	image_transport::ImageTransport it(nh);
  		pubimg = it.advertise("/iarc/realsense/color", 1);
  	}

 	void cloud_cb (const sensor_msgs::PointCloud2 input)
  	{
    	// Create a container for the data
    	pcl::PCLPointCloud2 pcl_pc;
	    pcl_conversions::toPCL(input, pcl_pc);
	    pcl::fromPCLPointCloud2(pcl_pc, _cloud);

	    //wave filter1
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PassThrough<pcl::PointXYZ> pass;
	    pass.setInputCloud (cloud);
	    pass.setFilterFieldName ("z");
	    pass.setFilterLimits (-2.5, 2.5);//height limit 5
	    //pass.setFilterLimitsNegative (true);
	    pass.filter (*cloud_filtered);

	    //wave filter2
	    /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
	    fbf.setInputCloud (cloud);
    	fbf.setSigmaS (5.0);
    	fbf.setSigmaR (0.03);
    	fbf.filter (*cloud_filtered2);*/

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    	sor.setInputCloud(cloud_filtered); 
    	sor.setMeanK(50);
    	sor.setStddevMulThresh(1);
    	sor.filter(*cloud_filtered2);  


    	//select plane
	    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	    // Create the segmentation object
	    pcl::SACSegmentation<pcl::PointXYZ> seg;
	    // Optional
	    seg.setOptimizeCoefficients (true);
	    // Mandatory
	    seg.setModelType (pcl::SACMODEL_PLANE);
	    seg.setMethodType (pcl::SAC_RANSAC);
	    seg.setDistanceThreshold (0.1);
	    seg.setInputCloud (cloud_filtered2);
	    seg.segment (*inliers, *coefficients);

	    if (inliers->indices.size () == 0)
	    {
	    	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	    	return;
	    }

	    /*std::cout << "Model iacoefficients: " << float(coefficients->values[0]) << " " 
	                                        << float(coefficients->values[1]) << " "
	                                        << float(coefficients->values[2]) << " " 
	                                        << float(coefficients->values[3]) << std::endl;*/
	    float a=float(coefficients->values[0]);
	    float b=float(coefficients->values[1]);
	    float c=float(coefficients->values[2]);
	    float d=float(coefficients->values[3]);
	    float height=-d/sqrt(a*a+b*b+c*c);;
	    std_msgs::Float32 altitude;
	    if(height>0.4)
	    altitude.data=height;
		else 
		altitude.data=11111;
	   	std::cout<<"********altitude*********:  "<<altitude.data<<std::endl;
	   	pub.publish(altitude);
	}

  	void image_cb(const sensor_msgs::ImageConstPtr& msg)
  	{
      	cv_bridge::CvImagePtr cv_ptr;
      	try {
        	cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
        
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

        sensor_msgs::ImagePtr msgimg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
        pubimg.publish(msgimg);
    }
    
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  ros::Subscriber subimg;
  //ros::Publisher pubimg;
  image_transport::Publisher pubimg;
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  
  //new object
  calculate_altitude_publish cap;  

  // Spin
  ros::spin ();
}

