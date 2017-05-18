// FILE: "Leg_detector_node"
// AUTHOR: Marco Antonio Becerra Pedraza (http://www.marcobecerrap.com)
// MODIFIED BY: Ferdian Jovan (http://github.com/ferdianjovan)
// SUMMARY: This program receives the LaserScan msgs and executes a leg detector algorithm
// > to search for persons. At the end publishes a a vector with all the persons found
// > and their position relative to the sensor.
//
// NOTES: This leg detector is based on the work described in:
// Bellotto, N. & Hu, H.
// Multisensor-Based Human Detection and Tracking for Mobile Service Robots
// IEEE Trans. on Systems, Man, and Cybernetics -- Part B, 2009, 39, 167-181

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>

#define PI 3.1416
#define minpts 5
#define sigma 0.05
#define post_min 0.03
#define post_max 0.08
// #define safty_dis 1.5

using namespace std;

bool sensor_on   = false;
int g_counter = 0;

string sensor_frame_id;
sensor_msgs::LaserScan SensorMsg;
boost::mutex mutex;

vector < double >  laser_x;
vector < double >  laser_y;
vector < double >  laser_r;
vector < double >  laser_t;
vector < double > rec_x;
vector < double > rec_y;

vector < geometry_msgs::Pose > HumanPoseVector;
vector < geometry_msgs::Pose > Laser_Vector;
vector < geometry_msgs::Pose > laser_output;
geometry_msgs::Point HumanPoint;
geometry_msgs::Quaternion HumanQuaternion;
geometry_msgs::Pose Laserpose;

vector<int> mark ;
list<int> stack;
double beta;//angle_increment

void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
bool is_nan(double x);
bool is_finite(double x);
double Dist2D( double x0, double y0, double x1, double y1 );
void classfy(int signal, int position, int first, int *group);
void danger_direction();// the unmaned plane cannot to near to things
void choose(int begin , int end ,double aver_x , double aver_y , int number);//delete untipical data by length
// void choose_angle();//delete untipical data by detceting points' number
// void choose_direction();//choose viable direction after delete data

int main(int argc, char **argv){

  ros::init(argc, argv, "laser_detector");
  ros::NodeHandle n;
  ros::Publisher  node_pub = n.advertise <geometry_msgs::PoseArray>("laser", 10000); // Humans in the environment

  // get param from launch file
  string laser_scan = "/scan";
  ros::param::get("~laser_scan", laser_scan);
  ros::Subscriber node_sub = n.subscribe(laser_scan, 10000, LaserCallback);
  geometry_msgs::PoseArray msgx;
  ros::Rate loop_rate(10);//frequency:1/0.1

  int seq_counter = 0;

  while( ros::ok() ){
    if( sensor_on == true ){
      int signal = 1;//group number
      vector<int> start;//the start position of each group
      vector<int> stop;//the stop position of each group

      //classify
    int i = 0;
    int stack_number;
    int group_number = 0;
    // //delete
    // HumanPoseVector.clear();
    // //delete

      while (i < laser_r.size())
      {
        if (stack.empty())//zhan is empty
        {
          if (laser_r[i] == 10)//useless data
          {i++;  mark[i]=-10;}
          else if (mark[i] != 0)//visited
          {
            i++;
          }
          else {//unvisited
            stack.push_back(i);
            classfy(signal,i,0,&group_number);
          }
        }
        else//core pionts_group:signal
        {
          while(!stack.empty())
          {
            stack_number = stack.front();
            classfy(signal,stack_number,1,&group_number);
            stack.sort();
            stack.unique();
          }
          double mid_x,mid_y;
          mid_x = HumanPoint.x/group_number;
          mid_y = HumanPoint.y/group_number;

          // HumanPoint.x = mid_x * cos( mid_y );
          // HumanPoint.y = mid_x * sin( mid_y );
          int sig1 = 0;
          int sig2 = mark.size()-1;
          while (mark[sig1]!=signal)
          {sig1++;}
          while (mark[sig2]!=signal)
          {sig2--;}

          //choose class :find the zhuzi
          choose(sig1 , sig2 , mid_x , mid_y , group_number);

          i++;
          signal++;
          group_number = 0;
        }
      }//classfy

      // //the result of choose
      // HumanPoseVector.clear();
      // HumanPoseVector.assign(Laser_Vector.begin() , Laser_Vector.end());
      // Laser_Vector.clear();
      // int midd = HumanPoseVector.size();
      // for (int i = 0 ; i < midd ; i++)
      // {
      //   for( int j = HumanPoseVector[i].orientation.x ; j < HumanPoseVector[i].orientation.y ; j++)
      //   {
      //     double mid1,mid2;
      //     mid1 = laser_r[j];
      //     mid2 = laser_t[j];
      //       HumanPoint.x = mid1 * cos( mid2 );
      //       HumanPoint.y = mid1 * sin( mid2 );
      //       HumanPoint.z = 1;
      //       HumanQuaternion.x = 0;
      //       HumanQuaternion.y = 0;
      //       HumanQuaternion.z = 0;
      //       HumanQuaternion.w = 1;
      //
      //       geometry_msgs::Pose HumanPose;
      //       HumanPose.position = HumanPoint;
      //       HumanPose.orientation= HumanQuaternion;
      //       Laser_Vector.push_back( HumanPose );
      //   }//int j >sig1 <sig2
      // }//int i < midd
//reback

      // Header config
      msgx.header.stamp = ros::Time::now();
      msgx.header.frame_id = SensorMsg.header.frame_id;
      msgx.header.seq = seq_counter;
      msgx.poses = Laser_Vector;
      node_pub.publish( msgx );
      sensor_on = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
    seq_counter++;
  }

  return 0;
}

bool is_nan(double x){
  return(x==x);
}

bool is_finite(double x){
    return (x <= 10 && x >= 0);
}

//get data and ransac
void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){

  // To get header data from sensor msg
  SensorMsg = *msg;
  laser_r.clear();
  laser_t.clear();
  laser_x.clear();
  laser_y.clear();

  rec_x.clear();
  rec_y.clear();

  mark.clear();
  stack.clear();

  HumanPoseVector.clear();
  Laser_Vector.clear();
  laser_output.clear();

  sensor_on = true;

  double px, py, pr, pt;

  beta = msg->angle_increment;

  mark.assign(msg->ranges.size(),0);//visited and not visited;

  for( unsigned i = 2; i < msg->ranges.size()-2; i++ ){
    pt = msg->angle_min + ( i * msg->angle_increment);//angle

    if (is_nan(msg->ranges[i]) &&  is_finite(msg->ranges[i]))
    {
      double a[5];//calman mid_fliter
      int b[5];
      double number;
      int mid_min = i - 2;
      int mid_max = i + 2;
      for( unsigned m = mid_min; m < mid_max; m++ )
      {
        number = msg->ranges[m];
        if (is_nan(number) && (is_finite(number)))
        {
          a[m-mid_min] = number;
        }
        else{a[m-mid_min] = 10;}
        b[m-mid_min] = m;
      }
      double min;
      int weizhi;
      int temp;
      for (int m = mid_min; m < mid_min + 3 ;m++)
      {
        min = a[m-mid_min];
        weizhi = m;
        for (int j = m+1 ; j<mid_min + 4 ;j++)
        {
          if(min>a[j-mid_min])
          {weizhi = j; min = a[j-mid_min]; }
        }

        a[weizhi-mid_min] = a[m-mid_min];
        a[m-mid_min] = min;
        temp = b[weizhi-mid_min];
        b[weizhi-mid_min] = b[m-mid_min];
        b[m-mid_min] = temp;
      }

      pr = a[2];//distance
      if (pr == 10)
      {laser_r.push_back(msg->ranges[i]);laser_t.push_back(pt);}
      else
      {laser_r.push_back( pr );     laser_t.push_back(pt);}
    }
    else
    {
      laser_r.push_back(10);     laser_t.push_back(pt);
    }
  }

  // // Filtering laser scan
  // LaserFilter_Mean( &laser_r, FILTER_SIZE );
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){
    px = laser_r[ i ] * cos( laser_t[ i ] );
    py = laser_r[ i ] * sin( laser_t[ i ] );
    laser_x.push_back( px );
    laser_y.push_back( py );
  }
}

void classfy(int signal, int position, int first, int *group){
  int k = position;
  int sig = signal;
  int min;
  int max;

    min = (k - minpts < 0)? 0 : k-minpts;
    max = (k + minpts) > laser_r.size() ? laser_r.size() : k+minpts;
    int amount = 0;
    int dis[2*minpts + 1] = {0};

    for (unsigned i = min; i< max ;i++)
    {
      double mid = Dist2D(laser_x[k],laser_y[k],laser_x[i],laser_y[i]);
      if (mid < sigma)
      {
        amount++;
        dis[i - min] = 1;
      }
    }

    stack.pop_front();
    if (first == 1)
    {
      (*group) = (*group) + 1 ; 	 HumanPoint.x += laser_r[ k ];    HumanPoint.y += laser_t[ k ];
      mark[k] = signal;

      if (amount>=minpts)//core point
      {
        for (unsigned i = min; i< max ;i++)
        {
          if (dis[i - min] == 1 && (mark[i] == 0 || mark[i] == -1)) {stack.push_back(i);}
        }
      }
    }//first==1
    else
    {
      if (amount >= minpts) //initial core point
      {
        mark[k] = sig;
        (*group) = 1;
        HumanPoint.x = laser_r[ k ];  HumanPoint.y = laser_t[ k ];  HumanPoint.z = signal;
        for (unsigned i = min; i< max ;i++)
        {
          if (dis[i - min] == 1 && (mark[i] == 0 || mark[i] == -1)){stack.push_back(i);}
        }
      }
      else
      {
        mark[k] = -1;
      }
    }//first==0

}//classfy
//
// void danger_direction(){
//   int i = 0;
//   int danger_begin;
//   int danger_end;
//   while ( i < laser_r.size())
//   {
//     if (laser_r[i] < safty_dis)
//     {
//       danger_begin = i;
//       danger_end = i;
//       int mid_i = i + 1;
//       int safty_count = 0;
//       while (safty_count < minpts && mid_i < laser_r.size())
//       {
//         safty_count++;
//         if (laser_r[mid_i] < safty_dis) {safty_count = 0; danger_end = mid_i ;}
//         mid_i++;
//       }
//       HumanPoint.x = laser_r[danger_begin];
//       HumanPoint.y = laser_t[danger_begin];
//       for (unsigned j = danger_begin + 1 ; j < danger_end ; j++)
//       {
//         if (laser_r[j] < HumanPoint.x)
//         {HumanPoint.x = laser_r[j]; HumanPoint.y = laser_t[j];}
//       }
//       HumanPoint.z = danger_end - danger_begin + 1;
//       HumanQuaternion.x = danger_begin;
//       HumanQuaternion.y = danger_end;
//       HumanQuaternion.z = laser_r[danger_begin];
//       HumanQuaternion.w = laser_r[danger_end];
//       Laserpose.position = HumanPoint;
//       Laserpose.orientation = HumanQuaternion;
//       Laser_Vector.push_back(Laserpose);
//       i = danger_end + 1;
//     }//danger_direction
//     else i++;
//   }//search
// }

void choose(int start , int end , double aver_x , double aver_y , int number){
  geometry_msgs::Pose HumanPose;
  float radius_start;
  float radius_end;
  float theta;
  float length;//the size of the class

  radius_start = laser_r[start];
  radius_end = laser_r[end];
  theta = laser_t[end] - laser_t[start];

  //find class
  length = sqrt( pow(radius_start , 2) + pow(radius_end , 2) - 2 * radius_end * radius_start * cos( theta ));

  if (length < post_max && length > post_min)//find
  {
    HumanPoint.x = aver_x;
    HumanPoint.y = aver_y;
    HumanPoint.z = number;
    //reback
    HumanQuaternion.x = laser_r[start];//|
    HumanQuaternion.y = laser_t[start];//|
    //reback
    HumanQuaternion.z = laser_r[end];//|
    HumanQuaternion.w = laser_t[end];//|

    HumanPose.position = HumanPoint;
    HumanPose.orientation= HumanQuaternion;
    Laser_Vector.push_back(HumanPose);
  }

}

// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}
