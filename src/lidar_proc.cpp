#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <math.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <string>
#include <iostream>
#include <fstream>


using namespace ros;
using namespace std;

struct LASER_POINT_NEW
{
  PCL_ADD_POINT4D;

  float    intensity;                 ///< laser intensity reading
  float    range;
  float    v_angle;
  float    h_angle;
  int  laserid;                      ///< laser ring number
  unsigned int point_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(LASER_POINT_NEW,// 注册点类型宏
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, v_angle, v_angle)
                                  (float, h_angle, h_angle)
                                  (float, range, range)
                                  (int, laserid, laserid)
                                  (unsigned int, point_time, point_time)
                                  )

//std_msgs::Header _velodyne_header;
ofstream outfile;
ofstream outfile_dis_accu;
//确定目标区域
float x_roi_l= -0.1;float x_roi_r=0.1;
float z_roi_d= -2.2;float z_roi_u=0.3;
float y_roi=4;
//精度
float dis_accu = 0.0;
int frame_accu = 0;
double  H_accu =6.341;
#define PI  3.1415926
//水平角分辨率
float v_angle_roi=1.0; //水平角度统计范围
float x_v_roi=tan(v_angle_roi*PI/180.0)*H_accu;
float v_resulotion_accu = 0.0;
int frame_v_resolution = 0;

//
void InitTables_chaowu();
static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
static const int Num_Of_Detector=64;
static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
float sin_rot_table_[ROTATION_MAX_UNITS];
float cos_rot_table_[ROTATION_MAX_UNITS];

int thita[Num_Of_Detector];
float cos_thita_value[Num_Of_Detector];
float sin_thita_value[Num_Of_Detector];


void dis_accuracy(const pcl::PointCloud<LASER_POINT_NEW>::Ptr in_cloud_ptr) //4  //精度
{
  float x, y,z;
  float dis = 0.0;
  float D_corr = 0.0;
  float dis_lidar = 0.0;
  int laser_number=0;
  int alpha=0;
  int flag = 0;
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {
    x = in_cloud_ptr->points[i].x;
    y = in_cloud_ptr->points[i].y;
    z = in_cloud_ptr->points[i].z;
    if(x<x_roi_r && x>x_roi_l && z<z_roi_u && z>z_roi_d && y>0.3)
    {
      dis = dis + y;
      flag++;
      laser_number=  in_cloud_ptr->points[i].laserid;
      dis_lidar=in_cloud_ptr->points[i].range;
      alpha=in_cloud_ptr->points[i].h_angle*100;
      D_corr=H_accu/cos_rot_table_[alpha]/cos_thita_value[laser_number]-dis_lidar;
      outfile<<"ID:"<<laser_number<<"; dis:"<<dis_lidar<<"; y:"<<y<<"; bis:"<<H_accu-y<<"; Intensity:"<<in_cloud_ptr->points[i].intensity<<"; h_angle:"<<alpha<<"; D_corr:"<<D_corr<<endl;
    }
  }
  if(flag>=10)
  {
    dis = dis/flag;
    // ROS_INFO("dis_avg:%f,  Flag:%d", dis,flag);
    dis = fabs(dis-H_accu);
    outfile_dis_accu<<dis<<endl;
    dis_accu = dis_accu + dis;
    // ROS_INFO("dis_accu:%f", dis);
    frame_accu++;
  }
  if(frame_accu>10)
  {
    dis_accu = dis_accu / frame_accu;
    ROS_INFO("dis_accu:%f", dis_accu);
    frame_accu = 0;
  }
}
void vangle_resulution(const pcl::PointCloud<LASER_POINT_NEW>::Ptr in_cloud_ptr) //4  //水平角分辨率
{
  float x, y,z;
  float v_resulution = 0.0;
  int flag = 0;
  int id=0;
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {
    x = in_cloud_ptr->points[i].x;
    y = in_cloud_ptr->points[i].y;
    z = in_cloud_ptr->points[i].z;
    if(x<x_roi_r && x>x_roi_l && z<z_roi_u && z>z_roi_d && y>y_roi)
    {
      id=in_cloud_ptr->points[i].laserid;
      break;
    }
  }
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {
    int t_id=in_cloud_ptr->points[i].laserid;
    if (t_id==id)
    {
      x = in_cloud_ptr->points[i].x;
      y = in_cloud_ptr->points[i].y;
      z = in_cloud_ptr->points[i].z;
      if(x<x_v_roi&& x>-x_v_roi && z<z_roi_u && z>z_roi_d && y>H_accu-1)
      {
        flag++;
      }
    }
  }
  if(flag>2)
  {
    v_resulution = v_angle_roi*2/flag;
    v_resulotion_accu=v_resulotion_accu+v_resulution;
    frame_v_resolution++;
  }
  if(frame_v_resolution>10)
  {
    v_resulotion_accu = v_resulotion_accu / frame_v_resolution;
    ROS_INFO("v_resulotion_accu:%f", v_resulotion_accu);
    frame_v_resolution = 0;
  }
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{ 
  pcl::PointCloud<LASER_POINT_NEW>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<LASER_POINT_NEW>);
  pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
  dis_accuracy(current_sensor_cloud_ptr);
  vangle_resulution(current_sensor_cloud_ptr);
}

int main(int argc, char **argv) 
{ 
  outfile.open("data.txt");
  outfile_dis_accu.open("dis_accu.txt");
  InitTables_chaowu();
  ros::Time::init();
  ros::init(argc, argv, "lidar_proc_node");
  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");
  private_nh.param("H_accu", H_accu, 5.55);
  x_v_roi=tan(v_angle_roi*PI/180.0)*H_accu;
  ROS_INFO("x_v_roi:%f,H_accu:%f", x_v_roi,H_accu);
  ros::Subscriber subscriber = h.subscribe("/point_raw", 1, lidar_callback);
  //ROS_INFO("%f", x_roi);
  ros::spin();
  return 0;
}  

void InitTables_chaowu()
{

  /////  l209
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index)
  {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  memset(thita,0,sizeof(int)*Num_Of_Detector);
  memset(cos_thita_value,0,sizeof(float)*Num_Of_Detector);
  memset(sin_thita_value,0,sizeof(float)*Num_Of_Detector);


  int start=-940;//245-- -940
  for (int i=0;i<16;++i)
  {
    thita[i]=start+i*79;
  }

  start=-268;//285-- -268
  for (int i=0;i<8;++i)
  {
    thita[i+16]=start+i*79;
  }
  start=-900;// -347--- -900
  for (int i=0;i<8;++i)
  {
    thita[i+24]=start+i*79;
  }

  start=-2209;// -1024--- -2209
  for (int i=0;i<16;++i)
  {
    thita[i+32]=start+i*79;
  }

  start=-1538;// -1538--- -985
  for (int i=0;i<8;++i)
  {
    thita[i+48]=start+i*79;
  }
  start=-2170;// -2170--- -1617
  for (int i=0;i<8;++i)
  {
    thita[i+56]=start+i*79;
  }



  for (int i=0;i<Num_Of_Detector;++i)
  {
    if (thita[i]<0)
    {
      cos_thita_value[i]=cos_rot_table_[-thita[i]];
      sin_thita_value[i]=-sin_rot_table_[-thita[i]];
    }
    else
    {
      cos_thita_value[i]=cos_rot_table_[thita[i]];
      sin_thita_value[i]=sin_rot_table_[thita[i]];
    }
  }

}
