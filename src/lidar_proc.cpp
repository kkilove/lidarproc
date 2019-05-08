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
double z_roi_d= 0.0;// dianxing -1.75
float z_roi_u=-0.1;
float y_roi=0.1;
//精度
float dis_accu = 0.0;
int frame_accu = 0;
double  H_accu =6.341;
#define PI  3.1415926

//zongxiang角分辨率
float h_resulotion_accu = 0.0;
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
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;
int flag4 = 0;
void dis_accuracy(const pcl::PointCloud<LASER_POINT_NEW>::Ptr in_cloud_ptr) //4  //精度
{
  float x, y,z;
  float dis = 0.0;
  float dis2=0.0;
  float D_corr = 0.0;
  float dis_lidar = 0.0;
  int laser_number=0;
  int alpha=0;

  float t_cos_thita=0;
  float t_sin_thita=0;
  float t_t_thita=0;
  bool data_flag=false;
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {
    x = in_cloud_ptr->points[i].x;
    y = in_cloud_ptr->points[i].y;
    z = in_cloud_ptr->points[i].z;
    if(x<x_roi_r && x>x_roi_l && z<z_roi_u && z>z_roi_d && y>0)
    {
      data_flag=true;
      flag4++;
      laser_number=  in_cloud_ptr->points[i].laserid;

      dis_accu = dis_accu + fabs(y-H_accu);
      dis_lidar=in_cloud_ptr->points[i].range;
      dis2 = dis2 + fabs(dis_lidar-H_accu);
      if(dis_lidar<12.0)
      {
        t_t_thita=(float)thita[laser_number];
        int t_thita=t_t_thita*(H_accu-0.0012)/(H_accu-0.0985);
        if (t_thita<0)
        {
          t_cos_thita=cos_rot_table_[-t_thita];
          t_sin_thita=-sin_rot_table_[-t_thita];
        }
        else
        {
          t_cos_thita=cos_rot_table_[t_thita];
          t_sin_thita=sin_rot_table_[t_thita];
        }

      }
      else
      {
        t_cos_thita=cos_thita_value[laser_number];
        t_sin_thita=sin_thita_value[laser_number];
      }
      alpha=in_cloud_ptr->points[i].h_angle*100;
      D_corr=H_accu/cos_rot_table_[alpha]/t_cos_thita-dis_lidar;
      outfile<<"ID:"<<laser_number<<"; dis:"<<dis_lidar<<"; instensity:"<<in_cloud_ptr->points[i].v_angle<<"; TDC/AD:"<<in_cloud_ptr->points[i].point_time<<"; bis:"<<H_accu-y<<"; h_angle:"<<alpha<<"; D_corr:"<<D_corr<<endl;

    }
  }
  if(data_flag)
  {
    flag1++;
    data_flag=false;
  }
  if(flag1>=200)
  {
    dis_accu = dis_accu/flag4;
    dis2 = dis2/flag4;
    ROS_INFO("dis1_accu:%f", dis_accu);
    ROS_INFO("dis2_accu:%f", dis2);
    flag1=0;
    flag4=0;
    dis_accu=0;
    dis2=0;
  }

}

void vangle_resulution(const pcl::PointCloud<LASER_POINT_NEW>::Ptr in_cloud_ptr) //4  //水平角分辨率
{
  float x, y,z;
  float dis_23 = 0.0;
  float z_23 = 0.0;
  float dis_32 = 0.0;
  float z_32 = 0.0;
  int id=0;
  int flag23=0;
  int flag32=0;
  bool dataflag=false;
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {

    x = in_cloud_ptr->points[i].x;
    y = in_cloud_ptr->points[i].y;
    z = in_cloud_ptr->points[i].z;
    id=  in_cloud_ptr->points[i].laserid;
    if(x<0.04 && x>-0.04 && y>0)
    {
      if (id==23)
      {
        dis_23 = dis_23 + y;
        z_23=z_23+fabs(z);
        flag23++;
        dataflag=true;
      }
      if (id==32)
      {

        dis_32 = dis_32 + y;
        z_32=z_32+fabs(z);
        flag32++;
        dataflag=true;
      }
    }
  }
  if(dataflag)
  {
    if (flag23>0 &&flag32>0)
    {
      h_resulotion_accu=h_resulotion_accu+(atan((z_23/flag23)/H_accu)+atan((z_32/flag32)/H_accu))*180/3.1416;
      flag3++;
      dataflag=false;
    }
  }

  if(flag3>=200)
  {
    float t=h_resulotion_accu/flag3;

    ROS_INFO("h_resulotion:%f", t);
    h_resulotion_accu=0;
    flag3=0;
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
  ros::Time::init();
  ros::init(argc, argv, "lidar_proc_node");
  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");

  private_nh.param("H_accu", H_accu, 5.55);
  private_nh.param("z_roi_d", z_roi_d, -2.2);



  char* filename = new char[32];
  sprintf(filename, "%2f.txt", H_accu);

  outfile.open(filename);
  outfile_dis_accu.open("dis_accu.txt");
  InitTables_chaowu();

  ROS_INFO("z_roi_d:%f,H_accu:%f",z_roi_d,H_accu);
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
