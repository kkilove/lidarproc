#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <string>


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
//精度
float dis_accu = 0.0;
int frame_accu = 0;
#define H_accu  15.4
#define PI  3.1415926
//水平角分辨率
int v_angle_roi=2; //水平角度统计范围
float x_roi=tan(v_angle_roi*PI/180)*H_accu;
float v_resulotion_accu = 0.0;
int frame_v_resolution = 0;

void dis_accuracy(const pcl::PointCloud<LASER_POINT_NEW>::Ptr in_cloud_ptr) //4  //精度
{
  float x, y,z;
  float dis = 0.0;
  int flag = 0;
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
  {
    x = in_cloud_ptr->points[i].x;
    y = in_cloud_ptr->points[i].y;
    z = in_cloud_ptr->points[i].z;
    if(x<0.1 && x>-0.1 && z<0.1 && z>-0.1 && y>0.2)
    {
      dis = dis + y;
      flag++;
    }
  }
  if(flag>=10)
  {
    dis = dis/flag;
    dis = fabs(dis-H_accu);
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
    if(x<x_roi&& x>-x_roi && z<0.1 && z>-0.1 && y>0.2)
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
      if(x<x_roi&& x>-x_roi && z<0.1 && z>-0.1)
      {
        flag++;
      }
    }
  }
  if(flag>=10)
  {
    v_resulution = v_angle_roi*2/flag;
    v_resulotion_accu=v_resulotion_accu+v_resulution;
    // ROS_INFO("v_resulotion_accu:%f", v_resulution);
    // ROS_INFO("flag:%d", flag);
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
  ros::Time::init();
  ros::init(argc, argv, "lidar_proc_node");
  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");
  ros::Subscriber subscriber = h.subscribe("/point_raw", 1, lidar_callback);
  //ROS_INFO("%f", x_roi);
  ros::spin();
  return 0;
}  
