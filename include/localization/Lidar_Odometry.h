
#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ctime>

using namespace std;

#define DEBUG_ENABLE 0

// 激光里程计类
class Lidar_odometry
{
public:  
  Lidar_odometry()  
  {  
    scan_new = false ;                              // 是否有新的激光帧
    LO_INIT = false;                                // 激光里程计初始化                          
    pos_pub = nh.advertise<geometry_msgs::Pose> ("NDT_location", 1);                                  // 初始化发布器      
    laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("rslidar_points", 1, &Lidar_odometry::scan_cb, this);   // 激光雷达的订阅   
    LO_predict =  nh.advertise<geometry_msgs::Pose> ("LO_predict", 1);   
 //   cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("PointCloud_output", 1);        // 点云的显示
    /************************************这里还是先简单的把初值设置为0吧*********************************************/
 //   Eigen::AngleAxisf predict_rotation(M_PI/2, Eigen::Vector3f::UnitX()) ;              // 旋转的预测值 
 //   Eigen::Quaternionf q (0.99930137466394, -0.0302042760550403, -0.00750171424198088, 0.0206927181922387);  
 //   Eigen::AngleAxisf predict_rotation(q) ;     
    curr_ScanFrame = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }  
  
  bool scan_new ;                              // 是否有新的激光帧
  bool LO_INIT ;                                // 激光里程计初始化
  Eigen::Matrix4f T_curr,T_ref;                        // 参考帧的位姿与当前帧的位姿
  Eigen::Matrix4f T_relative;                          // 两帧间的相对运动
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_ScanFrame;  // 雷达的当前帧   
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_ScanFrame;   // 雷达的参考帧     
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;    // 定义ndt对象   
  void LO_test_set();                                      
  void LO_initialize();
  void LO();
  // LO的重载
  // 输入当前帧，得到当前帧与参考帧间的运动
  Eigen::Matrix4f LO(pcl::PointCloud<pcl::PointXYZ>::Ptr curr_ScanFrame);
  void LO_();
  //回调函数  接收发送过来的点云msg  
  void scan_cb (const sensor_msgs::PointCloud2 cloud_msg);
  void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t);

private:  
  ros::NodeHandle nh;   
  ros::Publisher pos_pub;       // 创建一个结果的发布者  
  ros::Subscriber laser_scan;   // 创建一个订阅者  订阅话题laser_scan
  ros::Publisher cloud_pub;
  ros::Publisher LO_predict;    // 发送给地图匹配的里程计预测位姿
  /**********************增加赋初值*************************************/
  Eigen::AngleAxisf predict_rotation;                 // 旋转的预测值 
  Eigen::Translation3f predict_translation;           // 位置的预测值

};

#endif



