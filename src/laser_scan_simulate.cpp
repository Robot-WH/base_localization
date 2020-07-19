
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace std;
// 读取点云数据
// 返回Ptr  即模板类PointCloud的智能指针    指向PointCloud<PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(std::string const &file_path){
    // Loading first scan.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1)      // 读取点云文件
    {
        ROS_INFO("Couldn't read the pcd file\n");
        return nullptr;
    }
    ROS_INFO("PCD read OK!\n");
    return cloud;       // 返回Ptr指针
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "laser_scan_simulate");

    ROS_INFO("Started Lidar simulate node");
    
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("rslidar_points", 1);     // 创建一个点云的发布者  发布的topic是pcl_output
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    string file;
    nh.param<std::string>("file_path", file, "/home/lwh/Zhonghelou/scan.pcd");      // 读取参数文件的file_path
    std::cout<<"map file path: "<< file << std::endl;
    auto target_cloud = read_cloud_point(file);                                              // 读取目标点云   地图
    if(target_cloud==nullptr)    
       return 0;
    std::cout<<"cloud size "<< target_cloud->size() << std::endl;

    sensor_msgs::PointCloud2 output;      // ROS需要的点云数据格式
    pcl::toROSMsg(*target_cloud, output);

    output.header.stamp=ros::Time::now(); 
    output.header.frame_id = "laser_link";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
