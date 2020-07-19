#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;


class cloud_pub
{
  public:
    cloud_pub()
    {
        file = nh.param<string>("Lidar_localization/globalmap_pcd", "-");       // 全局句柄读取地图路径
   //     nh.param<std::string>("file_path", file, "/home/gogo/ZhongheLou2.pcd");                                
        std::cout<<"map file path: "<< file << std::endl;
        map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
   //     scan_cloud = pcl::PointCloud<pcl::PointXYZ>();
        scan_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    
    string file;                // 文件名
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;             // 地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud;
    void read_cloud_point(std::string const &file_path);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Map_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    void Pose_callback(geometry_msgs::Pose car_pose);
    void scan_callback (const sensor_msgs::PointCloud2 cloud_msg);
    void initialize();

  private:
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("PointCloud_output", 100);   // 创建一个点云的发布者  发布的topic是PointCloud_output
    ros::Subscriber localization = nh.subscribe<geometry_msgs::Pose>("NDT_location", 1, &cloud_pub::Pose_callback, this);// 创建一个订阅者 获取激光的定位
    ros::Subscriber laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("rslidar_points", 1, &cloud_pub::scan_callback, this);     
};



// 读取点云数据
// 返回Ptr  即模板类PointCloud的智能指针    指向PointCloud<PointT>
void cloud_pub::read_cloud_point(std::string const &file_path){
   
//    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);             // 地图
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *map_cloud) == -1)      // 读取点云文件
    {
        ROS_INFO("Couldn't read the pcd file\n");
        return;
    }
    /*
    for (int j = 0; j < map->points.size(); j += 1)
    {
    pcl::PointXYZRGB p;
    p.x = map->points[j].x;
    p.y = map->points[j].y;
    p.z = map->points[j].z;
    p.r = 0;
    p.g = 200;
    p.b = 0;
    map_cloud->points.push_back(p);
    }
    */
    ROS_INFO("PCD read OK!\n");
}

// 地图变换
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub::Map_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{  
   // 构造变换矩阵
   // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
   // 旋转向量
   //Eigen::AngleAxisd rotation_vector(M_PI/2, Eigen::Vector3d(1,0,0));     // 绕x轴旋转90度
   //T.rotate(rotation_vector);       // 旋转
   //T.pretranslate(Eigen::Vector3d(0,0,0));      // 平移
   pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   Eigen::Affine3f transform = Eigen::Affine3f::Identity();
   transform.translation() << 0.0, 0.0, 0.0;      // 平移  
   transform.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));   // 旋转
   pcl::transformPointCloud(*input_cloud, *map_cloud, transform); 
   return map_cloud;
}

// 保存最新的一帧点云
void cloud_pub::scan_callback (const sensor_msgs::PointCloud2 cloud_msg)
{
//  pcl::PointCloud<pcl::PointXYZ> scan_cloud;                 // 扫描数据
  pcl::PCLPointCloud2 pcl_cloud;                             // sensor_msgs::PointCloud2首先转换为PCLPointCloud2----->PointCloud<pcl::PointXYZ>
  pcl_conversions::toPCL(cloud_msg,pcl_cloud);
  pcl::fromPCLPointCloud2(pcl_cloud,*scan_cloud);            // 转换为PCL的类型 
/*
  for (int j = 0; j < scan_cloud.points.size(); j += 1)
  {
    pcl::PointXYZRGB p;
    p.x = scan_cloud.points[j].x;
    p.y = scan_cloud.points[j].y;
    p.z = scan_cloud.points[j].z;
    p.r = 255;//红色
    p.g = 0;
    p.b = 0;
    colorScan->points.push_back(p);
  }*/

}

// 接收到最新的位姿
void cloud_pub::Pose_callback(geometry_msgs::Pose car_pose)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f cloud_pose = Eigen::Matrix4f::Identity() ;        // 变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // 旋转
    Eigen::Quaternionf q;                             // 接收四元数
    q.x() = car_pose.orientation.x;
    q.y() = car_pose.orientation.y;
    q.z() = car_pose.orientation.z;
    q.w() = car_pose.orientation.w;
    transform.rotate(q.matrix());                             // 旋转
    transform.translation() << car_pose.position.x, car_pose.position.y, car_pose.position.z;      // 平移    

    pcl::transformPointCloud(*scan_cloud, *trans_cloud, transform); 
 //   colorScan->clear();
    *sum_cloud = *map_cloud + *trans_cloud;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*sum_cloud, output);
    output.header.stamp=ros::Time::now(); 
    output.header.frame_id = "map";
    pcl_pub.publish(output);
}

void cloud_pub::initialize()
{
  read_cloud_point(file);
  if(map_cloud==nullptr){
    cout<<"MAP read error!"<<endl;
    return;
  }    
  std::cout<<"cloud size "<< map_cloud->size() << std::endl;    
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "map_pub");
    ROS_INFO("Started PCL read node");  
    
    cloud_pub cp;
 //   nh.param<std::string>("file_path", file, "/home/lwh/桌面/NDT_PCL_demo/cloud1.pcd");        // 读取参数文件的file_path  
 //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
 //   pcl::copyPointCloud(*target_cloud, *target_cloudRGB);
 //   auto map_cloud = Map_Rotation(original_cloud);
 //   map_cloud = Map_transform(map_cloud);      // 对地图进行处理
 ///   pcl::io::savePCDFileASCII("/home/lwh/Zhonghelou/ZhongheLou2.pcd", *map_cloud);     // 保存地图
 //   pcl_pub.publish(output);               // 发送地图
 ///   ros::spin();
    // 地图固定发送频率为1HZ
    cp.initialize();      
    ros::Rate loop_rate(100);                    
    while (ros::ok())
    {  
        ros::spinOnce();                     // 信息回调处理   这个函数不会阻塞  
        loop_rate.sleep();
    }
    return 0;
}