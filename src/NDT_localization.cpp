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

//#include <eigen3/Eigen/Geometry>

using namespace std;

#define DEBUG_ENABLE 1

// 读取地图数据
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

class NDT
{
public:  
  NDT()  
  {  
    NDT_pub = nh.advertise<geometry_msgs::Pose> ("NDT_location", 1);                                  // 初始化发布器      
   // laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("laser_scan", 1, &NDT::scan_cb, this);     // 初始化订阅器     这个写法比较特殊必须要这样写
    laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("rslidar_points", 1, &NDT::scan_cb, this);     

    /***************************************设置在地图上的初始坐标*********************************************/
 //   Eigen::AngleAxisf predict_rotation(M_PI/2, Eigen::Vector3f::UnitX()) ;              // 旋转的预测值 
 //   Eigen::Quaternionf q (0.99930137466394, -0.0302042760550403, -0.00750171424198088, 0.0206927181922387);  
 //   Eigen::AngleAxisf predict_rotation(q) ; 
    predict_rotation = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,0,1));
    predict_translation = Eigen::Translation3f(0,0,0);
    
  }  
  bool scan_new = false ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_scan ;                                 // 激光当前帧
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;    // 定义ndt对象
  void ndt_solve();
  //回调函数  接收发送过来的点云msg  用于定位
  void scan_cb (const sensor_msgs::PointCloud2 cloud_msg);
  // 读取用于定位的地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_readMap();
  // ndt的配置
  void ndt_Config(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Map_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
  void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t );

private:  
  ros::NodeHandle nh;   
  ros::Publisher NDT_pub;       // 创建一个NDT结果的发布者  
  ros::Subscriber laser_scan;   // 创建一个订阅者  订阅话题laser_scan

  /**********************增加赋初值*************************************/
  Eigen::AngleAxisf predict_rotation;                 // 旋转的预测值 
  Eigen::Translation3f predict_translation;           // 位置的预测值

};
                                                                                                                                                                           
//回调函数  接收发送过来的点云msg   并计算位姿
void NDT::scan_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  static int t=0;
  scan_new = true;                              // 接收标志为置为一
  pcl::PCLPointCloud2 pcl_cloud;                             // sensor_msgs::PointCloud2首先转换为PCLPointCloud2----->PointCloud<pcl::PointXYZ>
  pcl_conversions::toPCL(cloud_msg,pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud,*scan_cloud);            // 转换为PCL的类型 
  // 对输入点云进行滤波   使点云的数量减少到10%      便于计算
  // 点云降采样滤波  这里会产生很多小三维体素包围点云，对于一个三维体素内部的点云只用体素的重心处的一个点来替代
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);          // 滤波后的点云
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                                // 声明一个对pcl::PointXYZ滤波的滤波器
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);                                                                                              // 体素的大小                                                                                 

  approximate_voxel_filter.setInputCloud(scan_cloud);        // 设置滤波的点云
  approximate_voxel_filter.filter(*filtered_cloud);          // 输出                 
  std::cout<<"I receive laser data: "<< filtered_cloud->size()<<"t: "<<t << std::endl;
  ndt.setInputSource(filtered_cloud);                        // 设置为输入点云
  t++;
}

void NDT::ndt_solve()
{
  //  设置NDT算法的初值   没有初值也能运行  但是一个好的初值能帮助更好的定位  实际中通常用航迹推算方法计算初始位姿
  //  Eigen::AngleAxisf init_rotation(0.6331, Eigen::Vector3f::UnitZ());  // AngleAxisf旋转向量   0.3 ~ 1.03   17度～59度  +-20度能拉回                           // 绕Z轴
  //  Eigen::Translation3f init_translation (2.3, 0.720047, 0);           //  测试x轴 0.3-4.2
  scan_new = false ;
  const clock_t begin_time = clock() ;
  Eigen::Matrix4f predict_pose = (predict_translation * predict_rotation).matrix();
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);     // 输入点云 经过配准后的结果
  ndt.align(*result_cloud, predict_pose);   // 进行配准  将filtered_cloud对齐后输出为result_cloud

  float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
  cout << "time："<<seconds << endl;

  // 获取变换矩阵
  Eigen::Matrix<float, 4, 4> T = ndt.getFinalTransformation();                               // 输出的T是map->base_link
  cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()               // ndt.hasConverged() 成功则返回1
              << "score: " << ndt.getFitnessScore() << std::endl;                            // 输出得分   得分在0.3以上匹配就不好了
  // 提取出R和t
  geometry_msgs::Pose car_pose;
  Eigen::Matrix3f R = T.block<3,3>(0,0);       // 提取T中 从(0,0)开始的3*3矩阵   !!!!!!!!!注意这里Matrix3f的类型一定要与T的类型匹配  T是float，如果这里是Matrix3d, 则会出现大版的错误
  Eigen::Vector3f t = T.block<3,1>(0,3);    
 
//  show_cloud(filtered_cloud, R, t);
  /***********************增加赋初值的操作*******************************/
  predict_rotation.fromRotationMatrix(R);
  predict_translation = Eigen::Translation3f(t(0), t(1), t(2));

  #if DEBUG_ENABLE==1  
  cout<<"T: "<<T<<endl;
  cout<<"R: "<<R<<endl;
  cout<<"t: "<<t<<endl;
  #endif
  // 将Eigen类型的变换矩阵  转换为geometry_msgs::Pose并发布出去  话题NDT_location 
  car_pose.position.x = t(0);
  car_pose.position.y = t(1);
  car_pose.position.z = t(2);
  Eigen::Quaternionf q(R);     // 注意  四元数有两种类型 Quaternionf和Quaterniond  后面的f和d表示float和double那么用于构造四元数的对象也必须要是同类型的
  car_pose.orientation.x = q.x();
  car_pose.orientation.y = q.y();
  car_pose.orientation.z = q.z();
  car_pose.orientation.w = q.w();
  NDT_pub.publish(car_pose);
}

// 参数： target_cloud： 即地图点云
void NDT::ndt_Config(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
   ndt.setTransformationEpsilon(0.01);  // 定义变换矢量[x, y, z, roll, pitch, yaw]的最小变化量 当变化量小于该值时算法停止
   ndt.setStepSize(0.1);                // 设置优化的最大步长 
   ndt.setResolution(1.0);              // 设置网格化立体的边长
   ndt.setMaximumIterations(35);        // 迭代次数
   ndt.setInputTarget(target_cloud);    // 对齐的目标点云
}

// 读取用于定位的地图
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT::NDT_readMap()
{
    string file;
//  nh.param<std::string>("file_path", file, "/home/lwh/桌面/NDT_PCL_demo/cloud1.pcd");    // 读取参数文件的file_path
    nh.param<std::string>("file_path", file, "/home/lwh/Zhonghelou/ZhongheLou2.pcd");    // 读取参数文件的file_path
    std::cout<<"map file path: "<< file << std::endl;
    // Loading first scan.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1)      // 读取点云文件
    {
        ROS_INFO("Couldn't read the pcd file\n");
        return nullptr;
    }
    ROS_INFO("PCD read OK!\n");
    if(cloud==nullptr)    
       return 0;
    std::cout<<"map cloud size "<< cloud->size() << std::endl; 
    return cloud;
}

// PointCloud变换
pcl::PointCloud<pcl::PointXYZ>::Ptr NDT::Map_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
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

// 输入点云转换到Map坐标系中
void NDT::show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << t(0), t(1), t(2);      // 平移  
    transform.rotate (R);   // 旋转
    //transform.translation() << 0.0, 0.0, 0.0;       // 平移  
    //transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));   // 旋转
    pcl::transformPointCloud(*scan_cloud, *map_cloud, transform); 
    // 转换为ROS数据
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*map_cloud, output);      
    output.header.stamp=ros::Time::now(); 
    output.header.frame_id = "map";
  //  cloud_pub.publish(output);
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "NDT_localization");
    ROS_INFO("Started NDT localization node");   
    NDT ndt_loc;                         // 实例化一个ndt对象
    auto map = ndt_loc.NDT_readMap();    // 读取地图
    ndt_loc.ndt_Config(map);             // ndt参数配置  设置地图为map 
    // 循环频率
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if(ndt_loc.scan_new == true)     // 当有新数据接收后
            ndt_loc.ndt_solve();
        ros::spinOnce();                 // 信息回调处理   这个函数不会阻塞   每次调用   会一次性将消息队列中全部数据通过多次调用回调函数处理完
        loop_rate.sleep();
    }
    
    return 0;
}
