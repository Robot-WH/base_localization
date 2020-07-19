
#include "Lidar_Odometry.h"

// 单独雷达里程计测试的必要设置
void Lidar_odometry::LO_test_set()
{
    pos_pub = nh.advertise<geometry_msgs::Pose> ("NDT_location", 1);                                             // 初始化发布器      
    laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("rslidar_points", 1, &Lidar_odometry::scan_cb, this);   // 激光雷达的订阅    
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("PointCloud_output", 1);                                 // 点云的显示   
}

// 参数： target_cloud： 即地图点云
void Lidar_odometry::LO_initialize()
{
   ndt.setTransformationEpsilon(0.01);  // 定义变换矢量[x, y, z, roll, pitch, yaw]的最小变化量 当变化量小于该值时算法停止
   ndt.setStepSize(0.1);                // 设置优化的最大步长 
   ndt.setResolution(1.0);              // 设置网格化立体的边长
   ndt.setMaximumIterations(35);        // 迭代次数
   // 当前帧和参考帧位姿初始化为单位阵
   T_curr = Eigen::Matrix4f::Identity();
   T_relative = T_ref = T_curr;         // 赋值
   ndt.setInputTarget(curr_ScanFrame);  // 当前点云设置为参考点云
   LO_INIT = true;
   cout<<"Lidar Odometry initialize success!"<<endl;
}

//回调函数  接收发送过来的点云msg   并计算位姿
void Lidar_odometry::scan_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  static int t=0;
  scan_new = true;                                           // 接收标志为置为一
  pcl::PCLPointCloud2 pcl_cloud;                             // sensor_msgs::PointCloud2首先转换为PCLPointCloud2----->PointCloud<pcl::PointXYZ>
  pcl_conversions::toPCL(cloud_msg, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud,*scan_cloud);            // 转换为PCL的类型                      
  // 对输入点云进行滤波   使点云的数量减少到10%      便于计算
  // 点云降采样滤波  这里会产生很多小三维体素包围点云，对于一个三维体素内部的点云只用体素的重心处的一个点来替代
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);          // 滤波后的点云
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                                // 声明一个对pcl::PointXYZ滤波的滤波器
  approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);                                                                                              // 体素的大小                                                                                 

  approximate_voxel_filter.setInputCloud(scan_cloud);             // 设置滤波的点云
  approximate_voxel_filter.filter(*filtered_cloud);               // 输出                 
//  std::cout<<"I receive laser data: "<< filtered_cloud->size()<<"t: "<<t << std::endl;
  curr_ScanFrame = filtered_cloud;    
  t++;
}

void Lidar_odometry::LO()
{
  double score;
  //  设置NDT算法的初值   没有初值也能运行  但是一个好的初值能帮助更好的定位  实际中通常用航迹推算方法计算初始位姿
  //  Eigen::AngleAxisf init_rotation(0.6331, Eigen::Vector3f::UnitZ());  // AngleAxisf旋转向量   0.3 ~ 1.03   17度～59度  +-20度能拉回                           // 绕Z轴
  //  Eigen::Translation3f init_translation (2.3, 0.720047, 0);           //  测试x轴 0.3-4.2
  if(scan_new == false)  return;    // 如果没有新激光数据
  scan_new = false;    
  if(LO_INIT == false)              // 如果LO还没初始化
  {
    LO_initialize();                // 初始化
    return;     
  }
  ndt.setInputSource(curr_ScanFrame);                        // 如果不是第一帧  则设置为输入点云
  const clock_t begin_time = clock();
//  Eigen::Matrix4f predict_pose = (predict_translation * predict_rotation).matrix();
  Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();                                // 两两帧匹配的初值设置为单位阵
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);     // 输入点云经过配准后的结果
  ndt.align(*result_cloud, predict_pose);   // 进行配准  将filtered_cloud对齐后输出为result_cloud
  // 测试时间
  float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
  cout << "time："<<seconds << endl;
  // 获取前后两帧变换矩阵
  Eigen::Matrix4f T = ndt.getFinalTransformation();                                          // 输出的T是当前帧到参考帧的变换
  score = ndt.getFitnessScore();
  if(score >= 1)   
  cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()               // ndt.hasConverged() 成功则返回1
              << "score: " << score << std::endl;                            // 输出得分   得分在0.3以上匹配就不好了
  T_curr = T_ref*T;    
  T_ref = T_curr;
  ndt.setInputTarget(curr_ScanFrame);               // 当前点云设置为参考点云
  // 提取出R和t
  geometry_msgs::Pose car_pose;
  Eigen::Matrix3f R = T_curr.block<3,3>(0,0);       // 提取T中 从(0,0)开始的3*3矩阵   !!!!!!!!!注意这里Matrix3f的类型一定要与T的类型匹配  T是float，如果这里是Matrix3d, 则会出现大版的错误
  Eigen::Vector3f t = T_curr.block<3,1>(0,3);    

  show_cloud(curr_ScanFrame,R,t);

  #if DEBUG_ENABLE==1  
  cout<<"T: "<<T_curr<<endl;
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
  pos_pub.publish(car_pose);
}

// LO的重载
// 输入当前帧，得到当前帧与参考帧间的运动
Eigen::Matrix4f Lidar_odometry::LO(pcl::PointCloud<pcl::PointXYZ>::Ptr curr_ScanFrame)
{ 
  ndt.setInputSource(curr_ScanFrame);                                                        // 如果不是第一帧  则设置为输入点云
  const clock_t begin_time = clock();
  //  Eigen::Matrix4f predict_pose = (predict_translation * predict_rotation).matrix();
  Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();                                // 两两帧匹配的初值设置为单位阵   这里只是粗略的估计  后面会改进
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);     // 输入点云经过配准后的结果
  ndt.align(*result_cloud, predict_pose);                                                    // 进行配准  将filtered_cloud对齐后输出为result_cloud
  // 测试时间
  float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
 // cout << "time："<<seconds << endl;
  // 获取前后两帧变换矩阵
  Eigen::Matrix4f T = ndt.getFinalTransformation();                                          // 输出的T是当前帧到参考帧的变换
 // cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()               // ndt.hasConverged() 成功则返回1
 //             << "score: " << ndt.getFitnessScore() << std::endl;                            // 输出得分   得分在0.3以上匹配就不好了
  ndt.setInputTarget(curr_ScanFrame);                                                        // 当前帧设置为被匹配帧
  return T;
}


void Lidar_odometry::LO_()
{ 
  double score;
  if(scan_new == false)  return;    // 如果没有新激光数据
  scan_new = false;                 
  if(LO_INIT == false)              // 如果LO还没初始化
  {
    LO_initialize();                // 初始化
    return;     
  }
  ndt.setInputSource(curr_ScanFrame);                        // 如果不是第一帧  则设置为输入点云
  const clock_t begin_time = clock();                             
//  Eigen::Matrix4f predict_pose = (predict_translation * predict_rotation).matrix();
  Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();                                // 两两帧匹配的初值设置为单位阵
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);     // 输入点云经过配准后的结果
  ndt.align(*result_cloud, predict_pose);   // 进行配准  将filtered_cloud对齐后输出为result_cloud
  // 测试时间
  float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC;
  cout << "Lidar odometry time："<<seconds << endl;
  // 获取前后两帧变换矩阵
  Eigen::Matrix4f T = ndt.getFinalTransformation();                                          // 输出的T是当前帧到参考帧的变换
  score = ndt.getFitnessScore();
 // if(score >= 2) return;
  cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()               // ndt.hasConverged() 成功则返回1
              << "score: " << score << std::endl;                            // 输出得分   得分在0.3以上匹配就不好了
  T_relative = T_relative*T;                               // 获得里程计相对运动
//  cout<<"relative pos: "<<T_relative<<endl;
  // 判断视觉里程计的运动是否到达阈值   到达阈值则进行NDT校正
  Eigen::Matrix3f R = T_relative.block<3,3>(0,0);          // 提取T中 从(0,0)开始的3*3矩阵   !!!!!!!!!注意这里Matrix3f的类型一定要与T的类型匹配  T是float，如果这里是Matrix3d, 则会出现大版的错误
  Eigen::Vector3f t = T_relative.block<3,1>(0,3);   
  Eigen::AngleAxisf relative_rotation;
  relative_rotation.fromRotationMatrix(R);                 // 将旋转矩阵转换成旋转向量
  // 注意min是std命名空间的函数 ！！！      相对旋转大于40度  位移大一1 则退出里程计进行地图匹配校正
  if(std::min((double)fabs(relative_rotation.angle()),2*M_PI-fabs(relative_rotation.angle()))>=0.5||t.norm()>=1) {
  // 需要进行地图校正  将里程计计算出来的相对位姿发送给地图匹配节点
  geometry_msgs::Pose LO_pd;    // LO的预测位姿
  // 将Eigen类型的变换矩阵  转换为geometry_msgs::Pose并发布出去  话题NDT_location 
  LO_pd.position.x = t(0);
  LO_pd.position.y = t(1);
  LO_pd.position.z = t(2);
  Eigen::Quaternionf q(R);     // 注意  四元数有两种类型 Quaternionf和Quaterniond  后面的f和d表示float和double那么用于构造四元数的对象也必须要是同类型的
  LO_pd.orientation.x = q.x();
  LO_pd.orientation.y = q.y();
  LO_pd.orientation.z = q.z();
  LO_pd.orientation.w = q.w();
  LO_predict.publish(LO_pd);
  T_relative = Eigen::Matrix4f::Identity();      // 相对位移清0
}     
  ndt.setInputTarget(curr_ScanFrame);            // 当前点云设置为参考点云
                                              
}

void Lidar_odometry::show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << t(0), t(1), t(2);      // 平移  
    transform.rotate (R);                             // 旋转
    //transform.translation() << 0.0, 0.0, 0.0;       // 平移  
    //transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));       // 旋转
    pcl::transformPointCloud(*scan_cloud, *map_cloud, transform); 
    // 转换为ROS数据
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*map_cloud, output);      
    output.header.stamp=ros::Time::now(); 
    output.header.frame_id = "map";
    cloud_pub.publish(output);
}





