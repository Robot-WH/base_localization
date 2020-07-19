#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ctime>
#include "Lidar_Odometry.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "calib.h"

//#include <eigen3/Eigen/Geometry>

using namespace std;

#define VO 1
#define map_match 0

// 点云数据处理类
class Point_process
{
   public:
       // 删除NaN点 
      void EraseInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
       //去除离群点
      void OutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out);

};
//遍历并删除无效点
void Point_process::EraseInvalidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->points.begin();  //迭代找点
	int cnt = 0;
	while (it != cloud->points.end())
	{
		float x, y, z;
		x = it->x;
		y = it->y;
		z = it->z;		
		//cout << "x: " << x << "  y: " << y << "  z: " << z << "  rgb: " << rgb << endl;
		if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
		{
			it = cloud->points.erase(it); //删除了迭代点的之后他会返回下一个点
			cnt++;
		}
		else
			++it;
	}
	std::cout << "erase points: " << cnt << std::endl;
}
//去除离群点
void Point_process::OutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out)
{
	std::cout << "begin cloud_in size: " << cloud_in->size() << std::endl;
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
	pcFilter.setInputCloud(cloud_in);             //设置待滤波的点云
	pcFilter.setRadiusSearch(0.03);               // 设置搜索半径
	pcFilter.setMinNeighborsInRadius(3);      // 设置一个内点最少的邻居数目
	pcFilter.filter(*cloud_out);        //滤波结果存储到cloud_filtered

	std::cout << "success OutlierFilter, size: " << cloud_out ->size() <<std::endl;
}


class Lidar_localization
{
public:  
  Lidar_localization()  
  {  
    pos_pub = nh.advertise<geometry_msgs::Pose> ("NDT_location", 1);                                  // 初始化发布器      
   // laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("laser_scan", 1, &NDT::scan_cb, this);     // 初始化订阅器   这个写法比较特殊必须要这样写
    laser_scan = nh.subscribe<sensor_msgs::PointCloud2> ("/rslidar_points", 1, &Lidar_localization::scan_cb, this);      
    /***************************************设置在地图上的初始坐标*********************************************/
 //   Eigen::AngleAxisf predict_rotation(M_PI/2, Eigen::Vector3f::UnitX()) ;              // 旋转的预测值 
 //   Eigen::Quaternionf q (0.99930137466394, -0.0302042760550403, -0.00750171424198088, 0.0206927181922387);  
 //   Eigen::AngleAxisf predict_rotation(q) ; 
 //   predict_rotation = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,0,1));                // 默认坐标
    predict_rotation = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(0,0,1));     
    predict_translation = Eigen::Translation3f(0,0,0);
    PreCurrPos_T = Eigen::Matrix4f::Identity();                                           // 初始坐标为0
    PrePreCurrPos_T = Eigen::Matrix4f::Identity(); 
    curr_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }  
  // c++ 11 可以直接类内初始化了
  ros::Time previous_time= ros::Time(0);
  bool frist_scan = true;  
  bool scan_new = false ;     // 新接收到雷达数据？                                       
  bool Liloc_init = false;    // 定位初始化  
  bool Pos_init = false;      // 位姿初始化
  bool LoPredict_OK = false;   
  bool run_mode = 0;          // 匹配模式 
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_scan ;                                 // 激光当前帧
  Lidar_odometry lo;                                                              // 激光里程计对象
  Eigen::Matrix4f currPos_T;                                                      // Lidar给出的在地图上的定位位姿
  Eigen::Matrix4f PreCurrPos_T;                                                   // 上一次校正得到的位姿
  Eigen::Matrix4f PrePreCurrPos_T; 
  float ndt_score = 0;
  Point_process* Pp = new Point_process();   

  // 位姿
  struct Pose
  {
    double x=0;
    double y=0;
    double z=0;
    double pitch=0;
    double roll=0;
    double yaw=0;
  };

  // 状态
  struct state
  {
    struct Pose curr_pose;     // 当前pose
    // 速度   z,pitch,row变化不大 仅仅采用匀速运动模型
    double curr_velocity_x=0;
    double curr_velocity_y=0;
    double curr_velocity_z=0;
    double curr_velocity_yaw=0;
    double curr_velocity_pitch=0;
    double curr_velocity_roll=0;
    // 加速度   x和y和航向角yaw变化较多 因此用匀加速模型预测
    double curr_accel_x=0;
    double curr_accel_y=0;
    double curr_accel_yaw=0;    
  };
          
  state curr_Lidar_state, pre_Lidar_state;   // 雷达的运动状态 
  // 预测的初值
  double predict_x=0, predict_y=0, predict_z=0, predict_yaw=0, predict_pitch=0, predict_roll=0;

//  void state_initialize();                   // 状态初始化 
  void Localization_initialize();              // 定位初始化                                     
  void solve_localization_each_scan(ros::Time time);        // 定位函数2
  void state_update(double diff_time, Eigen::Matrix4f currPos_T);   // 状态更新
  Eigen::Matrix4f ndt_solve();                 // ndt求解
  //回调函数  接收发送过来的点云msg  用于定位
  void scan_cb (const sensor_msgs::PointCloud2 cloud_msg);
  // 读取用于定位的地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr NDT_readMap();
  // ndt的配置
  void ndt_Config(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
  // 显示输入点云
  void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t );
  // 发送定位位姿
  void send_pos(Eigen::Matrix4f T);
  void predict_pose(double diff_time);

private:  
  ros::NodeHandle nh;   
  ros::Publisher pos_pub;       // 创建一个NDT结果的发布者  
  ros::Subscriber laser_scan;   // 创建一个订阅者  订阅话题laser_scan
  ros::Subscriber LO_predict;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;    // 定义ndt对象
  /**********************增加赋初值*************************************/
  Eigen::AngleAxisf predict_rotation;                 // 旋转的预测值 
  Eigen::Translation3f predict_translation;           // 位置的预测值

};  


//回调函数  接收发送过来的点云msg  获取处理好后的输入点云
void Lidar_localization::scan_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  static int t=0;
  scan_new = true;                                           // 接收标志为置为一
  lo.scan_new = true;                                        // 视觉里程计更新标志位置一
  
  pcl::PCLPointCloud2 pcl_cloud;                             // sensor_msgs::PointCloud2首先转换为PCLPointCloud2----->PointCloud<pcl::PointXYZ>
  pcl_conversions::toPCL(cloud_msg,pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud,*scan_cloud);            // 转换为PCL的类型 
  // 去除无效点
  std::vector<int> indices;               
  pcl::removeNaNFromPointCloud(*scan_cloud, *scan_cloud, indices);

  // 对输入点云进行滤波   使点云的数量减少到10%      便于计算
  // 点云降采样滤波  这里会产生很多小三维体素包围点云，对于一个三维体素内部的点云只用体素的重心处的一个点来替代
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                                // 声明一个对pcl::PointXYZ滤波的滤波器
  approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);                                                                                          // 体素的大小                                                                                 

  approximate_voxel_filter.setInputCloud(scan_cloud);        // 设置滤波的点云
  approximate_voxel_filter.filter(*curr_scan);               // 输出                 
  //  std::cout<<"I receive laser data: "<< curr_scan->size()<<"t: "<<t << std::endl;    
  solve_localization_each_scan(cloud_msg.header.stamp);                           // ndt求解
}

Eigen::Matrix4f Lidar_localization::ndt_solve()
{
  cout<<"-----------NDT solve------------"<<endl;
  //  设置NDT算法的初值   没有初值也能运行  但是一个好的初值能帮助更好的定位  实际中通常用航迹推算方法计算初始位姿
  //  Eigen::AngleAxisf init_rotation(0.6331, Eigen::Vector3f::UnitZ());  // AngleAxisf旋转向量   0.3 ~ 1.03   17度～59度  +-20度能拉回                           // 绕Z轴
  //  Eigen::Translation3f init_translation (2.3, 0.720047, 0);           //  测试x轴 0.3-4.2
  ndt.setInputSource(curr_scan);                                          // 将滤波后的数据设置为输入点云
  const clock_t begin_time = clock() ;
  Eigen::Matrix4f predict_pose = (predict_translation * predict_rotation).matrix();          // 设定预测值
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);     // 输入点云 经过配准后的结果
  ndt.align(*result_cloud, predict_pose);                                                    // 进行配准  将filtered_cloud对齐后输出为result_cloud

  float seconds = float(clock( ) - begin_time) / CLOCKS_PER_SEC ;
  cout << "ndt time："<<seconds << endl ;
  ndt_score = ndt.getFitnessScore();   // 获得得分
  // 获取变换矩阵
  Eigen::Matrix4f T = ndt.getFinalTransformation();                               // 输出的T是map->base_link
  cout << "score: " << ndt_score << std::endl;                            // 输出得分   得分在0.3以上匹配就不好了
  return T;
}

// 参数： target_cloud： 即地图点云
// 最主要的参数是网格边长Resolution   一般来说  边长越长 定位范围越大 初值的容错能力越强  但耗时也越长，边长越小，定位范围越小，越需要更加精确的初值，但耗时越少
// 我们这里由于位姿初始化时需要定位能力比较强，所以这里边长设置的大一些，  初始化完成后，边长缩小，降低耗时
void Lidar_localization::ndt_Config(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
   ndt.setTransformationEpsilon(0.01);  // 定义变换矢量[x, y, z, roll, pitch, yaw]的最小变化量 当变化量小于该值时算法停止
   ndt.setStepSize(0.1);                // 设置优化的最大步长 
   ndt.setResolution(5);                // 设置网格化立体的边长   初始定位的网格边长大一些
   ndt.setMaximumIterations(35);        // 迭代次数
   ndt.setInputTarget(target_cloud);    // 对齐的目标点云
}

// 读取用于定位的地图
pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar_localization::NDT_readMap()
{
    string file;
    //  nh.param<std::string>("file_path", file, "/home/lwh/桌面/NDT_PCL_demo/cloud1.pcd");     // 读取参数文件的file_path
    cout<<"read localized map..."<<endl;
    file = nh.param<string>("Lidar_localization/globalmap_pcd", "-");       // 全局句柄读取地图路径
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
    // 去除NaN点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    // 对定位的地图进行滤波   注意  用于定位的地图不是原来的点云地图  用于定位的地图不需要很密集的点云因为会造成耗时增加
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                                // 声明一个对pcl::PointXYZ滤波的滤波器
    approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);        // 这里设置的滤波体素大小要和激光帧滤波体素大小一致                                                                                  // 体素的大小                                                                                 
    approximate_voxel_filter.setInputCloud(cloud);              // 设置滤波的点云    
    approximate_voxel_filter.filter(*filter_map);               // 输出    
    std::cout<<"map cloud after filter size "<< filter_map->size() << std::endl; 
    return filter_map;
}


// 输入点云转换到Map坐标系中
void Lidar_localization::show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud, Eigen::Matrix3f R, Eigen::Vector3f t)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << t(0), t(1), t(2);              // 平移  
    transform.rotate (R);                                     // 旋转
    //transform.translation() << 0.0, 0.0, 0.0;               // 平移  
    //transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));   // 旋转
    pcl::transformPointCloud(*scan_cloud, *map_cloud, transform); 
    // 转换为ROS数据
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*map_cloud, output);      
    output.header.stamp=ros::Time::now(); 
    output.header.frame_id = "map";
  //  cloud_pub.publish(output);
}

// 地图定位初始化   读取地图   设置NDT参数  
void Lidar_localization::Localization_initialize()
{
   cout<<"Begin initialize...."<<endl;
   auto map = NDT_readMap();                            // 读取地图
   ndt_Config(map);                                     // 设置ndt对象的参数  配准的目标点云为map
   Liloc_init = true;     
   cout<<"Map initialize ok!"<<endl;
}

// 新的激光帧来了时   先预测大概的pose
void Lidar_localization::predict_pose(double diff_time)
{
 // 采用匀加速运动模型预测
   predict_x = pre_Lidar_state.curr_pose.x + pre_Lidar_state.curr_velocity_x*diff_time ;
   predict_y = pre_Lidar_state.curr_pose.y + pre_Lidar_state.curr_velocity_y*diff_time ;
   predict_yaw = pre_Lidar_state.curr_pose.yaw + pre_Lidar_state.curr_velocity_yaw*diff_time ;
   predict_z = pre_Lidar_state.curr_pose.z + pre_Lidar_state.curr_velocity_z*diff_time;
   predict_pitch = pre_Lidar_state.curr_pose.pitch;
   predict_roll = pre_Lidar_state.curr_pose.roll;
  // 赋值给预测量   直接通过欧拉角转换为旋转向量的方法     
  predict_rotation = Eigen::AngleAxisf(predict_yaw, Eigen::Vector3f::UnitZ())
                     *Eigen::AngleAxisf(predict_pitch, Eigen::Vector3f::UnitY())
                     *Eigen::AngleAxisf(predict_roll, Eigen::Vector3f::UnitX());
  predict_translation = Eigen::Translation3f(predict_x, predict_y, predict_z);
  
  cout<<"predict x: "<<predict_x<<"predict y: "<<predict_y
  <<"predict z: "<<predict_z<<endl<<"predict yaw: "<<predict_yaw
  <<"predict pitch: "<<predict_pitch<<"predict roll: "<<predict_roll<<endl;

}

#define predict_threshold 0.03
#define ndt_MaxMatchscore 1.5
#define check_time 30

/**
* @brief 状态的更新  速度-加速度
* @param:current_time 当前时间
* @param:currPos_T NDT刚刚解算出的位姿
*/
void Lidar_localization::state_update(double diff_time, Eigen::Matrix4f currPos_T)
{
   // 更新当前位姿
  curr_Lidar_state.curr_pose.x = currPos_T(0,3);
  curr_Lidar_state.curr_pose.y = currPos_T(1,3);
  curr_Lidar_state.curr_pose.z = currPos_T(2,3);
  // 下面这个函数不好  当角度超过了Pi之后   就会从PI直接到0  
  //  Eigen::Vector3f euler_angles = currPos_T.block<3,3>(0,0).eulerAngles(2,1,0);     // 旋转矩阵转换为欧拉角   ZYX顺序  yaw-pitch-roll   
  Eigen::Vector3d euler_angles(0,0,0);
  tf::Matrix3x3 mat_rotation;            
  mat_rotation.setValue(static_cast<double>(currPos_T(0, 0)), static_cast<double>(currPos_T(0, 1)), static_cast<double>(currPos_T(0, 2)),
                   static_cast<double>(currPos_T(1, 0)), static_cast<double>(currPos_T(1, 1)), static_cast<double>(currPos_T(1, 2)),
                   static_cast<double>(currPos_T(2, 0)), static_cast<double>(currPos_T(2, 1)), static_cast<double>(currPos_T(2, 2)));
  mat_rotation.getRPY(euler_angles(2), euler_angles(1), euler_angles(0), 1);     // 转换成欧拉角  -PI —— PI 

  curr_Lidar_state.curr_pose.yaw = euler_angles(0);
  curr_Lidar_state.curr_pose.pitch = euler_angles(1);
  curr_Lidar_state.curr_pose.roll = euler_angles(2);
                                                                       
  // 如果位置初值没有找到
  static unsigned char t = 0;
  if(!Pos_init){
    // 时间检查   
    t++;
    if(t>=check_time)                                                  // 如果超过20次都没收敛  初始化失败   
    {
      cout<<"POS init error!"<<endl;
      ros::shutdown();     
    }

    double error = sqrt( (currPos_T(0,3) - predict_x)*(currPos_T(0,3) - predict_x)
                        +(currPos_T(1,3) - predict_y)*(currPos_T(1,3) - predict_y) 
                        +(currPos_T(2,3) - predict_z)*(currPos_T(2,3) - predict_z)
    );
    cout<<"predict-ndt error: "<<error<<endl;

    // 判定初始化有没有成功  
    if(ndt_score<=ndt_MaxMatchscore && error<= predict_threshold)  
    {
      ndt.setResolution(1.0);                // 设置网格化立体的边长
      Pos_init = true;      
      cout<<"pos initialized OK! "<<endl;
    }
  }
  else{   
    // 更新速度   
    curr_Lidar_state.curr_velocity_x = (curr_Lidar_state.curr_pose.x - pre_Lidar_state.curr_pose.x) / diff_time;   
    curr_Lidar_state.curr_velocity_y = (curr_Lidar_state.curr_pose.y - pre_Lidar_state.curr_pose.y) / diff_time;
    curr_Lidar_state.curr_velocity_z = (curr_Lidar_state.curr_pose.z - pre_Lidar_state.curr_pose.z) / diff_time;    
    // 更新加速度
    curr_Lidar_state.curr_accel_x = (curr_Lidar_state.curr_velocity_x - pre_Lidar_state.curr_velocity_x) / diff_time;  
    curr_Lidar_state.curr_accel_y = (curr_Lidar_state.curr_velocity_y - pre_Lidar_state.curr_velocity_y) / diff_time;

    // 单独更新角度  因为欧拉角在180度边界会发生突变  由PI->-PI
    // yaw
    if(fabs(curr_Lidar_state.curr_pose.yaw - pre_Lidar_state.curr_pose.yaw)>M_PI)
      curr_Lidar_state.curr_velocity_yaw = (2*M_PI - fabs(curr_Lidar_state.curr_pose.yaw) - fabs(pre_Lidar_state.curr_pose.yaw) ) / diff_time;
    else
      curr_Lidar_state.curr_velocity_yaw = (curr_Lidar_state.curr_pose.yaw - pre_Lidar_state.curr_pose.yaw) / diff_time;
    // pitch
    if(fabs(curr_Lidar_state.curr_pose.pitch - pre_Lidar_state.curr_pose.pitch)>M_PI)
      curr_Lidar_state.curr_velocity_pitch = (2*M_PI - fabs(curr_Lidar_state.curr_pose.pitch) - fabs(pre_Lidar_state.curr_pose.pitch) ) / diff_time;
    else
      curr_Lidar_state.curr_velocity_pitch = (curr_Lidar_state.curr_pose.pitch - pre_Lidar_state.curr_pose.pitch) / diff_time;
    // roll
    if(fabs(curr_Lidar_state.curr_pose.roll - pre_Lidar_state.curr_pose.roll)>M_PI)
      curr_Lidar_state.curr_velocity_roll = (2*M_PI - fabs(curr_Lidar_state.curr_pose.roll) - fabs(pre_Lidar_state.curr_pose.roll) ) / diff_time;
    else
      curr_Lidar_state.curr_velocity_roll = (curr_Lidar_state.curr_pose.roll - pre_Lidar_state.curr_pose.roll) / diff_time;

    // yaw加速度
    curr_Lidar_state.curr_accel_yaw = (curr_Lidar_state.curr_velocity_yaw - pre_Lidar_state.curr_velocity_yaw) / diff_time;  
  }
  
  cout<<"Lidar x: "<<curr_Lidar_state.curr_pose.x<<"Lidar y: "<<curr_Lidar_state.curr_pose.y
  <<"Lidar z: "<<curr_Lidar_state.curr_pose.z<<endl<<"Lidar yaw: "<<curr_Lidar_state.curr_pose.yaw
  <<"Lidar pitch: "<<curr_Lidar_state.curr_pose.pitch<<"Lidar roll: "<<curr_Lidar_state.curr_pose.roll
  <<endl<<"Lidar vx: "<<curr_Lidar_state.curr_velocity_x<<"Lidar vy: "<<curr_Lidar_state.curr_velocity_y
  <<"Lidar vz: "<<curr_Lidar_state.curr_velocity_z<<endl<<"Lidar v yaw: "<<curr_Lidar_state.curr_velocity_yaw
  <<"Lidar v pitch: "<<curr_Lidar_state.curr_velocity_pitch<<"Lidar v roll: "<<curr_Lidar_state.curr_velocity_roll
  <<endl<<"Lidar accel x "<<curr_Lidar_state.curr_accel_x<<"Lidar accel y: "<<curr_Lidar_state.curr_accel_y
  <<"Lidar accel yaw: "<<curr_Lidar_state.curr_accel_yaw<<endl<<endl;
  pre_Lidar_state = curr_Lidar_state;
}

// 对于每一帧激光数据都执行
void Lidar_localization::solve_localization_each_scan(ros::Time time)
{
    double diff_time = (time - previous_time).toSec();     // 计算时间差
    cout<<"scan diff-time: "<<diff_time<<endl;  
    // 如果位姿初始化没有完成  则一直进行位姿初始化  
    if(frist_scan)                           
    {       
        currPos_T = ndt_solve();                       // 对这一帧求解ndt，获取在地图上的精确初始值    
        state_update(diff_time, currPos_T);            // 状态更新
        frist_scan=false;    
    }
    else
    {
       predict_pose(diff_time);                            // 先预测                           
       currPos_T = ndt_solve();                            // 求解NDT矫正
       state_update(diff_time, currPos_T);                 // 状态更新
    }
    send_pos(currPos_T);      
    previous_time = time;
}

// ndt定位的结果发步出去
void Lidar_localization::send_pos(Eigen::Matrix4f T)
{
  // 提取出R和t
  geometry_msgs::Pose car_pose;
  Eigen::Matrix3f R = T.block<3,3>(0,0);       // 提取T中 从(0,0)开始的3*3矩阵   !!!!!!!!!注意这里Matrix3f的类型一定要与T的类型匹配  T是float，如果这里是Matrix3d, 则会出现大版的错误
  Eigen::Vector3f t = T.block<3,1>(0,3);    

  #if DEBUG_ENABLE==1  
  cout<<"T: "<<T<<endl;
  cout<<"R: "<<R<<endl;
  cout<<"t: "<<t<<endl;
  #endif
  // 将Eigen类型的变换矩阵  转换为geometry_msgs::Pose并发布出去  话题NDT_location 
  car_pose.position.x = t(0);
  car_pose.position.y = t(1);
  car_pose.position.z = t(2);
  Eigen::Quaternionf q(R);        // 注意  四元数有两种类型 Quaternionf和Quaterniond  后面的f和d表示float和double那么用于构造四元数的对象也必须要是同类型的
  car_pose.orientation.x = q.x();
  car_pose.orientation.y = q.y();
  car_pose.orientation.z = q.z();
  car_pose.orientation.w = q.w();
  pos_pub.publish(car_pose);
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "Lidar_localization");   
    ROS_INFO("Started Lidar localization node");   
    Lidar_localization loc_lidar;  
    loc_lidar.Localization_initialize();            // 初始化
    // 循环频率
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();           // 信息回调处理   这个函数不会阻塞   每次调用   会一次性将消息队列中全部数据通过多次调用回调函数处理完
        loop_rate.sleep();
    }
    return 0;
}




















