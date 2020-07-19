#include <iostream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>


using namespace std;
#define DEBUG_ENABLE 1 

// 基于EKF的传感器融合
class msf_ekf
{
  public:
    msf_ekf()
    {   
        // 创建订阅者  订阅话题NDT_location   消息buff 1  回调函数 Lidar_callback  必须要有个this
        Lidar = nh.subscribe<geometry_msgs::Pose>("NDT_location", 1, &msf_ekf::Lidar_callback, this);   
        pub1 = nh.advertise<visualization_msgs::Marker>("robot_trajectory",100);
        localization_trans.header.frame_id = "map";         //
        localization_trans.child_frame_id = "base_link";    //base_link

        // 设置显示轨迹的参数
        mark1.action = mark1.ADD;
        mark1.header.stamp = ros::Time::now();
        mark1.header.frame_id = "map";
        mark1.lifetime = ros::Duration();
        mark1.ns = "markers";
        mark1.id = 0;
        mark1.type = mark1.LINE_STRIP;
        mark1.pose.orientation.w = 1.0;
        mark1.scale.x = 0.2;
        mark1.scale.y = 0.2;
        mark1.color.r = 210/255.0;    
        mark1.color.g = 0/255.0;
        mark1.color.b = 0/255.0;
        mark1.color.a = 1.0;

    }

    void Lidar_callback(geometry_msgs::Pose car_pose);
    void msf_solve();
    void show_trajectory(const geometry_msgs::Point P);         // 显示轨迹

  private:
    visualization_msgs::Marker mark1;     // 用于rviz轨迹显示
    ros::NodeHandle nh;   
    ros::Publisher fusion_out;            // 创建一个NDT结果的发布者
    ros::Publisher pub1;
    ros::Subscriber GNSS_INS;             // 创建一个订阅者 获取组合导航信息
    ros::Subscriber Lidar;                // 创建一个订阅者 获取激光的定位
    geometry_msgs::TransformStamped localization_trans;           // TF  msg
    tf::TransformBroadcaster odom_broadcaster;            // 一个publisher的封装  通过SendTransform()发送TF
};

void msf_ekf::Lidar_callback(geometry_msgs::Pose car_pose)
{
  #if DEBUG_ENABLE
  // 将geometry_msgs::Pose信息转换为tf类型的R和t
  tf::Matrix3x3 R_retrieve;                         // tf的旋转矩阵
  tf::Quaternion q;                                 // tf的四元数
  tf::quaternionMsgToTF(car_pose.orientation, q);   // 将geometry_msgs::Quaternion转换为tf::Quaternion
  R_retrieve.setRotation(q);                        // 由四元数恢复旋转矩阵
  tf::Point p;                                      // tf的位置
  tf::pointMsgToTF(car_pose.position, p);           // 将geometry_msgs::Point数据转换为tf::Point
  show_trajectory(car_pose.position);               // 显示轨迹
//  cout<<"LIDAR's R: "<<endl<<R_retrieve[0][0]<<","<<R_retrieve[0][1]<<","<<R_retrieve[0][2]<<endl<<R_retrieve[1][0]<<","<<R_retrieve[1][1]<<","<<R_retrieve[1][2]<<endl<<R_retrieve[2][0]<<","<<R_retrieve[2][1]<<","<<R_retrieve[2][2]<<endl;
//  cout<<"LIDAR's position :"<<p.x()<<", "<<p.y()<<", "<<p.z()<<endl;
  #endif
  // 将计算得出的定位结果  转换为TF变换发送出去
  ros::Time current_time = ros::Time::now();    
  localization_trans.header.stamp = current_time;     
  // 位置  
  localization_trans.transform.translation.x = p.x();  
  localization_trans.transform.translation.y = p.y();  
  localization_trans.transform.translation.z = p.z();  
  localization_trans.transform.rotation = car_pose.orientation;     // 设定旋转
  odom_broadcaster.sendTransform(localization_trans);  

}

void msf_ekf::show_trajectory(const geometry_msgs::Point P)
{
  geometry_msgs::Point p1;

  p1.x = P.x;
  p1.y = P.y;
  p1.z = P.z;
  if(mark1.points.size() >= 400)
  {    //当这个ｖｅｃｔｅｒ的长度大于４０个，就删除前面的数
      mark1.points.erase(mark1.points.begin());
  }
  // 添加该点
  mark1.points.push_back(p1);      
  mark1.header.stamp = ros::Time::now();
  pub1.publish(mark1);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "msf_fusion");
  ROS_INFO("Started sensor fusion node");   
  msf_ekf msf;     
  ros::spin(); 
    
  return 0;

}