
#include "Lidar_Odometry.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "Lidar_odometry");
    ROS_INFO("Started Lidar_odometry node");               
    Lidar_odometry lo;   
 //   lo.LO_test_set();
    // 循环频率  100HZ  让延迟小一点
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        lo.LO_();   
   //     lo.LO();
        ros::spinOnce();                 // 信息回调处理   这个函数不会阻塞   每次调用   会一次性将消息队列中全部数据通过多次调用回调函数处理完
        loop_rate.sleep();
    }
    
    return 0;
}



