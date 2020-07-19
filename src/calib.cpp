
#include "calib.h"
#include <iomanip>
using namespace std; 
/**
* 通过最小二乘计算GPS坐标到MAP坐标
*
*
*/
int calibration::now_len = 0;                         //  当前读取的数据量  初始化为0

// 添加数据   AX=b
void calibration::add_data(Eigen::Vector3d gps_pos,Eigen::Vector3d map_pos)
{  
 //  if(gps_pos(0)<10000||gps_pos(1)<1000)   return;   
   if(now_len>=data_num)                // 数据未满
   {
    ok = 1;                             // 数据完成标志位置一
    cout<<"data is get ready! "<<endl;
    return;
   }
   A_GPS.row(3*now_len) << gps_pos(0),gps_pos(1),gps_pos(2),0,0,0,0,0,0;
   A_GPS.row(3*now_len+1) << 0,0,0,gps_pos(0),gps_pos(1),gps_pos(2),0,0,0;
   A_GPS.row(3*now_len+2) << 0,0,0,0,0,0,gps_pos(0),gps_pos(1),gps_pos(2);
   b_map(3*now_len) = map_pos(0);
   b_map(3*now_len+1) = map_pos(1);
   b_map(3*now_len+2) = map_pos(2);
   now_len++;
   cout<<"data num : "<<now_len<<endl;
}

// 解析法求解最小二乘  X = (A.T*A)^-1*A.T*b
// 返回变换矩阵 Eigen::Matrix3d
Eigen::Matrix3d calibration::Solve()
{
   Eigen::Matrix3d Convert_M;
   Eigen::MatrixXd AAT;
   Eigen::MatrixXd result;

   if(ok == 1)    // 如果数据准备完成
   {
     cout<<setiosflags(ios::fixed);    // 保证控制的是小数点后面的位数
     cout<<setprecision(7);
     cout<<"begin calculate! wait..."<<endl;
     // 检查矩阵
     cout<<"A_GPS: "<<endl<<A_GPS<<endl;
     cout<<"b_map: "<<endl<<b_map<<endl;
     // 计算(A^T)*A
     AAT = A_GPS.transpose()*A_GPS;
     cout<<"AAT: "<<endl<<AAT<<endl;
     // 计算状态   9x1向量
     result = AAT.inverse()*(A_GPS.transpose())*b_map ;   
     cout<<"result: "<<endl<<result<<endl;
     result.resize(3, 3);
     Convert_M = result.transpose();
     cout<<"gps to Map : "<<endl<<Convert_M<<endl;
   }
   return Convert_M;
}

// 线性方程最小二乘解
Eigen::Matrix3d calibration::Solve_by_Eigen()
{ 
   Eigen::Matrix3d Convert_M;
   if(ok == 1)                                         // 如果数据准备完成
   {
    cout<<"begin calculate! wait..."<<endl;
    cout<<"b_map: "<<endl<<b_map<<endl;
    Eigen::MatrixXd result = A_GPS.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_map);    // 通过SVD求解
    result.resize(3,3);
    // 要转置一下
    Convert_M = result.transpose();
    cout<<"gps to Map : "<<endl<<Convert_M<<endl;
   }
   return Convert_M;
}






