#ifndef CALIB_H
#define CALIB_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Jacobi>

// 校正类
class calibration
{
   public:
     int data_num;            // 计算的数据量 
     static int now_len;      // 当前输入的数据量
     bool ok;                 // 数据准备好为1
     
     calibration(){}
     calibration(int num)
     {  
       // 设置计算的数据量
       data_num = num;
       ok = 0; 
       // 设置矩阵的size    
       A_GPS.conservativeResize(data_num*3,9);   // 3*data_num行  9列
       b_map.conservativeResize(data_num*3);
       // 数据清0
       A_GPS.setZero();
       b_map.setZero();
     }
     void add_data(Eigen::Vector3d gps_pos,Eigen::Vector3d map_pos) ;    // 添加数据
     Eigen::Matrix3d Solve();  // 开始计算
     Eigen::Matrix3d Solve_by_Eigen();  // 开始计算
    
  private:
     Eigen::MatrixXd A_GPS;     // 系数矩阵A  GPS数据
     Eigen::VectorXd b_map;     

};


#endif