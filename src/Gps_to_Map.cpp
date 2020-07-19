
#include <iostream>
#include <fstream>              // 定义读写已命名文件的类型
#include <vector>
#include <sstream>              // 多定义的类型则用于读写存储在内存中的string对象
#include <ros/ros.h>
#include <iomanip>
#include "calib.h"
using namespace std;
/*********************************************获取GPS坐标到地图坐标的变换矩阵*********************************************/
#define ENABLE_DEBUG 0

// 经度处理
double lon_process(string gps_lon)
{
  double p_lon;
  string p_lon_s;
  p_lon_s = gps_lon.substr(6,5);     //提取显著变化的部分
  p_lon = atof(p_lon_s.c_str());     // 转为double型
  return p_lon;
}
double lat_process(string gps_lat)
{
  double p_lat;
  string p_lat_s;
  p_lat_s = gps_lat.substr(5,4);     //提取显著变化的部分
  p_lat = atof(p_lat_s.c_str());     // 转为double型
  return p_lat;
}

// 数据预处理
void data_process(vector<string> dataArray, Eigen::Vector3d &gps_data, Eigen::Vector3d &map_data)
{
 // gps_data << lon_process(dataArray[1]),lat_process(dataArray[2]),1;
  gps_data << atof(dataArray[1].c_str()) - 746000, atof(dataArray[2].c_str()) - 2549000, 1;
  map_data << atof(dataArray[4].c_str()),atof(dataArray[5].c_str()),1;
}

// 离线求解 
// 将数据集一部分用于最小二乘计算   一部分用于验证
// calib: 校正对象    Convert_matrix：校正输出矩阵  
void OffLineCalibration(calibration &calib, Eigen::Matrix3d &Convert_matrix)
{
 int index=1;
 bool solve_ok=false;                                          // 校正矩阵是否生成
 //读文件  
 ifstream inFile("/home/lwh/code/localization_ws/src/localization/UTM/GPS_UTM.csv",ios::in);
 string lineStr;
 // 遍历文件的每一行数据
 while(getline(inFile,lineStr))
 {
    istringstream Readstr(lineStr);     // 将整行字符串lineStr读入到字符串流istringstream中
    string str;                         // 存储每一行的某一列字符串
    vector<string> lineArray;
    // 将一行的数据分开
    //按照逗号分隔
    while(getline(Readstr,str,','))     
      lineArray.push_back(str);          //一行数据以vector保存
    // 第一行不要
    if(index!=1){
    Eigen::Vector3d gps_data;
    Eigen::Vector3d map_data;
 //   提取我们要的数据
 //   gps_data << atof(lineArray[1].c_str()),atof(lineArray[2].c_str()),1;
 //   map_data << atof(lineArray[4].c_str()),atof(lineArray[5].c_str()),1;
    data_process(lineArray, gps_data, map_data);
    #if ENABLE_DEBUG
    cout<<setiosflags(ios::fixed);    // 保证控制的是小数点后面的位数
    cout<<setprecision(7);
    cout<<"GPS: "<<endl<<gps_data<<endl<<"map: "<<endl<<map_data<<endl<<"index: "<<index<<endl;
    #endif
    // 如果校正为完成
        if(solve_ok==false){
            calib.add_data(gps_data,map_data);                            // 添加数据
            if(calib.ok==1)                                               // 数据已满
            {
            Convert_matrix = calib.Solve(); 
         //   Convert_matrix = calib.Solve_by_Eigen();
            solve_ok = true;     // 校正完成标志位
            cout<<"Solve ok!! begin validate..."<<endl;
            }
        }
        else
        {
        Eigen::Vector3d map_predict = Convert_matrix*gps_data;      // 计算预测结果
        cout<<"Predict GPS: "<<gps_data(0)<<", "<<gps_data(1)<<", "<<gps_data(2)<<endl<<" predict map pos： "<<\
                                        map_predict(0)<<", "<<map_predict(1)<<", "<<map_predict(2)<<endl<<" Truth-value: "<<\
                                        map_data(0)<<", "<<map_data(1)<<", "<<map_data(2)<<"  Index: "<<index<<endl;
        }
    }
    index++;
//    if(index>400)  return;
 }
}


int main(int argc, char **argv)
{
 ros::init (argc, argv, "Gps_to_Map");
 ROS_INFO("Started GPS Map calibration node");   
 calibration cal(4000);            // 校正对象   数据量4000  
 Eigen::Matrix3d Convert_matrix;
 OffLineCalibration(cal, Convert_matrix);
 return 0;
}



















