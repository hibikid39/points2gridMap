#include <ros/ros.h>
#include <iostream>  // for debug writing
#include <string>    // useful for reading and writing
#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream
#include <cmath>

#include "make_OccupancyGridMap/MakeMap.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

int main(int argc, char **argv){
  ros::init(argc, argv, "make_OccupancyGridMap");

  // 地図のpublisher
  ros::NodeHandle nh;
  ros::Publisher pub_og, pub_pc;
  pub_og = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
  pub_pc = nh.advertise<sensor_msgs::PointCloud>("pc", 1);

  sensor_msgs::PointCloud pc;
  pc.header.stamp = ros::Time::now();
  pc.header.seq = 1;
  pc.header.frame_id = "map";
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "rgb";
  pc.channels.push_back(channel);

  MakeMap mm;

  // 入力ファイルを開く
  std::ifstream inputfile_scan, inputfile_pose;
  std::string filename_scan, filename_pose;
  filename_scan = "/home/takahashi/catkin_ws/scan_odom2.txt";
  inputfile_scan.open(filename_scan, std::ios::in);
  if(!inputfile_scan) {
    ROS_INFO_STREAM("[ERROR] cannot open " << filename_scan);
    exit(1);
  }
  filename_pose = "/home/takahashi/catkin_ws/poses2.txt";
  inputfile_pose.open(filename_pose, std::ios::in);
  if(!inputfile_pose) {
    ROS_INFO_STREAM("[ERROR] cannot open " << filename_pose);
    exit(1);
  }

  // 変数宣言 定義
  int stamp = 0;
  int data_num, lps_num;
  std::string buf;
  std::string buf_angle;
  std::string buf_range;
  geometry_msgs::Pose2D pose;
  geometry_msgs::Point lp;
  geometry_msgs::Point32 point;
  double angle, range;

  // 全体のデータ点数を取得 pose.txtの1行目
  std::getline(inputfile_pose, buf); // データ数
  data_num = std::stoi(buf);

  ros::Rate loop_rate(2);

  for (int i=0; i<data_num; i++) {
    pc.points.clear();
    pc.channels[0].values.clear();

    // pose
    std::getline(inputfile_pose, buf, ' '); // POSEを空読み
    std::getline(inputfile_pose, buf, ' '); // タイムスタンプ
    std::getline(inputfile_pose, buf, ' '); // x
    pose.x = std::stod(buf);
    std::getline(inputfile_pose, buf, ' '); // y
    pose.y = std::stod(buf);
    std::getline(inputfile_pose, buf); // theta[deg]
    pose.theta = std::stod(buf);
    mm.set_pose(pose);

    //scan
    std::getline(inputfile_scan, buf, ' '); // LASERSCANを空読み
    std::getline(inputfile_scan, buf, ' '); // タイムスタンプ
    std::getline(inputfile_scan, buf, ' '); // スキャン点の数
    lps_num = std::stoi(buf);
    mm.alloc_lps(lps_num);  // 事前にメモリ確保
    for (int j=0; j<lps_num; j++) {
      std::getline(inputfile_scan, buf_angle, ' ');
      std::getline(inputfile_scan, buf_range, ' ');
      angle = std::stod(buf_angle);
      range = std::stod(buf_range);
      double a = DEG2RAD(angle);
      lp.x = range*cos(a);
      lp.y = range*sin(a);
      lp.z = 0;

      // clpを参照スキャンの座標系に変換 ワールド座標系
      double rad = DEG2RAD(pose.theta);
      double x, y;
      x = cos(rad)*lp.x - sin(rad)*lp.y + pose.x;
      y = sin(rad)*lp.x + cos(rad)*lp.y + pose.y;
      point.x = lp.x = x;
      point.y = lp.y = y;
      point.z = 0;
      pc.points.push_back(point);
      pc.channels[0].values.emplace_back((float)0xFFFFFF);

      mm.add_lp(lp);

      if(inputfile_scan.eof()) {
        ROS_INFO("[ERROR] lps_num is fault");
        exit(1);
      }
    }
    // オドメトリは不要だが読む
    std::getline(inputfile_scan, buf, ' '); // x
    std::getline(inputfile_scan, buf, ' '); // y
    std::getline(inputfile_scan, buf);      // theta[deg]

    mm.make_map();

    pub_og.publish(mm.og);   // 地図出力
    pub_pc.publish(pc);

    loop_rate.sleep();
  }

  ROS_INFO("End");
  return 0;
}
