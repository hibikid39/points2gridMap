#ifndef MAKE_MAP_H_
#define MAKE_MAP_H_

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

using namespace std;

// SLAMフロントエンド。ロボット位置推定、地図生成、ループ閉じ込みを取り仕切る。
class MakeMap {
private:
  vector<geometry_msgs::Point> lps;       // スキャン点群
  geometry_msgs::Pose2D pose;             // ロボット位置

  double csize, rsize;
  int tsize, w;

public:
  nav_msgs::OccupancyGrid og;             // 占有格子地図

  MakeMap() : csize(0.1), rsize(150) {
    tsize = int(rsize/csize);           // テーブルサイズの半分
    w = int(2*tsize + 1);        // テーブルサイズ

    // og領域確保
    og.data.resize(w*w);
    for(int i = 0; i < w*w; i++){
      og.data[i] = -1; // -1 is unknown
    }
    og.info.resolution = csize;
    og.info.width = static_cast<int>(2*tsize + 1);
    og.info.height = static_cast<int>(2*tsize + 1);
    og.info.origin.position.x = -1.0*rsize;
    og.info.origin.position.y = -1.0*rsize;
    og.info.origin.position.z = 0;
    og.info.origin.orientation = rpy_to_geometry_quat(0, 0, 0);

    og.header.stamp = ros::Time::now();
    og.header.seq = 1;
    og.header.frame_id = "map";
  }
  ~MakeMap() {}

  void alloc_lps(int num) {
    lps.reserve(num);
  }

  void add_lp(const geometry_msgs::Point lp) {
    lps.push_back(lp);
  }
  void set_pose(const geometry_msgs::Pose2D pose_) {
    pose = pose_;
  }

  // RPYからQuaternionに変換
  geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
  }

  void idx2geom(geometry_msgs::Pose2D &pose, const int idx_x, const int idx_y){
    pose.x = (idx_x - tsize) * csize;
    pose.y = (idx_y - tsize) * csize;
  }

  void geom2idx(const geometry_msgs::Pose2D pose, int &idx_x, int &idx_y){
    idx_x = pose.x / csize + tsize;
    idx_y = pose.y / csize + tsize;
  }

  int og_idx(int idx_x, int idx_y) {
    return static_cast<size_t>(idx_y*(2*tsize +1) + idx_x);
  }

  void make_map();

};

#endif
