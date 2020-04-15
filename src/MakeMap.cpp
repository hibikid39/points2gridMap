#include "make_OccupancyGridMap/MakeMap.h"
#define DIS 30/csize // LRFの計測可能領域
/*
double MakeMap::inverse_sensor_model(int x, int y, int mx, int my, geometry_msgs::Point lp) {
  geometry_msgs::Pose2D pose, m;
  idx2geom(pose, x, y);
  idx2geom(m, mx, my);

  double r = std::sqrt((pose.x - m.x)*(pose.x - m.x) + (pose.y - m.y)*(pose.y - m.y));

}
*/

void MakeMap::make_map() {
  ROS_INFO("pose = (%f, %f, %f)", pose.x, pose.y, pose.theta);
  ROS_INFO("lps.size = %d", int(lps.size()));
/*
  for(int i = 0; i < w*w; i++){
    og.data[i] = -1; // -1 is unknown
  }
*/
/*
  int posx, posy;
  geom2idx(pose, pos_idx_x, pos_idx_y);

  for(int k=0; k<lps.size(); k++) {
    for(int i=pos_idx_x-DIS; i<pos_idx_x+DIS; i++) {
      for(int j=pos_idx_y-DIS; j<pos_idx_y+DIS; j++) {
        if(((i-pos_idx_x)*(i-pos_idx_x)+(j-pos_idx_y)*(j-pos_idx_y)) < DIS*DIS){  // センサの範囲内：円
          int idx = og_idx(i, j);
          og.data[idx] += inverse_sensor_model(pos_idx_x, pos_idx_y, i, j, lps[i]);
        }
      }
    }
  }
*/


  for(int i=0; i<lps.size(); i++) {
    int xi, yi, pxi, pyi;
    xi = static_cast<int>(lps[i].x/csize) + tsize; // 計測点のインデックス
    yi = static_cast<int>(lps[i].y/csize) + tsize;
    pxi = static_cast<int>(pose.x/csize) + tsize; // 自己位置のインデックス
    pyi = static_cast<int>(pose.y/csize) + tsize;

    double slope;       // 直線の傾き
    int x_start, x_end, y_start, y_end; // 直線の始点終点

    if(std::abs((lps[i].y-pose.y) / (lps[i].x-pose.x)) < 1) { //傾きが45度未満
      if (xi > pxi) {                                 // 自己位置が左側
        slope = (lps[i].y-pose.y) / (lps[i].x-pose.x);
        x_start = pxi; x_end = xi-1; y_start = pyi;
      } else if (xi < pxi){                           // 自己位置が右側
        slope = (pose.y-lps[i].y) / (pose.x-lps[i].x);
        x_start = xi+1; x_end = pxi; y_start = yi;
      }
      // 占有していない格子
      for(int j = x_start; j <= x_end; j++){
        // y = dy/dx * ⊿x + y_s
        int idx_y = static_cast<int>(slope * (j-x_start) + y_start);
        size_t idx = static_cast<size_t>(idx_y * (2 * tsize + 1) + j);
        if (og.data[idx] == -1 || og.data[idx] == 0) og.data[idx] = 0;
        else og.data[idx] -= 10; // 投票
      }
      // 占有している格子
      size_t idx = static_cast<size_t>(yi * (2 * tsize + 1) + xi);
      if (og.data[idx] == -1 || og.data[idx] == 100) og.data[idx] = 100;
      else og.data[idx] += 10; // 投票

    }else{  // 傾きが45度以上

      if (yi > pyi) {                                 // 自己位置が左側
        slope = (lps[i].x-pose.x) / (lps[i].y-pose.y);
        y_start = pyi; y_end = yi-1; x_start = pxi;
      } else if (yi < pyi){                           // 自己位置が右側
        slope = (pose.x-lps[i].x) / (pose.y-lps[i].y);
        y_start = yi+1; y_end = pyi; x_start = xi;
      }
      // 占有していない格子
      for(int j = y_start; j <= y_end; j++){
        // x = dx/dy * ⊿y + x_s
        int idx_x = static_cast<int>(slope * (j-y_start) + x_start);
        size_t idx = static_cast<size_t>(j * (2 * tsize + 1) + idx_x);
        if (og.data[idx] == -1 || og.data[idx] == 0) og.data[idx] = 0;
        else og.data[idx] -= 10; // 投票
      }
      // 占有している格子
      size_t idx = static_cast<size_t>(yi * (2 * tsize + 1) + xi);
      if (og.data[idx] == -1 || og.data[idx] == 100) og.data[idx] = 100;
      else og.data[idx] += 10; // 投票
    }
  }

  lps.clear();
}
