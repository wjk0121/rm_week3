#pragma once
#include "w3_types.hpp"
#include "world_frame.hpp"
#include<opencv2/core.hpp>
#include<array>
#include<vector>
using namespace std;

namespace rmv
{
    //板坐标系规定与第二周一致
    array<cv::Point3d,4> plateCornersWorld(const EkfState& s,int j,const PlateGeometry& g,double width_m,double height_m);

    //把四块板划到画面上
    void drawPlates(cv::Mat& img,const WorldFrame& wf,double t,const EkfState& s,const PlateGeometry& g,const array<bool,4>& is_big,double small_w_m,double small_h_m,double big_w_m,double big_h_m,int highlight = -1);

    //右上角状态叠加
    void drawStateText(cv::Mat& img,const EkfState& s,int frame_id,int matched_j,int n_meas);

}   //namespace rmv
