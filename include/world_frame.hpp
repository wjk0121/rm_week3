#pragma once 

//世界系：用imu姿态把相机系量侧转到重力对齐的世界系
#include"imu_series.hpp"
#include"math3d.hpp"
#include"w3_types.hpp"
using namespace std;
namespace rmv
{
    class WorldFrame
    {
        public:
         WorldFrame(const ImuSeries* imu,const W3Config* cfg);

         //相机系->世界系
         Mat3 R_cam_to_world(double t_img)const;
         Mat3 R_world_to_cam(double t_img)const;

         Vec3 cam_to_world(double t_img, const Vec3& p_cam)const;
         Vec3 world_to_cam(double t_img, const Vec3& p_w)const;

         //世界点投影到像素
         Vec2 project(double t_img,const Vec3& p_w)const;

         //像素  归一化相机坐标
         Vec2 undistortToNormalized(const Vec2& uv)const;

         const W3Config& config() const {return *cfg_;}
        private:
         const ImuSeries* imu_=nullptr;
         const W3Config* cfg_=nullptr;
         rmv::Mat3 R_ci_;

         
        
    };

    
}        //namespace rmv
