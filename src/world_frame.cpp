#include"world_frame.hpp"
using namespace std;

namespace rmv
{
    WorldFrame::WorldFrame(const ImuSeries* imu,const W3Config* cfg):imu_(imu),cfg_(cfg)
    {
        R_ci_=rmv::Mat3::fromArray(cfg_->R_ci);

    }
    Mat3 WorldFrame::R_cam_to_world(double t_img)const
    {
        //世界系方向=R_imu->world(t)*(相机系方向->imu方向)
        return imu_->rotationAt(t_img,cfg_->img_imu_offset)*R_ci_;

    }
    Mat3 WorldFrame::R_world_to_cam(double t_img)const
    {
        //旋转矩阵正交
        return R_cam_to_world(t_img).transposed();

    }
    Vec3 WorldFrame::cam_to_world(double t_img,const Vec3& p_cam)const
    {
        const Vec3 t_ic(cfg_->t_ic[0],cfg_->t_ic[1],cfg_->t_ic[2]);
        const Mat3 R_iw = imu_->rotationAt(t_img,cfg_->img_imu_offset);
        return R_iw*(R_ci_*p_cam+t_ic);
    }
    Vec3 WorldFrame::world_to_cam(double t_img,const Vec3& p_w)const
    {
        const Vec3 t_ic(cfg_->t_ic[0],cfg_->t_ic[1],cfg_->t_ic[2]);
        const Mat3 R_iw = imu_->rotationAt(t_img,cfg_->img_imu_offset);
        return R_ci_.transposed()*(R_iw.transposed()*p_w-t_ic);
    }
    Vec2 WorldFrame::project(double t_img,const Vec3& p_w) const
    {
        const Vec3 p = world_to_cam(t_img,p_w);
        if(p.z<=1e-6)
        return {-1e9,-1e9};
        const double xn = p.x/p.z;
        const double yn = p.y/p.z;

        const double k1 = cfg_->D[0],k2 = cfg_->D[1],p1 = cfg_->D[2],p2 = cfg_->D[3],k3 = cfg_->D[4];
        const double r2 = xn*xn+yn*yn;
        const double radial = 1.0+r2*(k1+r2*(k2+r2*k3));
        const double xd = xn*radial+2.0*p1*xn*yn+p2*(r2+2.0*xn*xn);
        const double yd = yn*radial+p1*(r2+2.0*yn*yn)+2.0*p2*xn*yn;

        const double fx = cfg_->K[0],cx = cfg_->K[2];
        const double fy = cfg_->K[4],cy = cfg_->K[5];
        return {fx*xd+cx,fy*yd+cy};

    }
    Vec2 WorldFrame::undistortToNormalized(const Vec2& uv) const 
    {
        const double fx = cfg_->K[0],cx = cfg_->K[2];
        const double fy = cfg_->K[4],cy = cfg_->K[5];
        const double xd = (uv.x-cx)/fx;
        const double yd = (uv.y-cy)/fy;

        const double k1 = cfg_->D[0],k2 = cfg_->D[1],p1 = cfg_->D[2],p2 = cfg_->D[3],k3 = cfg_->D[4];

        //畸变模型没有解析解
        double xn = xd,yn = yd;
        for(int i = 0;i<5;++i)
        {
            const double r2 = xn*xn+yn*yn;
            const double radial = 1.0+r2*(k1+r2*(k2+r2*k3));
            const double dx = 2.0*p1*xn*yn+p2*(r2+2.0*xn*xn);
            const double dy = p1*(r2+2.0*yn*yn)+2.0*p2*xn*yn;
            xn = (xd-dx)/radial;
            yn = (yd-dy)/radial;
        }
        return {xn,yn};
    }
}        //namespce rmv
