#pragma once 
#include "math3d.hpp"
#include "w3_types.hpp"
using namespace std;

namespace rmv
{
    //第j块板中心的世界系位置
    inline Vec3 plateWorldPosition(const EkfState& s,int j,const PlateGeometry& g)
    {
        const double a = s.theta+deg2rad(g.phase_deg[j]);
        const double rj = s.r+g.radius_offset[j]+(j%2?s.dr:0.0);
        return {s.cx-rj*cos(a),s.cy-rj*sin(a),s.za+g.dh[j]+(j%2?s.dh:0.0)};
    }
    //第j块板的方位角量测=车体theta+相位
    inline double plateThetaMeas(const EkfState& s,int j,const PlateGeometry& g)
    {
        return wrapAngle(s.theta+deg2rad(g.phase_deg[j]));

    }
    //由内法线求theta
    //板的法线可能有倾斜，求水平投影的方位角即可
    inline double thetaFromInwardNormal(const Vec3& n_in_world)
    {
        return atan2(n_in_world.y,n_in_world.x);
    }
    // Columns: X(right), Y(down), Z(inward). Shared by pose solving and rendering.
    inline Mat3 plateRotationWorld(double yaw,double pitch)
    {
        const Vec3 z(cos(pitch)*cos(yaw),cos(pitch)*sin(yaw),sin(pitch));
        const Vec3 x(sin(yaw),-cos(yaw),0),y=z.cross(x);
        Mat3 out;
        for(int k=0;k<3;++k) {
            const Vec3 v=k==0?x:(k==1?y:z);
            out.m[0][k]=v.x;out.m[1][k]=v.y;out.m[2][k]=v.z;
        }
        return out;
    }
}   //namespace rmv
