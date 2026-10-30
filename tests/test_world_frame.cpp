#include "imu_series.hpp"
#include "world_frame.hpp"
#include "frame_timeline.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
using namespace rmv;
int main() {
    int errors=0;auto check=[&](bool b,const char* n){if(!b){std::cerr<<"FAIL "<<n<<'\n';++errors;}};
    const std::string file="dynamic_test_imu.txt",frames="dynamic_frame_times.csv";
    {std::ofstream f(file);f<<std::setprecision(17);
     for(int i=0;i<=10;++i) {double t=i*.1,angle=2*t;
         // Alternate quaternion signs: interpolation must still use the shortest arc.
         double sign=i%2?-1:1;f<<t<<' '<<sign*std::cos(angle/2)<<" 0 0 "<<sign*std::sin(angle/2)<<'\n';}}
    ImuSeries imu;check(imu.load(file),"load dynamic IMU");W3Config cfg;
    cfg.t_ic[0]=.1;cfg.t_ic[1]=-.2;cfg.t_ic[2]=.3;WorldFrame wf(&imu,&cfg);
    double t=.359,query=t-.009,expected=2*query;auto q=imu.quatAt(t,-.009);auto r=q.toMat3();
    check(std::abs(std::atan2(r.m[1][0],r.m[0][0])-expected)<1e-10,"9ms sign and SLERP at non-sample time");
    Vec3 p(.3,-.2,4),w=wf.cam_to_world(t,p),back=wf.world_to_cam(t,w);
    check((p-back).norm()<1e-10,"nonidentity rotation + lever-arm inverse");
    Vec3 origin=wf.cam_to_world(t,{}),want=r*Vec3(.1,-.2,.3);
    check((origin-want).norm()<1e-10,"lever arm rotates with IMU");
    auto pixel=wf.project(t,w);auto ray=wf.undistortToNormalized(pixel);
    check(std::hypot(ray.x-p.x/p.z,ray.y-p.y/p.z)<1e-7,"distortion round trip");
    {std::ofstream f(file);f<<"0 1 0 0 0\n0 1 0 0 0\n";}check(!imu.load(file),"reject duplicate IMU time");
    {std::ofstream f(file);f<<"0 1 0 0 0\n1 0 0 0 0\n";}check(!imu.load(file),"reject zero quaternion");
    {std::ofstream f(file);f<<"0 1 0 0 0\nbad row\n1 1 0 0 0\n";}check(!imu.load(file),"do not skip corrupt index rows");
    {std::ofstream f(frames);f<<"frame,t_image_s\n0,0.520\n1,0.562\n2,0.603\n";}
    auto ts=loadFrameTimes(frames);check(ts.size()==3&&std::abs(ts[1]-.562)<1e-12,"explicit frame timestamp input");
    {std::ofstream f(frames);f<<"frame,t_image_s\n0,1\n2,2\n";}
    bool rejected=false;try{loadFrameTimes(frames);}catch(const std::exception&){rejected=true;}check(rejected,"reject missing frame index");
    std::filesystem::remove(file);std::filesystem::remove(frames);
    std::cout<<"Dynamic rotation, interpolation, time-offset and timestamp validation: "<<(errors?"FAIL":"PASS")<<'\n';return errors?1:0;
}
