#include "pose_estimator.hpp"
#include "measurement.hpp"
#include "state_visualizer.hpp"
#include "w3_config_loader.hpp"
#include "small_matrix.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
using namespace rmv;
int main() {
    int errors=0;auto check=[&](bool b,const char* n){if(!b){std::cerr<<"FAIL "<<n<<'\n';++errors;}};
    W3Config cfg;cfg.corner_sigma_px=.5;cfg.position_noise_floor_m=1e-6;cfg.yaw_noise_floor_deg=1e-4;cfg.imu_attitude_sigma_deg=0;
    CameraParams cam;cam.camera_matrix=cv::Mat(3,3,CV_64F,cfg.K).clone();cam.dist_coeffs=cv::Mat(1,5,CV_64F,cfg.D).clone();
    const std::string file="pose_test_imu.txt";{std::ofstream f(file);f<<"0 1 0 0 0\n1 1 0 0 0\n";}
    ImuSeries imu;check(imu.load(file),"pose IMU");WorldFrame wf(&imu,&cfg);PoseEstimator estimator(cam,cfg);PnpSolver raw(cam);
    const double t=.5,yaw=.35;const Vec3 pcam(.04,-.03,2.0),world=wf.cam_to_world(t,pcam);
    auto Rw=plateRotationWorld(yaw,deg2rad(-15));Mat3 Rc=wf.R_world_to_cam(t)*Rw;
    cv::Mat R(3,3,CV_64F);for(int i=0;i<3;++i) for(int j=0;j<3;++j) R.at<double>(i,j)=Rc.m[i][j];
    cv::Mat rv;cv::Rodrigues(R,rv);std::vector<cv::Point3d> obj{{-.0675,-.0275,0},{.0675,-.0275,0},{.0675,.0275,0},{-.0675,.0275,0}};
    std::vector<cv::Point2d> projected;cv::projectPoints(obj,rv,cv::Vec3d(pcam.x,pcam.y,pcam.z),cam.camera_matrix,cam.dist_coeffs,projected);
    Armor armor;for(auto p:projected) armor.corners.emplace_back(p);auto candidates=raw.solveCandidates(armor);
    check(candidates.size()==2,"IPPE both candidates retained");auto result=estimator.estimate(armor,t,wf);
    check(result.ok,"constrained solve");
    if(result.ok) {
        check((result.measurement.p_w-world).norm()<1e-4,"exact synthetic position");
        check(std::abs(wrapAngle(result.measurement.theta-yaw))<1e-4,"exact synthetic yaw and pitch prior");
        double inv[16];check(small::inverseSpd(result.measurement.covariance.data(),4,inv),"full pose covariance SPD");
        check(std::abs(result.measurement.covariance[3])>1e-10,"position/yaw correlation retained");
    }
    // Regression: uncalibrated camera/IMU roll in the real clip is a few degrees.
    // A hard pitch/upright constraint rejected most frames; soft priors must tolerate it.
    cv::Mat roll_vector=(cv::Mat_<double>(3,1)<<.07,0,0),roll_rotation;
    cv::Rodrigues(roll_vector,roll_rotation);cv::Mat tilted=roll_rotation*R,tilted_r;
    cv::Rodrigues(tilted,tilted_r);std::vector<cv::Point2d> tilted_pixels;
    cv::projectPoints(obj,tilted_r,cv::Vec3d(pcam.x,pcam.y,pcam.z),cam.camera_matrix,cam.dist_coeffs,tilted_pixels);
    Armor tilted_armor;for(auto p:tilted_pixels) tilted_armor.corners.emplace_back(p);
    auto tilted_result=estimator.estimate(tilted_armor,t,wf);
    check(tilted_result.ok&&tilted_result.pose.rms_px<1,"soft installation prior tolerates residual roll");
    std::mt19937 rng(17);std::normal_distribution<double> noise(0,.5);double nees=0;int count=0;
    for(int k=0;k<100;++k) {
        Armor noisy=armor;for(auto& p:noisy.corners){p.x+=noise(rng);p.y+=noise(rng);}
        auto fit=estimator.estimate(noisy,t,wf);if(!fit.ok) continue;
        auto& m=fit.measurement;double e[4]={m.p_w.x-world.x,m.p_w.y-world.y,m.p_w.z-world.z,wrapAngle(m.theta-yaw)},inv[16];
        if(!small::inverseSpd(m.covariance.data(),4,inv)) continue;
        for(int i=0;i<4;++i) for(int j=0;j<4;++j) nees+=e[i]*inv[i*4+j]*e[j];++count;
    }
    std::cout<<"Pixel-noise Monte Carlo: valid="<<count<<" mean pose NEES="<<nees/count<<" (4 DoF)\n";
    check(count==100&&nees/count>1&&nees/count<7,"pixel Jacobian uncertainty propagation");
    // A bad YAML matrix must not silently fall back to built-in calibration values.
    const std::string yaml="invalid_pose_config.yaml";
    {cv::FileStorage f(yaml,cv::FileStorage::WRITE);f<<"camera_matrix"<<cv::Mat::eye(2,2,CV_64F)<<"dist_coeffs"<<cv::Mat::zeros(1,5,CV_64F);}
    W3Config invalid;check(!loadW3Config(yaml,invalid),"reject malformed calibration shape");
    std::filesystem::remove(file);std::filesystem::remove(yaml);return errors?1:0;
}
