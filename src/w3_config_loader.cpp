#include "w3_config_loader.hpp"
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>
#include <stdexcept>
namespace rmv {
namespace {
void matrix(const cv::FileStorage& f,const char* key,double* out,int rows,int cols,bool required=false) {
    if(f[key].empty()) {if(required) throw std::runtime_error(std::string("Missing ")+key);return;}
    cv::Mat m;f[key]>>m;
    if(m.rows!=rows || m.cols!=cols || m.channels()!=1) throw std::runtime_error(std::string("Invalid shape: ")+key);
    m.convertTo(m,CV_64F);if(!cv::checkRange(m)) throw std::runtime_error(std::string("Nonfinite ")+key);
    for(int i=0;i<rows;++i) for(int j=0;j<cols;++j) out[i*cols+j]=m.at<double>(i,j);
}
void scalar(const cv::FileStorage& f,const char* key,double& value) {
    if(!f[key].empty()) f[key]>>value;
    if(!std::isfinite(value)) throw std::runtime_error(std::string("Nonfinite ")+key);
}
void boolean(const cv::FileStorage& f,const char* key,bool& value) {
    if(f[key].empty()) return;
    int n;f[key]>>n;
    if(n!=0 && n!=1) throw std::runtime_error(std::string("Boolean must be 0/1: ")+key);
    value=n!=0;
}
}
bool loadW3Config(const std::string& path,W3Config& cfg) {
    try {
        cv::FileStorage f(path,cv::FileStorage::READ);if(!f.isOpened()) return false;
        matrix(f,"camera_matrix",cfg.K,3,3,true);matrix(f,"dist_coeffs",cfg.D,1,5,true);
        matrix(f,"R_ci",cfg.R_ci,3,3);matrix(f,"t_ic",cfg.t_ic,1,3);
        matrix(f,"plate_radius_offset",cfg.plate.radius_offset.data(),1,4);
        matrix(f,"plate_phase_deg",cfg.plate.phase_deg.data(),1,4);matrix(f,"plate_dh",cfg.plate.dh.data(),1,4);
        scalar(f,"img_imu_offset",cfg.img_imu_offset);scalar(f,"plate_radius",cfg.plate.radius);
        scalar(f,"radius_min",cfg.plate.radius_min);scalar(f,"radius_max",cfg.plate.radius_max);
        scalar(f,"plate_inward_pitch_deg",cfg.plate.inward_pitch_deg);
        scalar(f,"corner_sigma_px",cfg.corner_sigma_px);scalar(f,"position_noise_floor_m",cfg.position_noise_floor_m);
        scalar(f,"yaw_noise_floor_deg",cfg.yaw_noise_floor_deg);scalar(f,"imu_attitude_sigma_deg",cfg.imu_attitude_sigma_deg);
        scalar(f,"plate_pitch_sigma_deg",cfg.plate_pitch_sigma_deg);scalar(f,"plate_roll_sigma_deg",cfg.plate_roll_sigma_deg);
        scalar(f,"traditional_corner_sigma_px",cfg.traditional_corner_sigma_px);
        scalar(f,"traditional_position_floor_m",cfg.traditional_position_floor_m);
        scalar(f,"traditional_yaw_floor_deg",cfg.traditional_yaw_floor_deg);
        boolean(f,"constrain_plate_pitch",cfg.constrain_plate_pitch);
        boolean(f,"estimate_pair_geometry",cfg.plate.estimate_pair_geometry);
        bool angle=cfg.use_theta_obs!=0;boolean(f,"use_theta_obs",angle);cfg.use_theta_obs=angle;
        cv::Mat r(3,3,CV_64F,cfg.R_ci);
        if(cv::norm(r.t()*r-cv::Mat::eye(3,3,CV_64F))>1e-6 || std::abs(cv::determinant(r)-1)>1e-6)
            throw std::runtime_error("R_ci must be a proper rotation");
        if(cfg.plate.radius_min<=0 || cfg.plate.radius_max<=cfg.plate.radius_min ||
           cfg.corner_sigma_px<=0 || cfg.position_noise_floor_m<=0 || cfg.yaw_noise_floor_deg<=0 ||
           cfg.imu_attitude_sigma_deg<0 || cfg.plate_pitch_sigma_deg<=0 || cfg.plate_roll_sigma_deg<=0 ||
           cfg.traditional_corner_sigma_px<=0 || cfg.traditional_position_floor_m<=0 || cfg.traditional_yaw_floor_deg<=0 ||
           std::abs(cfg.plate.inward_pitch_deg)>=60)
            throw std::runtime_error("Invalid geometry/noise configuration");
        for(int j=0;j<4;++j) {
            double rj=cfg.plate.radius+cfg.plate.radius_offset[j];
            if(rj<cfg.plate.radius_min || rj>cfg.plate.radius_max) throw std::runtime_error("Initial radius outside mechanical bounds");
        }
        return true;
    } catch(const std::exception& e) {std::cerr<<"Invalid week3 configuration: "<<e.what()<<"\n";return false;}
}
}
