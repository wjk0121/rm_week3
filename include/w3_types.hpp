#pragma once
#include "math3d.hpp"
#include <array>
#include <limits>
#include <string>
namespace rmv {
// Fixed slots; observing a new plate does not reset the car reference.
// [cx,vx,cy,vy,za,vz,theta,omega,r,dr,dh]. Odd slots use r+dr and za+dh.
struct EkfState {
    double cx=0,vx=0,cy=0,vy=0,za=0,vz=0,theta=0,omega=0,r=0,dr=0,dh=0;
};
struct PlateGeometry {
    double radius=.20, radius_min=.12, radius_max=.40, inward_pitch_deg=-15.;
    bool estimate_pair_geometry=true;
    std::array<double,4> radius_offset{{0,0,0,0}}, phase_deg{{0,90,180,270}}, dh{{0,0,0,0}};
};
using Covariance4=std::array<double,16>;
inline Covariance4 diagonalCovariance(double x,double y,double z,double a) {
    Covariance4 c{}; c[0]=x; c[5]=y; c[10]=z; c[15]=a; return c;
}
struct Measurement {
    Vec3 p_w;
    double theta=0;
    bool is_big=false;
    int target_id=-1;
    // Independent noise, full row-major covariance in [x,y,z,yaw].
    Covariance4 covariance=diagonalCovariance(.03*.03,.03*.03,.10*.10,deg2rad(3)*deg2rad(3));
    // G times unit Gaussian: the SAME attitude error affects all plates in a frame.
    std::array<double,12> shared_noise{};
    Covariance4 totalCovariance() const {
        auto c=covariance;
        for(int i=0;i<4;++i) for(int j=0;j<4;++j) for(int k=0;k<3;++k)
            c[i*4+j]+=shared_noise[i*3+k]*shared_noise[j*3+k];
        return c;
    }
};
struct PlateObservation { int plate=0; Measurement measurement; };
struct TrackerDiag {
    int matched_j=-1,matched_meas=-1,n_meas=0,n_matched=0;
    std::array<int,4> matched_meas_by_plate{{-1,-1,-1,-1}};
    bool initialization=false,reinit=false,skip=false,has_prediction=false;
    double nis=std::numeric_limits<double>::quiet_NaN();
    int nis_dof=0;
    EkfState predicted;
};
struct W3Config {
    double K[9]={2362.2795952460974,0,679.17513921636282,0,2336.1916035376717,529.06858905837635,0,0,1};
    double D[5]={-.098561592108101559,.11173066531932681,-.0038352945867108721,-.0020421443354436304,0};
    double R_ci[9]={0,0,1,-1,0,0,0,-1,0},t_ic[3]={0,0,0};
    double img_imu_offset=-.009,corner_sigma_px=1.5,position_noise_floor_m=.01;
    double yaw_noise_floor_deg=3.,imu_attitude_sigma_deg=.3;
    double plate_pitch_sigma_deg=5.,plate_roll_sigma_deg=5.;
    double traditional_corner_sigma_px=3.,traditional_position_floor_m=.02,traditional_yaw_floor_deg=5.;
    bool constrain_plate_pitch=true;
    int use_theta_obs=1;
    PlateGeometry plate;
    std::string imu_path;
};
}
