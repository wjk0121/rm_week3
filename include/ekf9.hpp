#pragma once
#include "w3_types.hpp"
#include <array>
#include <vector>
namespace rmv {
class VehicleEkf {
public:
    static constexpr int N=11;
    enum Index { CX,VX,CY,VY,ZA,VZ,TH,OM,R,DR,DH };
    explicit VehicleEkf(const PlateGeometry&,bool use_theta_obs=true);
    void initFromMeasurement(const Vec3&,double theta,int plate,double t,const double* covariance=nullptr);
    void predict(double dt,double sigma_a=.8,double sigma_alpha=2.,double sigma_z=.4);
    // covariance always has stride 4, including in position-only mode.
    bool update(int plate,const double* z,int dim,const double* covariance=nullptr);
    bool updateBatch(const std::vector<PlateObservation>&);
    double mahalanobis(int plate,const double* z,int dim,const double* covariance=nullptr) const;
    void observe(int plate,double* h,double* H,int& dim) const;
    const EkfState& state() const { return state_; }
    double cov(int i,int j) const { return P_[i*N+j]; }
    bool covarianceHealthy() const;
    bool useThetaObs() const { return use_theta_obs_; }
    int obsDim() const { return use_theta_obs_?4:3; }
    double lastNis() const { return last_nis_; }
    int lastDof() const { return last_dof_; }
private:
    void getStateVector(double*) const;
    void setStateFromVector(const double*);
    bool validGeometry(const double*) const;
    bool innovation(const std::vector<PlateObservation>&,double* residual,double* H,
                    double* noise,double* inverseS,int& dim) const;
    EkfState state_;
    std::array<double,N*N> P_{};
    PlateGeometry geo_;
    bool use_theta_obs_;
    double last_nis_=0;
    int last_dof_=0;
};
using Ekf9=VehicleEkf; // Compatibility for earlier coursework; now eleven states.
}
