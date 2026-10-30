#include "ekf9.hpp"
#include "measurement.hpp"
#include "small_matrix.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace rmv {
VehicleEkf::VehicleEkf(const PlateGeometry& g,bool a):geo_(g),use_theta_obs_(a) {
    for(int i=0;i<N;++i) P_[i*N+i]=1e6;
}
void VehicleEkf::getStateVector(double* x) const {
    const double a[N]={state_.cx,state_.vx,state_.cy,state_.vy,state_.za,state_.vz,
        state_.theta,state_.omega,state_.r,state_.dr,state_.dh}; std::copy(a,a+N,x);
}
void VehicleEkf::setStateFromVector(const double* x) {
    state_={x[CX],x[VX],x[CY],x[VY],x[ZA],x[VZ],x[TH],x[OM],x[R],x[DR],x[DH]};
}
bool VehicleEkf::validGeometry(const double* x) const {
    for(int k=0;k<N;++k) if(!std::isfinite(x[k])) return false;
    if(std::abs(x[DH])>.20) return false;
    for(int j=0;j<4;++j) {
        double r=x[R]+geo_.radius_offset[j]+(j%2?x[DR]:0.);
        if(r<geo_.radius_min || r>geo_.radius_max) return false;
    }
    return true;
}
void VehicleEkf::initFromMeasurement(const Vec3& p,double angle,int plate,double,const double* covariance) {
    if(plate<0 || plate>3 || !std::isfinite(angle) || !std::isfinite(p.x) ||
       !std::isfinite(p.y) || !std::isfinite(p.z)) throw std::invalid_argument("Invalid EKF initial measurement");
    auto c=Measurement().covariance;
    if(covariance) std::copy(covariance,covariance+16,c.begin());
    double check[16];
    if(!small::inverseSpd(c.data(),4,check)) throw std::invalid_argument("Invalid measurement covariance");
    const double ca=std::cos(angle),sa=std::sin(angle),r=geo_.radius+geo_.radius_offset[plate];
    state_={p.x+r*ca,0,p.y+r*sa,0,p.z-geo_.dh[plate],0,
        wrapAngle(angle-deg2rad(geo_.phase_deg[plate])),0,geo_.radius,0,0};
    // Full propagation of centre = plate + r * normal, including position/yaw correlation.
    constexpr int M=7; // [px,py,pz,yaw,r,dr,dh]
    double b[N*M]={},v[M*M]={};
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) v[i*M+j]=c[i*4+j];
    if(!use_theta_obs_) { for(int k=0;k<4;++k) v[3*M+k]=v[k*M+3]=0; v[3*M+3]=.3*.3; }
    v[4*M+4]=.05*.05;
    v[5*M+5]=v[6*M+6]=geo_.estimate_pair_geometry?.04*.04:1e-12;
    b[CX*M]=1;b[CX*M+3]=-r*sa;b[CX*M+4]=ca;b[CX*M+5]=(plate%2)*ca;
    b[CY*M+1]=1;b[CY*M+3]=r*ca;b[CY*M+4]=sa;b[CY*M+5]=(plate%2)*sa;
    b[ZA*M+2]=1;b[ZA*M+6]=-(plate%2);b[TH*M+3]=1;
    b[R*M+4]=1;b[DR*M+5]=1;b[DH*M+6]=1;
    P_.fill(0);
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) for(int k=0;k<M;++k) for(int l=0;l<M;++l)
        P_[i*N+j]+=b[i*M+k]*v[k*M+l]*b[j*M+l];
    P_[VX*N+VX]=P_[VY*N+VY]=1;P_[VZ*N+VZ]=.25;P_[OM*N+OM]=1;
    last_nis_=0;last_dof_=0;
}
void VehicleEkf::predict(double dt,double sigma_a,double sigma_alpha,double sigma_z) {
    if(!std::isfinite(dt) || dt<=0) throw std::invalid_argument("Prediction dt must be positive");
    double x[N];getStateVector(x);
    double f[N*N]={};for(int i=0;i<N;++i) f[i*N+i]=1;
    for(int k:{CX,CY,ZA,TH}) { x[k]+=x[k+1]*dt;f[k*N+k+1]=dt; }
    x[TH]=wrapAngle(x[TH]);
    std::array<double,N*N> next{};
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) for(int k=0;k<N;++k) for(int l=0;l<N;++l)
        next[i*N+j]+=f[i*N+k]*P_[k*N+l]*f[j*N+l];
    const double q[4]={sigma_a*sigma_a,sigma_a*sigma_a,sigma_z*sigma_z,sigma_alpha*sigma_alpha};
    for(int a=0;a<4;++a) {
        int k=2*a;next[k*N+k]+=.25*dt*dt*dt*dt*q[a];
        next[k*N+k+1]+=.5*dt*dt*dt*q[a];next[(k+1)*N+k]+=.5*dt*dt*dt*q[a];
        next[(k+1)*N+k+1]+=dt*dt*q[a];
    }
    next[R*N+R]+=2.4e-7*dt;
    if(geo_.estimate_pair_geometry) { next[DR*N+DR]+=1e-8*dt;next[DH*N+DH]+=1e-8*dt; }
    P_=next;setStateFromVector(x);
}
void VehicleEkf::observe(int plate,double* h,double* H,int& dim) const {
    const double odd=plate%2,a=state_.theta+deg2rad(geo_.phase_deg[plate]);
    const double r=state_.r+geo_.radius_offset[plate]+odd*state_.dr,ca=std::cos(a),sa=std::sin(a);
    h[0]=state_.cx-r*ca;h[1]=state_.cy-r*sa;h[2]=state_.za+geo_.dh[plate]+odd*state_.dh;
    std::fill(H,H+4*N,0.);
    H[CX]=1;H[TH]=r*sa;H[R]=-ca;H[DR]=-odd*ca;
    H[N+CY]=1;H[N+TH]=-r*ca;H[N+R]=-sa;H[N+DR]=-odd*sa;
    H[2*N+ZA]=1;H[2*N+DH]=odd;
    dim=obsDim();if(use_theta_obs_) { h[3]=wrapAngle(a);H[3*N+TH]=1; }
}
bool VehicleEkf::innovation(const std::vector<PlateObservation>& obs,double* y,double* H,
                            double* noise,double* inverseS,int& dim) const {
    const int d=obsDim();dim=d*static_cast<int>(obs.size());
    if(obs.empty() || obs.size()>4) return false;
    std::fill(noise,noise+dim*dim,0.);
    for(int b=0;b<(int)obs.size();++b) {
        if(obs[b].plate<0 || obs[b].plate>3) return false;
        const auto& m=obs[b].measurement;double h[4],hb[4*N],check[16];int hd;
        if(!small::inverseSpd(m.covariance.data(),4,check)) return false;
        observe(obs[b].plate,h,hb,hd);
        const double z[4]={m.p_w.x,m.p_w.y,m.p_w.z,m.theta};
        for(int i=0;i<d;++i) {
            y[b*d+i]=i==3?wrapAngle(z[i]-h[i]):z[i]-h[i];
            if(!std::isfinite(y[b*d+i])) return false;
            std::copy(hb+i*N,hb+(i+1)*N,H+(b*d+i)*N);
            for(int j=0;j<d;++j) noise[(b*d+i)*dim+b*d+j]=m.covariance[i*4+j];
        }
    }
    // Off-diagonal blocks retain shared IMU attitude noise between simultaneous plates.
    for(int a=0;a<(int)obs.size();++a) for(int b=0;b<(int)obs.size();++b)
        for(int i=0;i<d;++i) for(int j=0;j<d;++j) for(int k=0;k<3;++k)
            noise[(a*d+i)*dim+b*d+j]+=obs[a].measurement.shared_noise[i*3+k]*obs[b].measurement.shared_noise[j*3+k];
    double s[256]={};
    for(int i=0;i<dim;++i) for(int j=0;j<dim;++j) {
        s[i*dim+j]=noise[i*dim+j];
        for(int k=0;k<N;++k) for(int l=0;l<N;++l) s[i*dim+j]+=H[i*N+k]*P_[k*N+l]*H[j*N+l];
    }
    return small::inverseSpd(s,dim,inverseS);
}
double VehicleEkf::mahalanobis(int plate,const double* z,int dim,const double* covariance) const {
    if(dim!=obsDim()) return std::numeric_limits<double>::infinity();
    Measurement m;m.p_w={z[0],z[1],z[2]};if(dim==4) m.theta=z[3];
    if(covariance) std::copy(covariance,covariance+16,m.covariance.begin());
    double y[16],h[16*N],r[256],si[256];int d;
    if(!innovation({{plate,m}},y,h,r,si,d)) return std::numeric_limits<double>::infinity();
    double value=0;for(int i=0;i<d;++i) for(int j=0;j<d;++j) value+=y[i]*si[i*d+j]*y[j];
    return value;
}
bool VehicleEkf::update(int plate,const double* z,int dim,const double* covariance) {
    if(dim!=obsDim()) return false;
    Measurement m;m.p_w={z[0],z[1],z[2]};if(dim==4) m.theta=z[3];
    if(covariance) std::copy(covariance,covariance+16,m.covariance.begin());
    return updateBatch({{plate,m}});
}
bool VehicleEkf::updateBatch(const std::vector<PlateObservation>& obs) {
    double y[16],H[16*N],noise[256],si[256];int d;
    last_nis_=std::numeric_limits<double>::quiet_NaN();last_dof_=0;
    if(!innovation(obs,y,H,noise,si,d)) return false;
    last_nis_=0;last_dof_=d;
    for(int i=0;i<d;++i) for(int j=0;j<d;++j) last_nis_+=y[i]*si[i*d+j]*y[j];
    if(!std::isfinite(last_nis_) || last_nis_>small::chiSquareGate(d)) return false;
    double pht[N*16]={},gain[N*16]={},x[N];getStateVector(x);
    for(int i=0;i<N;++i) for(int j=0;j<d;++j) for(int k=0;k<N;++k)
        pht[i*d+j]+=P_[i*N+k]*H[j*N+k];
    for(int i=0;i<N;++i) for(int j=0;j<d;++j) for(int k=0;k<d;++k)
        gain[i*d+j]+=pht[i*d+k]*si[k*d+j];
    for(int i=0;i<N;++i) for(int j=0;j<d;++j) x[i]+=gain[i*d+j]*y[j];
    x[TH]=wrapAngle(x[TH]);if(!validGeometry(x)) return false;
    double a[N*N]={},ap[N*N]={};std::array<double,N*N> next{};
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) {
        a[i*N+j]=(i==j?1.:0.);for(int k=0;k<d;++k) a[i*N+j]-=gain[i*d+k]*H[k*N+j];
    }
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) for(int k=0;k<N;++k)
        ap[i*N+j]+=a[i*N+k]*P_[k*N+j];
    // Joseph update; keep full cross terms and commit state/covariance together.
    for(int i=0;i<N;++i) for(int j=0;j<N;++j) {
        for(int k=0;k<N;++k) next[i*N+j]+=ap[i*N+k]*a[j*N+k];
        for(int k=0;k<d;++k) for(int l=0;l<d;++l) next[i*N+j]+=gain[i*d+k]*noise[k*d+l]*gain[j*d+l];
    }
    for(int i=0;i<N;++i) for(int j=0;j<i;++j)
        next[i*N+j]=next[j*N+i]=.5*(next[i*N+j]+next[j*N+i]);
    double check[N*N];if(!small::inverseSpd(next.data(),N,check)) return false;
    P_=next;setStateFromVector(x);return true;
}
bool VehicleEkf::covarianceHealthy() const {
    double inverse[N*N];return small::inverseSpd(P_.data(),N,inverse);
}
}
