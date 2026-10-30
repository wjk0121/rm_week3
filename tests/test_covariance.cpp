#include "ekf9.hpp"
#include "measurement.hpp"
#include "small_matrix.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
using namespace rmv;
namespace {
int failures=0;
void check(bool ok,const char* name) {if(!ok){std::cerr<<"FAIL "<<name<<'\n';++failures;}}
void testCrossTermsAndBatch() {
    PlateGeometry g;VehicleEkf e(g);auto c=diagonalCovariance(.01,.01,.01,.01);c[1]=c[4]=.004;
    e.initFromMeasurement({1,0,.2},0,0,0,c.data());
    check(std::abs(e.cov(VehicleEkf::CX,VehicleEkf::CY)-.004)<1e-12,"full initial xy covariance");
    auto before=e.state();auto bad=c;bad[0]=-1;double z[4]={1,0,.2,0};
    check(!e.update(0,z,4,bad.data())&&e.state().cx==before.cx,"reject invalid R without mutation");
    z[0]=std::numeric_limits<double>::quiet_NaN();check(!e.update(0,z,4,c.data()),"reject NaN");
    VehicleEkf shared(g),independent(g);shared.initFromMeasurement({1,0,.2},0,0,0);independent.initFromMeasurement({1,0,.2},0,0,0);
    std::vector<PlateObservation> obs;
    for(int j:{0,2}) {Measurement m;m.p_w=plateWorldPosition(shared.state(),j,g);m.theta=plateThetaMeas(shared.state(),j,g);
        m.covariance=diagonalCovariance(1e-5,1e-5,1e-5,1e-5);m.shared_noise[6]=.10;obs.push_back({j,m});}
    check(shared.updateBatch(obs),"joint correlated update");
    for(auto& o:obs) {o.measurement.covariance=o.measurement.totalCovariance();o.measurement.shared_noise.fill(0);}
    check(independent.updateBatch(obs),"joint independent update");
    check(shared.cov(VehicleEkf::ZA,VehicleEkf::ZA)>independent.cov(VehicleEkf::ZA,VehicleEkf::ZA)*1.1,
          "shared noise is not counted twice as independent evidence");
    check(shared.covarianceHealthy()&&independent.covarianceHealthy(),"posterior SPD");
    VehicleEkf pos(g,false);pos.initFromMeasurement({1,0,.2},0,0,0,c.data());
    const double p[3]={1,0,.2};check(pos.update(0,p,3,c.data())&&pos.covarianceHealthy(),"3D observation stride");
}
void testUnequalGeometry() {
    PlateGeometry g;VehicleEkf e(g);EkfState truth;truth.cx=1.8;truth.cy=.1;truth.za=.2;
    truth.r=.225;truth.dr=.035;truth.dh=.045;
    auto c=diagonalCovariance(1e-5,1e-5,1e-5,deg2rad(.3)*deg2rad(.3));
    e.initFromMeasurement(plateWorldPosition(truth,0,g),0,0,0,c.data());
    for(int k=1;k<=300;++k) {
        truth.theta=wrapAngle(-1.3*k*.04);e.predict(.04);
        std::vector<PlateObservation> obs;
        for(int j=0;j<4;++j) {Measurement m;m.p_w=plateWorldPosition(truth,j,g);m.theta=plateThetaMeas(truth,j,g);m.covariance=c;obs.push_back({j,m});}
        check(e.updateBatch(obs),"unequal geometry update");
    }
    auto s=e.state();check(std::abs(s.r-truth.r)<.003&&std::abs(s.dr-truth.dr)<.003&&std::abs(s.dh-truth.dh)<.003,
                           "two radii and heights recover from wrong prior");
    // Finite differences check every Jacobian column for all four slots.
    double EkfState::* fields[VehicleEkf::N]={&EkfState::cx,&EkfState::vx,&EkfState::cy,&EkfState::vy,
        &EkfState::za,&EkfState::vz,&EkfState::theta,&EkfState::omega,&EkfState::r,&EkfState::dr,&EkfState::dh};
    for(int plate=0;plate<4;++plate) {
        double h[4],H[4*VehicleEkf::N];int d;e.observe(plate,h,H,d);
        for(int col=0;col<VehicleEkf::N;++col) {
            auto plus=s,minus=s;plus.*fields[col]+=1e-6;minus.*fields[col]-=1e-6;
            auto p=plateWorldPosition(plus,plate,g),m=plateWorldPosition(minus,plate,g);
            double derivative[4]={(p.x-m.x)/2e-6,(p.y-m.y)/2e-6,(p.z-m.z)/2e-6,
                wrapAngle(plateThetaMeas(plus,plate,g)-plateThetaMeas(minus,plate,g))/2e-6};
            for(int row=0;row<4;++row) check(std::abs(derivative[row]-H[row*VehicleEkf::N+col])<1e-6,
                                            "full observation Jacobian finite differences");
        }
    }
}
void testConsistency() {
    std::mt19937 random(431);std::normal_distribution<double> normal(0,1);
    double nis_sum=0,nis_dof=0,nees_sum=0;int accepted=0,finals=0;
    constexpr int trials=40,steps=180;
    for(int trial=0;trial<trials;++trial) {
        PlateGeometry g;VehicleEkf e(g);EkfState truth;
        truth.cx=2;truth.cy=.3;truth.za=.25;truth.theta=.2;truth.r=.22;truth.dr=.025;truth.dh=.035;
        truth.vx=.1;truth.vy=-.05;truth.omega=-1.4;
        auto c=diagonalCovariance(.005*.005,.005*.005,.005*.005,deg2rad(1)*deg2rad(1));
        auto measurement=[&](int j) {Measurement m;m.p_w=plateWorldPosition(truth,j,g);m.theta=plateThetaMeas(truth,j,g);m.covariance=c;
            m.p_w.x+=.005*normal(random);m.p_w.y+=.005*normal(random);m.p_w.z+=.005*normal(random);m.theta+=deg2rad(1)*normal(random);return m;};
        auto first=measurement(0);e.initFromMeasurement(first.p_w,first.theta,0,0,c.data());
        for(int k=1;k<=steps;++k) {
            constexpr double dt=.04;double a[4]={.4*normal(random),.4*normal(random),.15*normal(random),.8*normal(random)};
            truth.cx+=truth.vx*dt+.5*a[0]*dt*dt;truth.vx+=a[0]*dt;
            truth.cy+=truth.vy*dt+.5*a[1]*dt*dt;truth.vy+=a[1]*dt;
            truth.za+=truth.vz*dt+.5*a[2]*dt*dt;truth.vz+=a[2]*dt;
            truth.theta=wrapAngle(truth.theta+truth.omega*dt+.5*a[3]*dt*dt);truth.omega+=a[3]*dt;
            e.predict(dt,.4,.8,.15);std::vector<PlateObservation> obs;
            for(int j=0;j<4;++j) obs.push_back({j,measurement(j)});
            bool ok=e.updateBatch(obs);check(e.covarianceHealthy(),"Monte Carlo SPD");
            if(k>30) {nis_sum+=e.lastNis();nis_dof+=e.lastDof();accepted+=ok;}
        }
        const auto& s=e.state();const double error[VehicleEkf::N]={s.cx-truth.cx,s.vx-truth.vx,s.cy-truth.cy,s.vy-truth.vy,s.za-truth.za,s.vz-truth.vz,
            wrapAngle(s.theta-truth.theta),s.omega-truth.omega,s.r-truth.r,s.dr-truth.dr,s.dh-truth.dh};
        double p[VehicleEkf::N*VehicleEkf::N],inv[VehicleEkf::N*VehicleEkf::N];
        for(int i=0;i<VehicleEkf::N;++i) for(int j=0;j<VehicleEkf::N;++j) p[i*VehicleEkf::N+j]=e.cov(i,j);
        if(small::inverseSpd(p,VehicleEkf::N,inv)) {for(int i=0;i<VehicleEkf::N;++i) for(int j=0;j<VehicleEkf::N;++j) nees_sum+=error[i]*inv[i*VehicleEkf::N+j]*error[j];++finals;}
    }
    double ratio=nis_sum/nis_dof,nees=nees_sum/finals;
    std::cout<<"Monte Carlo: NIS/dof="<<ratio<<" final mean NEES="<<nees<<" (11 states), accepted="<<accepted<<'\n';
    check(ratio>.8&&ratio<1.2,"innovation consistency under stated noise model");
    check(finals==trials&&nees>5&&nees<17,"independent-trial NEES bounds");
}
}
int main() {testCrossTermsAndBatch();testUnequalGeometry();testConsistency();return failures?1:0;}
