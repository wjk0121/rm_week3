#include "tracker.hpp"
#include "measurement.hpp"
#include "small_matrix.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rmv {
Tracker::Tracker(const PlateGeometry& g,bool a,double gp,double ga,int skip)
    :geo_(g),use_theta_obs_(a),gate_pos_(gp),gate_ang_(deg2rad(ga)),max_skip_(skip),ekf_(g,a) {}
void Tracker::initFrom(const Measurement& m,double t) {
    ekf_=VehicleEkf(geo_,use_theta_obs_);auto c=m.totalCovariance();
    ekf_.initFromMeasurement(m.p_w,m.theta,0,t,c.data());
    target_id_=m.target_id;init_=true;last_t_=t;skip_=0;++n_init_;
}
void Tracker::fillZ(const Measurement& m,double* z) const {
    z[0]=m.p_w.x;z[1]=m.p_w.y;z[2]=m.p_w.z;if(use_theta_obs_) z[3]=m.theta;
}
std::vector<Tracker::Pair> Tracker::match(const std::vector<Measurement>& ms) const {
    std::vector<Pair> out;
    for(int j=0;j<4;++j) {
        const auto p=plateWorldPosition(ekf_.state(),j,geo_);
        double angle=plateThetaMeas(ekf_.state(),j,geo_);
        for(int i=0;i<(int)ms.size();++i) {
            if(target_id_>=0 && ms[i].target_id>=0 && ms[i].target_id!=target_id_) continue;
            double dp=std::hypot(ms[i].p_w.x-p.x,ms[i].p_w.y-p.y);
            double da=std::abs(wrapAngle(ms[i].theta-angle));
            if(dp>gate_pos_ || (use_theta_obs_ && da>gate_ang_)) continue;
            double z[4]={};fillZ(ms[i],z);auto c=ms[i].totalCovariance();
            double d2=ekf_.mahalanobis(j,z,ekf_.obsDim(),c.data());
            if(std::isfinite(d2) && d2<=small::chiSquareGate(ekf_.obsDim())) out.push_back({d2,i,j});
        }
    }
    std::sort(out.begin(),out.end(),[](const Pair& a,const Pair& b){
        if(a.cost!=b.cost) return a.cost<b.cost;
        return a.plate!=b.plate?a.plate<b.plate:a.meas<b.meas;
    });return out;
}
const EkfState* Tracker::step(double t,const std::vector<Measurement>& ms,TrackerDiag* diag) {
    if(!std::isfinite(t)) throw std::invalid_argument("Nonfinite image timestamp");
    TrackerDiag d;d.n_meas=static_cast<int>(ms.size());
    auto initialize=[&] {
        initFrom(ms.front(),t);d.initialization=true;d.reinit=n_init_>1;
        d.matched_j=0;d.matched_meas=0;d.n_matched=1;d.matched_meas_by_plate[0]=0;
    };
    auto finish=[&]() -> const EkfState* { if(diag) *diag=d;return init_?&ekf_.state():nullptr; };
    if(!init_) { if(!ms.empty()) initialize();return finish(); }
    double dt=t-last_t_;
    if(dt<=0) throw std::invalid_argument("Image timestamps must be strictly increasing");
    if(dt>.2) { init_=false;skip_=0;if(!ms.empty()) initialize();last_t_=t;return finish(); }
    ekf_.predict(dt);last_t_=t;d.predicted=ekf_.state();d.has_prediction=true;
    const auto candidates=match(ms);
    std::vector<bool> used(ms.size(),false);std::array<bool,4> used_plate{};
    std::vector<Pair> selected;
    for(const auto& p:candidates) if(!used[p.meas] && !used_plate[p.plate]) {
        selected.push_back(p);used[p.meas]=true;used_plate[p.plate]=true;
    }
    // All pairs use the same prediction. A joint update retains same-frame correlations.
    // If the joint gate fails, remove the least compatible pair and retry transactionally.
    while(!selected.empty()) {
        std::vector<PlateObservation> observations;
        for(const auto& p:selected) observations.push_back({p.plate,ms[p.meas]});
        if(ekf_.updateBatch(observations)) break;
        selected.pop_back();
    }
    if(!selected.empty()) {
        skip_=0;d.n_matched=static_cast<int>(selected.size());
        d.matched_j=selected.front().plate;d.matched_meas=selected.front().meas;
        for(const auto& p:selected) d.matched_meas_by_plate[p.plate]=p.meas;
        d.nis=ekf_.lastNis();d.nis_dof=ekf_.lastDof();
    } else {
        ++skip_;d.skip=true;
        if(skip_>=max_skip_) { init_=false;if(!ms.empty()) initialize(); }
    }
    return finish();
}
}
