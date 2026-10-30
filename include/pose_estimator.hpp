#pragma once
#include "pnp_solver.hpp"
#include "world_frame.hpp"
#include "w3_types.hpp"
namespace rmv {
struct PoseMeasurement {
    bool ok=false;
    PoseResult pose;
    Measurement measurement;
    double raw_rms_px=0,raw_pitch_deg=0;
    int candidates=0;
};
class PoseEstimator {
public:
    PoseEstimator(const CameraParams& camera,const W3Config& config);
    PoseMeasurement estimate(const Armor&,double t,const WorldFrame&) const;
private:
    CameraParams camera_;
    W3Config config_;
    PnpSolver pnp_;
};
}
