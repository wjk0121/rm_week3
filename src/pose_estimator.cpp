#include "pose_estimator.hpp"
#include "measurement.hpp"
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
namespace rmv {
namespace {
cv::Mat cvMatrix(const Mat3& r) {
    cv::Mat m(3,3,CV_64F);for(int i=0;i<3;++i) for(int j=0;j<3;++j) m.at<double>(i,j)=r.m[i][j];return m;
}
Vec3 column(const cv::Mat& r,int k) {return {r.at<double>(0,k),r.at<double>(1,k),r.at<double>(2,k)};}
using Project=std::function<bool(const cv::Mat&,cv::Mat&)>;
bool jacobian(const cv::Mat& x,const Project& project,cv::Mat& j) {
    cv::Mat initial;if(!project(x,initial)) return false;
    j=cv::Mat(initial.rows,x.rows,CV_64F);
    for(int k=0;k<x.rows;++k) {
        const double step=k<3?1e-5:1e-5;
        auto a=x.clone(),b=x.clone();a.at<double>(k)+=step;b.at<double>(k)-=step;
        cv::Mat pa,pb;if(!project(a,pa)||!project(b,pb)) return false;
        cv::Mat col=(pa-pb)/(2*step);col.copyTo(j.col(k));
    }return true;
}
bool fit(cv::Mat& x,const cv::Mat& observed,const Project& project,double& se) {
    cv::Mat predicted;if(!project(x,predicted)) return false;
    se=cv::norm(predicted-observed,cv::NORM_L2SQR);double damping=1e-3;
    for(int iteration=0;iteration<25;++iteration) {
        cv::Mat j;if(!jacobian(x,project,j)) return false;
        cv::Mat h=j.t()*j,g=j.t()*(predicted-observed),system=h.clone(),delta;
        for(int k=0;k<x.rows;++k) system.at<double>(k,k)+=damping*std::max(1.,h.at<double>(k,k));
        if(!cv::solve(system,-g,delta,cv::DECOMP_CHOLESKY)) return false;
        auto candidate=x+delta;cv::Mat next;
        if(project(candidate,next)) {
            double cost=cv::norm(next-observed,cv::NORM_L2SQR);
            if(cost<se) {
                x=candidate;predicted=next;double improvement=se-cost;se=cost;
                damping=std::max(1e-9,damping*.3);
                if(cv::norm(delta)<1e-7 || improvement<1e-9) break;
                continue;
            }
        }
        damping*=10;if(damping>1e10) break;
    }return true;
}
}
PoseEstimator::PoseEstimator(const CameraParams& c,const W3Config& g):camera_(c),config_(g),pnp_(c) {}
PoseMeasurement PoseEstimator::estimate(const Armor& armor,double t,const WorldFrame& wf) const {
    PoseMeasurement out;auto candidates=pnp_.solveCandidates(armor);out.candidates=static_cast<int>(candidates.size());
    if(candidates.empty()) return out;
    const Mat3 rw=wf.R_cam_to_world(t);const cv::Mat Rcw=cvMatrix(rw);
    const double w=(armor.size==ArmorSize::BIG?camera_.big_width:camera_.small_width)/1000.;
    const double h=(armor.size==ArmorSize::BIG?camera_.big_height:camera_.small_height)/1000.;
    const std::vector<cv::Point3d> object{{-w/2,-h/2,0},{w/2,-h/2,0},{w/2,h/2,0},{-w/2,h/2,0}};
    cv::Mat observed(8,1,CV_64F);for(int k=0;k<4;++k) {observed.at<double>(2*k)=armor.corners[k].x;observed.at<double>(2*k+1)=armor.corners[k].y;}
    const bool constrained=config_.constrain_plate_pitch;const int n=6;
    auto rotation=[&](const cv::Mat& x) {
        cv::Mat R;
        cv::Rodrigues(x.rowRange(3,6),R);
        return R;
    };
    Project project=[&](const cv::Mat& x,cv::Mat& uv) {
        if(!cv::checkRange(x)||x.at<double>(2)<=.05) return false;
        cv::Mat R=rotation(x),r;cv::Rodrigues(R,r);
        for(const auto& p:object) if(R.at<double>(2,0)*p.x+R.at<double>(2,1)*p.y+x.at<double>(2)<=.05) return false;
        std::vector<cv::Point2d> points;cv::projectPoints(object,r,x.rowRange(0,3),camera_.camera_matrix,camera_.dist_coeffs,points);
        uv=cv::Mat::zeros(constrained?10:8,1,CV_64F);
        for(int k=0;k<4;++k) {uv.at<double>(2*k)=points[k].x/config_.corner_sigma_px;uv.at<double>(2*k+1)=points[k].y/config_.corner_sigma_px;}
        if(constrained) {
            Vec3 normal=rw*column(R,2),tangent=rw*column(R,0);
            double pitch=std::atan2(normal.z,std::hypot(normal.x,normal.y));
            // Soft installation priors tolerate uncalibrated roll and modest chassis tilt.
            // A hard 15-degree constraint is inappropriate with assumed camera/IMU extrinsics.
            uv.at<double>(8)=(pitch-deg2rad(config_.plate.inward_pitch_deg))/deg2rad(config_.plate_pitch_sigma_deg);
            uv.at<double>(9)=std::asin(std::clamp(tangent.z,-1.,1.))/deg2rad(config_.plate_roll_sigma_deg);
        }
        return cv::checkRange(uv);
    };
    cv::Mat target=cv::Mat::zeros(constrained?10:8,1,CV_64F);
    cv::Mat scaled=observed/config_.corner_sigma_px;scaled.copyTo(target.rowRange(0,8));
    cv::Mat best;double best_cost=std::numeric_limits<double>::infinity();
    for(const auto& candidate:candidates) {
        cv::Mat rawR;cv::Rodrigues(candidate.rvec,rawR);Vec3 normal=rw*column(rawR,2);
        double raw_pitch=rad2deg(std::atan2(normal.z,std::hypot(normal.x,normal.y)));
        cv::Mat x(n,1,CV_64F);cv::Mat translation=candidate.tvec/1000.;translation.copyTo(x.rowRange(0,3));
        candidate.rvec.copyTo(x.rowRange(3,6));
        double cost;if(!fit(x,target,project,cost)) continue;
        // Compare both refined candidates under identical pixel and installation weights.
        double score=cost;
        if(!constrained) score+=std::pow((raw_pitch-config_.plate.inward_pitch_deg)/5.,2);
        if(score<best_cost) {best_cost=score;best=x.clone();out.raw_pitch_deg=raw_pitch;out.raw_rms_px=candidate.rms_px;}
    }
    if(best.empty()) return out;
    cv::Mat uv,J;if(!project(best,uv)||!jacobian(best,project,J)) return out;
    const double se=cv::norm(uv.rowRange(0,8)-scaled,cv::NORM_L2SQR)*config_.corner_sigma_px*config_.corner_sigma_px;
    cv::Mat R=rotation(best),rvec;cv::Rodrigues(R,rvec);
    // Reject a refinement that flips the inward face to the camera.
    if(R.col(2).dot(best.rowRange(0,3))<=0) return out;
    out.pose={true,rvec.clone(),best.rowRange(0,3)*1000.,std::sqrt(se/4.)};
    if(out.pose.rms_px>4. || cv::norm(best.rowRange(0,3))>15.) return out;
    auto& m=out.measurement;
    const Vec3 pcam(best.at<double>(0),best.at<double>(1),best.at<double>(2));m.p_w=wf.cam_to_world(t,pcam);
    const Vec3 normal=rw*column(R,2);m.theta=std::atan2(normal.y,normal.x);m.is_big=armor.size==ArmorSize::BIG;
    cv::Mat inverse;cv::SVD svd(J,cv::SVD::NO_UV);
    if(svd.w.at<double>(n-1)<1e-7 || !cv::invert(J.t()*J,inverse,cv::DECOMP_SVD)) return out;
    // J is whitened by corner uncertainty and the soft mechanical-prior uncertainty.
    const double inflation=std::max(1.,cv::norm(uv-target,cv::NORM_L2SQR)/std::max(1,uv.rows-n));
    cv::Mat A=cv::Mat::zeros(4,n,CV_64F);Rcw.copyTo(A(cv::Rect(0,0,3,3)));
    {
        for(int k=3;k<6;++k) {
            auto a=best.clone(),b=best.clone();a.at<double>(k)+=1e-5;b.at<double>(k)-=1e-5;
            Vec3 na=rw*column(rotation(a),2),nb=rw*column(rotation(b),2);
            A.at<double>(3,k)=wrapAngle(std::atan2(na.y,na.x)-std::atan2(nb.y,nb.x))/2e-5;
        }
    }
    cv::Mat covariance=A*(inflation*inverse)*A.t();
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) m.covariance[i*4+j]=.5*(covariance.at<double>(i,j)+covariance.at<double>(j,i));
    for(int k=0;k<3;++k) m.covariance[k*4+k]+=config_.position_noise_floor_m*config_.position_noise_floor_m;
    m.covariance[15]+=std::pow(deg2rad(config_.yaw_noise_floor_deg),2);
    const double sigma=deg2rad(config_.imu_attitude_sigma_deg);
    const Vec3 axes[3]={{1,0,0},{0,1,0},{0,0,1}};
    for(int k=0;k<3;++k) {
        Vec3 dp=axes[k].cross(m.p_w)*sigma,dn=axes[k].cross(normal)*sigma;
        m.shared_noise[k]=dp.x;m.shared_noise[3+k]=dp.y;m.shared_noise[6+k]=dp.z;
        m.shared_noise[9+k]=(normal.x*dn.y-normal.y*dn.x)/(normal.x*normal.x+normal.y*normal.y);
    }
    out.ok=true;return out;
}
}
