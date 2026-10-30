#include "pnp_solver.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
PnpSolver::PnpSolver(const CameraParams& p):params_(p) {
    auto points=[](double w,double h) {return std::vector<cv::Point3f>{
        {float(-w/2),float(-h/2),0},{float(w/2),float(-h/2),0},
        {float(w/2),float(h/2),0},{float(-w/2),float(h/2),0}};};
    small_obj_=points(p.small_width,p.small_height);big_obj_=points(p.big_width,p.big_height);
}
std::vector<PoseResult> PnpSolver::solveCandidates(const Armor& a) const {
    std::vector<PoseResult> out;
    if(a.corners.size()!=4) return out;
    for(const auto& p:a.corners) if(!std::isfinite(p.x)||!std::isfinite(p.y)) return out;
    if(!cv::isContourConvex(a.corners) || std::abs(cv::contourArea(a.corners))<4) return out;
    const auto& obj=a.size==ArmorSize::BIG?big_obj_:small_obj_;
    try {
        std::vector<cv::Mat> rs,ts;
        cv::solvePnPGeneric(obj,a.corners,params_.camera_matrix,params_.dist_coeffs,
                            rs,ts,false,cv::SOLVEPNP_IPPE);
        for(size_t i=0;i<rs.size();++i) {
            if(!cv::checkRange(rs[i])||!cv::checkRange(ts[i])||ts[i].at<double>(2)<=0) continue;
            cv::Mat R;cv::Rodrigues(rs[i],R);
            if(R.col(2).dot(ts[i])<=0) continue;
            bool valid=true;
            for(const auto& p:obj) if(R.at<double>(2,0)*p.x+R.at<double>(2,1)*p.y+ts[i].at<double>(2)<=0) valid=false;
            if(!valid) continue;
            std::vector<cv::Point2f> uv;cv::projectPoints(obj,rs[i],ts[i],params_.camera_matrix,params_.dist_coeffs,uv);
            double se=0;for(int k=0;k<4;++k) {auto d=uv[k]-a.corners[k];se+=d.dot(d);}
            PoseResult p;p.ok=true;p.rvec=rs[i].clone();p.tvec=ts[i].clone();p.rms_px=std::sqrt(se/4);out.push_back(p);
        }
    } catch(const cv::Exception&) { out.clear(); }
    std::sort(out.begin(),out.end(),[](const auto& a,const auto& b){return a.rms_px<b.rms_px;});return out;
}
PoseResult PnpSolver::solve(const Armor& a) const {
    auto c=solveCandidates(a);return c.empty()?PoseResult():c.front();
}
