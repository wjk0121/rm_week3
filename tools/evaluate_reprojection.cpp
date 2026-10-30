#include "detection_cache.hpp"
#include "camera.hpp"
#include "imu_series.hpp"
#include "measurement.hpp"
#include "state_visualizer.hpp"
#include "w3_config_loader.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
using namespace rmv;
int main(int argc,char** argv) {
    try {
        if(argc<6) throw std::runtime_error("usage: evaluate_reprojection states.csv detections.cache camera.yaml imu.txt output.csv [pitch_override_deg]");
        CameraParams cam;W3Config cfg;if(!loadCameraParams(argv[3],cam)||!loadW3Config(argv[3],cfg)) return 1;
        if(argc>6) cfg.plate.inward_pitch_deg=std::stod(argv[6]);
        ImuSeries imu;if(!imu.load(argv[4])) return 1;WorldFrame wf(&imu,&cfg);
        std::ifstream cache_file(argv[2]);std::string signature;std::getline(cache_file,signature);
        auto detections=readDetectionCache(argv[2],signature);
        std::ifstream in(argv[1]);std::string line,field;std::getline(in,line);std::stringstream hs(line);
        std::map<std::string,int> columns;int k=0;while(std::getline(hs,field,',')) columns[field]=k++;
        std::ofstream out(argv[5]);if(!out) throw std::runtime_error("Cannot write evaluation CSV");
        out<<"frame,n_detected,n_evaluated,rms_px\n";std::vector<double> errors;
        while(std::getline(in,line)) {
            std::stringstream row(line);std::vector<std::string> values;while(std::getline(row,field,',')) values.push_back(field);
            auto get=[&](const char* name,double fallback=0.) {auto it=columns.find(name);return it==columns.end()?fallback:std::stod(values.at(it->second));};
            int f=int(get("frame"));double t=get("t");if(f<0||f>=(int)detections.size()) throw std::runtime_error("State/cache frame mismatch");
            EkfState s;s.cx=get("cx");s.cy=get("cy");s.za=get("za");s.theta=get("theta");s.r=get("r");s.dr=get("dr");s.dh=get("dh");
            double sum=0;int count=0;
            if(std::isfinite(s.cx)) for(const auto& a:detections[f]) {
                double best=std::numeric_limits<double>::infinity();
                for(int j=0;j<4;++j) {
                    auto points=plateCornersWorld(s,j,cfg.plate,(a.size==ArmorSize::BIG?cam.big_width:cam.small_width)/1000.,(a.size==ArmorSize::BIG?cam.big_height:cam.small_height)/1000.);
                    auto center=wf.world_to_cam(t,plateWorldPosition(s,j,cfg.plate));
                    auto normal=wf.R_world_to_cam(t)*plateRotationWorld(s.theta+deg2rad(cfg.plate.phase_deg[j]),deg2rad(cfg.plate.inward_pitch_deg)).col(2);
                    if(normal.dot(center)<=0) continue;
                    double se=0;bool valid=true;
                    for(int p=0;p<4;++p) {
                        Vec3 v(points[p].x,points[p].y,points[p].z);if(wf.world_to_cam(t,v).z<=.05){valid=false;break;}
                        auto uv=wf.project(t,v);se+=std::pow(uv.x-a.corners[p].x,2)+std::pow(uv.y-a.corners[p].y,2);
                    }
                    if(valid) best=std::min(best,se/4.);
                }
                if(std::isfinite(best)) {sum+=best;++count;}
            }
            double rms=count?std::sqrt(sum/count):std::numeric_limits<double>::quiet_NaN();
            out<<f<<','<<detections[f].size()<<','<<count<<','<<rms<<'\n';
            if(f>=30&&std::isfinite(rms)) errors.push_back(rms);
        }
        if(errors.empty()) throw std::runtime_error("No evaluable frames");std::sort(errors.begin(),errors.end());
        std::cout<<"All raw detections, nearest front-facing predicted plate; no EKF gate selection. frames="<<errors.size()
                 <<" median_px="<<errors[errors.size()/2]<<" p90_px="<<errors[size_t(.9*(errors.size()-1))]<<'\n';
        return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
