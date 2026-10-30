#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include "detection_cache.hpp"
#include "frame_timeline.hpp"
#include "imu_series.hpp"
#include "measurement.hpp"
#include "nn_detector.hpp"
#include "number_classifier.hpp"
#include "pose_estimator.hpp"
#include "state_visualizer.hpp"
#include "tracker.hpp"
#include "traditional_detector.hpp"
#include "visualizer.hpp"
#include "w3_config_loader.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
namespace fs=std::filesystem;
using namespace rmv;
namespace {
constexpr double NaN=std::numeric_limits<double>::quiet_NaN();
struct Args {
    std::string input="data/test.avi",imu="data/imu.txt",camera="camera.yaml",models="models",method="nn";
    std::string mode="imu-index",frame_times,cache,out="results/overlay_week3.avi",csv="results/states_week3.csv";
    bool mode_explicit=false,video=true,debug=true,show=false;
    int max_frames=0;double start=NaN,offset=NaN,radius=NaN;
};
double number(const std::string& s) {
    size_t used;double v=std::stod(s,&used);if(used!=s.size()||!std::isfinite(v)) throw std::runtime_error("Invalid number: "+s);return v;
}
Args parse(int argc,char** argv) {
    Args a;
    for(int i=1;i<argc;++i) {
        std::string s=argv[i];auto value=[&]() {if(i+1>=argc) throw std::runtime_error("Missing value for "+s);return std::string(argv[++i]);};
        if(s=="--imu") a.imu=value();else if(s=="--camera") a.camera=value();else if(s=="--models") a.models=value();
        else if(s=="--method") a.method=value();else if(s=="--out") a.out=value();else if(s=="--csv") a.csv=value();
        else if(s=="--time-mode") {a.mode=value();a.mode_explicit=true;}
        else if(s=="--frame-times") a.frame_times=value();else if(s=="--detection-cache") a.cache=value();
        else if(s=="--video-start-time") a.start=number(value());else if(s=="--imu-offset") a.offset=number(value());
        else if(s=="--plate-radius") a.radius=number(value());
        else if(s=="--max-frames") {double n=number(value());if(n<0||n>10000000||n!=std::floor(n)) throw std::runtime_error("Invalid frame limit");a.max_frames=int(n);}
        else if(s=="--no-video") a.video=false;else if(s=="--show") a.show=true;
        else if(s=="--debug-detections") a.debug=true;else if(s=="--no-debug-detections") a.debug=false;
        else if(!s.empty()&&s[0]!='-') a.input=s;else throw std::runtime_error("Unknown argument: "+s);
    }
    if(!a.mode_explicit) {if(!a.frame_times.empty()) a.mode="timestamps";else if(std::isfinite(a.start)) a.mode="video";}
    if(a.mode!="imu-index"&&a.mode!="video"&&a.mode!="timestamps") throw std::runtime_error("Invalid time mode");
    if((a.mode=="timestamps")!=(!a.frame_times.empty())) throw std::runtime_error("timestamps mode requires --frame-times");
    if((a.mode=="video")!=std::isfinite(a.start)) throw std::runtime_error("video mode requires --video-start-time exclusively");
    if(a.method!="nn"&&a.method!="trad") throw std::runtime_error("method must be nn or trad");
    return a;
}
void parent(const std::string& path) {if(!fs::path(path).parent_path().empty()) fs::create_directories(fs::path(path).parent_path());}
double reprojection(const Armor& a,int j,const EkfState& s,const W3Config& cfg,const CameraParams& cam,const WorldFrame& wf,double t) {
    auto points=plateCornersWorld(s,j,cfg.plate,(a.size==ArmorSize::BIG?cam.big_width:cam.small_width)/1000.,
                                  (a.size==ArmorSize::BIG?cam.big_height:cam.small_height)/1000.);
    double sum=0;
    for(int k=0;k<4;++k) {
        Vec3 p(points[k].x,points[k].y,points[k].z);if(wf.world_to_cam(t,p).z<=.05) return NaN;
        auto uv=wf.project(t,p);double dx=uv.x-a.corners[k].x,dy=uv.y-a.corners[k].y;sum+=dx*dx+dy*dy;
    }return std::sqrt(sum/4);
}
double allMatchedReprojection(const std::vector<Armor>& armors,const TrackerDiag& d,const EkfState& s,
                              const W3Config& cfg,const CameraParams& cam,const WorldFrame& wf,double t) {
    double sum=0;int n=0;
    for(int j=0;j<4;++j) {int i=d.matched_meas_by_plate[j];if(i<0) continue;
        double r=reprojection(armors[i],j,s,cfg,cam,wf,t);if(std::isfinite(r)){sum+=r*r;++n;}}
    return n?std::sqrt(sum/n):NaN;
}
int run(const Args& a) {
    CameraParams cam;W3Config cfg;
    if(!loadCameraParams(a.camera,cam)||!loadW3Config(a.camera,cfg)) throw std::runtime_error("Invalid camera/configuration");
    if(std::isfinite(a.offset)) cfg.img_imu_offset=a.offset;
    if(a.method=="trad") {
        cfg.corner_sigma_px=cfg.traditional_corner_sigma_px;
        cfg.position_noise_floor_m=cfg.traditional_position_floor_m;
        cfg.yaw_noise_floor_deg=cfg.traditional_yaw_floor_deg;
    }
    if(std::isfinite(a.radius)) {
        for(double dr:cfg.plate.radius_offset) if(a.radius+dr<cfg.plate.radius_min||a.radius+dr>cfg.plate.radius_max)
            throw std::runtime_error("--plate-radius outside configured bounds");
        cfg.plate.radius=a.radius;
    }
    ImuSeries imu;if(!imu.load(a.imu)) throw std::runtime_error("Invalid IMU data");
    auto times=a.mode=="timestamps"?loadFrameTimes(a.frame_times):std::vector<double>();
    WorldFrame wf(&imu,&cfg);PoseEstimator poses(cam,cfg);Tracker tracker(cfg.plate,cfg.use_theta_obs!=0);
    cv::VideoCapture cap(a.input);if(!cap.isOpened()) throw std::runtime_error("Cannot open input video");
    const int width=int(cap.get(cv::CAP_PROP_FRAME_WIDTH)),height=int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const int frames=int(std::llround(cap.get(cv::CAP_PROP_FRAME_COUNT)));const double video_fps=cap.get(cv::CAP_PROP_FPS);
    if(width!=cam.image_width||height!=cam.image_height) throw std::runtime_error("Video/calibration resolution mismatch");
    if(a.mode=="imu-index"&&frames>0&&frames!=(int)imu.size()) throw std::runtime_error("Index mode requires equal video/IMU counts");
    if(a.mode=="timestamps"&&frames>0&&frames!=(int)times.size()) throw std::runtime_error("Frame timestamp count mismatch");
    if(a.mode=="video"&&(!std::isfinite(video_fps)||video_fps<=0)) throw std::runtime_error("Invalid video FPS");
    double fps=a.mode=="video"?video_fps:(a.mode=="timestamps"?double(times.size()-1)/(times.back()-times.front()):imu.meanRate());
    if(!std::isfinite(fps)||fps<=0) throw std::runtime_error("Invalid output FPS");
    if(a.mode=="imu-index") std::cerr<<"ASSUMPTION: image i uses IMU row i as its image timestamp; recording correspondence is UNVERIFIED.\n";
    if(a.mode=="video") std::cerr<<"ASSUMPTION: container times are capture times; verify separately for this recording.\n";
    std::cout<<"Input "<<width<<'x'<<height<<", frames="<<frames<<", IMU="<<imu.size()<<", output_fps="<<fps
             <<", offset="<<cfg.img_imu_offset<<", state_dim="<<VehicleEkf::N<<"\n";
    const std::string model=a.models+(a.method=="nn"?"/Infantry-v8n.onnx":"/mlp.onnx");
    const std::string signature="rm_week3_detections_v1 "+fileFingerprint(a.input)+" "+fileFingerprint(model)+" "+a.method+" "+enemyColorName(cam.enemy_color);
    std::vector<std::vector<Armor>> cached;bool use_cache=!a.cache.empty()&&fs::exists(a.cache);
    if(use_cache) {cached=readDetectionCache(a.cache,signature);if(frames>0&&frames!=(int)cached.size()) throw std::runtime_error("Incomplete detection cache");}
    std::unique_ptr<NumberClassifier> classifier;std::unique_ptr<TraditionalDetector> traditional;std::unique_ptr<NnDetector> nn;
    if(!use_cache) {
        if(a.method=="nn") nn=std::make_unique<NnDetector>(model,cam.enemy_color);
        else {classifier=std::make_unique<NumberClassifier>(model);traditional=std::make_unique<TraditionalDetector>(cam,*classifier);}
    }
    parent(a.csv);if(a.video) parent(a.out);
    cv::VideoWriter writer;if(a.video) {
        writer.open(a.out,cv::VideoWriter::fourcc('M','J','P','G'),fps,{width,height});
        if(!writer.isOpened()) throw std::runtime_error("Cannot create output video");
    }
    std::ofstream csv(a.csv);if(!csv) throw std::runtime_error("Cannot create output CSV");csv<<std::setprecision(10);
    csv<<"frame,t,cx,vx,cy,vy,za,vz,theta,omega,r,dr,dh,matched_j,n_detected,n_meas,n_matched,initialization,reinit,imu_clamped,pre_reproj_rms_px,pre_z_residual_m,pre_reproj_all_rms_px,post_reproj_all_rms_px,nis,nis_dof,covariance_ok";
    for(int k=0;k<VehicleEkf::N;++k) csv<<",P"<<k;
    csv<<'\n';
    std::array<bool,4> big{};cv::Mat frame;int fid=0,matched_frames=0,rejected=0,clamped=0;bool stopped=false;
    double previous=-std::numeric_limits<double>::infinity();
    while((a.max_frames==0||fid<a.max_frames)&&cap.read(frame)) {
        double t;
        if(a.mode=="imu-index") t=imu.timeAt(fid);
        else if(a.mode=="timestamps") {if(fid>=(int)times.size()) throw std::runtime_error("Missing frame timestamp");t=times[fid];}
        else {double ms=cap.get(cv::CAP_PROP_POS_MSEC);double relative=(std::isfinite(ms)&&ms>=0&&(fid==0||ms>0))?ms/1000.:fid/video_fps;t=a.start+relative;}
        if(t<=previous) throw std::runtime_error("Nonmonotonic image timestamps");
        previous=t;
        double query=t+cfg.img_imu_offset;bool boundary=query<imu.tBegin()-1e-9||query>imu.tEnd()+1e-9;
        // The legacy index assumption has no earlier attitude for its first frame only.
        if(boundary && !(a.mode=="imu-index"&&fid==0&&query<imu.tBegin()&&imu.tBegin()-query<=.02))
            throw std::runtime_error("Frame outside IMU coverage: "+std::to_string(fid));
        if(boundary) ++clamped;
        std::vector<Armor> armors;
        if(use_cache) {if(fid>=(int)cached.size()) throw std::runtime_error("Missing cached frame");armors=cached[fid];}
        else {armors=nn?nn->detect(frame):traditional->detect(frame);if(!a.cache.empty()) cached.push_back(armors);}
        std::vector<Measurement> measurements;std::vector<Armor> accepted;std::vector<std::pair<Armor,PoseResult>> debug;
        for(const auto& armor:armors) {
            auto result=poses.estimate(armor,t,wf);if(a.debug) debug.push_back({armor,result.pose});
            if(!result.ok) {++rejected;continue;}measurements.push_back(result.measurement);accepted.push_back(armor);
        }
        TrackerDiag diag;const auto* state=tracker.step(t,measurements,&diag);
        double pre=NaN,zres=NaN,pre_all=NaN,post_all=NaN;
        if(diag.has_prediction && !diag.initialization && diag.matched_j>=0) {
            pre=reprojection(accepted[diag.matched_meas],diag.matched_j,diag.predicted,cfg,cam,wf,t);
            zres=measurements[diag.matched_meas].p_w.z-plateWorldPosition(diag.predicted,diag.matched_j,cfg.plate).z;
            pre_all=allMatchedReprojection(accepted,diag,diag.predicted,cfg,cam,wf,t);
        }
        if(state) {
            if(diag.initialization) big.fill(false);
            for(int j=0;j<4;++j) if(diag.matched_meas_by_plate[j]>=0) big[j]=measurements[diag.matched_meas_by_plate[j]].is_big;
            post_all=allMatchedReprojection(accepted,diag,*state,cfg,cam,wf,t);
            if(a.video) {
                if(a.debug) Visualizer::drawResults(frame,debug);
                drawPlates(frame,wf,t,*state,cfg.plate,big,cam.small_width/1000.,cam.small_height/1000.,cam.big_width/1000.,cam.big_height/1000.,diag.matched_j);
                drawStateText(frame,*state,fid,diag.matched_j,diag.n_meas);
                if(a.mode=="imu-index") cv::putText(frame,"time: index assumption",{20,height-20},cv::FONT_HERSHEY_SIMPLEX,.5,{0,220,255},1,cv::LINE_AA);
            }
        }
        if(diag.n_matched) ++matched_frames;
        csv<<fid<<','<<t;
        if(state) csv<<','<<state->cx<<','<<state->vx<<','<<state->cy<<','<<state->vy<<','<<state->za<<','<<state->vz<<','<<state->theta<<','<<state->omega<<','<<state->r<<','<<state->dr<<','<<state->dh;
        else for(int k=0;k<VehicleEkf::N;++k) csv<<",nan";
        csv<<','<<diag.matched_j<<','<<armors.size()<<','<<measurements.size()<<','<<diag.n_matched<<','<<diag.initialization<<','<<diag.reinit<<','<<boundary
           <<','<<pre<<','<<zres<<','<<pre_all<<','<<post_all<<','<<diag.nis<<','<<diag.nis_dof<<','<<(state&&tracker.ekf().covarianceHealthy());
        for(int k=0;k<VehicleEkf::N;++k) csv<<','<<(state?tracker.ekf().cov(k,k):NaN);
        csv<<'\n';
        if(a.video) writer.write(frame);
        ++fid;
        if(a.show) {cv::imshow("rm_week3",frame);if(cv::waitKey(1)==27){stopped=true;break;}}
    }
    if(a.video) writer.release();
    csv.flush();if(!csv) throw std::runtime_error("CSV write failed");csv.close();
    bool limited=a.max_frames>0&&fid==a.max_frames;
    if(!stopped&&!limited&&frames>0&&fid!=frames) throw std::runtime_error("Video decode ended before declared frame count");
    if(!a.cache.empty()&&!use_cache&&!stopped&&!limited) writeDetectionCache(a.cache,signature,cached);
    cv::FileStorage meta(a.csv+".meta.json",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    if(!meta.isOpened()) throw std::runtime_error("Cannot write run metadata");
    meta<<"frames"<<fid<<"matched_frames"<<matched_frames<<"pose_rejected"<<rejected<<"initializations"<<tracker.initCount()
        <<"reinitializations"<<std::max(0,tracker.initCount()-1)<<"state_dim"<<VehicleEkf::N<<"time_mode"<<a.mode
        <<"frame_times"<<a.frame_times<<"timestamp_evidence"<<(a.mode=="timestamps"?"provided_timestamps":"unverified_assumption")
        <<"time_correspondence_verified"<<0
        <<"imu_offset_s"<<cfg.img_imu_offset<<"imu_clamped_frames"<<clamped<<"plate_inward_pitch_deg"<<cfg.plate.inward_pitch_deg
        <<"pitch_constrained"<<cfg.constrain_plate_pitch<<"pair_geometry_estimated"<<cfg.plate.estimate_pair_geometry
        <<"corner_sigma_px"<<cfg.corner_sigma_px<<"position_floor_m"<<cfg.position_noise_floor_m
        <<"yaw_floor_deg"<<cfg.yaw_noise_floor_deg<<"imu_attitude_sigma_deg"<<cfg.imu_attitude_sigma_deg
        <<"video_fingerprint"<<fileFingerprint(a.input)<<"imu_fingerprint"<<fileFingerprint(a.imu)<<"config_fingerprint"<<fileFingerprint(a.camera)
        <<"detection_signature"<<signature<<"opencv_version"<<CV_VERSION
        <<"extrinsics"<<"assumed_axis_map_and_configured_lever_arm; not calibrated by this program"
        <<"translation_reference"<<"IMU origin assumed stationary; no ego translation measurement"
        <<"full_run"<<(!limited&&!stopped);
    meta.release();
    std::cout<<"Processed="<<fid<<" matched_frames="<<matched_frames<<" pose_rejected="<<rejected
             <<" initializations="<<tracker.initCount()<<" reinitializations="<<std::max(0,tracker.initCount()-1)<<" imu_clamped="<<clamped<<"\n";
    if(tracker.initialized()) {auto s=tracker.ekf().state();std::cout<<"Final radius="<<s.r<<" dr="<<s.dr<<" dh="<<s.dh<<" omega="<<s.omega<<"\n";}
    return 0;
}
}
int main(int argc,char** argv) {
    try {return run(parse(argc,argv));}catch(const std::exception& e) {std::cerr<<"Error: "<<e.what()<<'\n';return 1;}
}
