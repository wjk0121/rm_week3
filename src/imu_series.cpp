#include "imu_series.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
namespace rmv {
bool ImuSeries::load(const std::string& path) {
    samples_.clear();std::ifstream in(path);if(!in) return false;
    std::string line;int row=0;
    while(std::getline(in,line)) {
        ++row;auto start=line.find_first_not_of(" \t\r");
        if(start==std::string::npos || line[start]=='#') continue;
        std::istringstream ss(line);ImuSample s;std::string extra;
        if(!(ss>>s.t>>s.q.w>>s.q.x>>s.q.y>>s.q.z) || (ss>>extra) ||
           !std::isfinite(s.t) || !std::isfinite(s.q.norm()) || s.q.norm()<1e-6 ||
           (!samples_.empty() && s.t<=samples_.back().t)) {
            std::cerr<<"Invalid/nonmonotonic IMU row "<<row<<"\n";samples_.clear();return false;
        }
        s.q=s.q.normalized();samples_.push_back(s);
    }
    return samples_.size()>=2;
}
double ImuSeries::tBegin() const {return samples_.empty()?0:samples_.front().t;}
double ImuSeries::tEnd() const {return samples_.empty()?0:samples_.back().t;}
Quaternion ImuSeries::quatAt(double t,double offset) const {
    if(samples_.empty() || !std::isfinite(t+offset)) throw std::invalid_argument("Invalid IMU query");
    const double query=t+offset;
    if(query<=tBegin()) return samples_.front().q;
    if(query>=tEnd()) return samples_.back().q;
    auto it=std::lower_bound(samples_.begin(),samples_.end(),query,[](const auto& s,double v){return s.t<v;});
    const auto& a=*(it-1);return slerp(a.q,it->q,(query-a.t)/(it->t-a.t));
}
Mat3 ImuSeries::rotationAt(double t,double offset) const {return quatAt(t,offset).toMat3();}
double ImuSeries::timeAt(int index) const {
    if(index<0 || index>=(int)samples_.size()) throw std::out_of_range("IMU sample index");
    return samples_[index].t;
}
double ImuSeries::meanRate() const {
    return samples_.size()<2?0:double(samples_.size()-1)/(tEnd()-tBegin());
}
}
