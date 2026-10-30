#include "frame_timeline.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
namespace rmv {
std::vector<double> loadFrameTimes(const std::string& path) {
    std::ifstream in(path);if(!in) throw std::runtime_error("Cannot open frame timestamps: "+path);
    std::string line;std::vector<double> times;
    if(!std::getline(in,line) || (line!="frame,t_image_s" && line!="frame,t_image_s\r"))
        throw std::runtime_error("Frame timestamp header must be frame,t_image_s");
    while(std::getline(in,line)) {
        if(line.empty() || line=="\r") continue;
        std::istringstream ss(line);int frame;double t;char comma;std::string extra;
        if(!(ss>>frame>>comma>>t) || comma!=',' || (ss>>extra) || frame!=(int)times.size() ||
           !std::isfinite(t) || (!times.empty() && t<=times.back()))
            throw std::runtime_error("Invalid/nonmonotonic frame timestamp row");
        times.push_back(t);
    }
    if(times.empty()) throw std::runtime_error("Empty frame timestamp file");
    return times;
}
}
