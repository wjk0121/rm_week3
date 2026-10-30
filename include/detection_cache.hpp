#pragma once
#include "types.hpp"
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace rmv {
inline std::string fileFingerprint(const std::string& path) {
    std::ifstream f(path,std::ios::binary);if(!f) throw std::runtime_error("Cannot fingerprint "+path);
    std::uint64_t h=14695981039346656037ull;char b[65536];
    while(f.read(b,sizeof(b))||f.gcount()) for(std::streamsize k=0;k<f.gcount();++k) {h^=static_cast<unsigned char>(b[k]);h*=1099511628211ull;}
    std::ostringstream out;out<<std::hex<<h;return out.str(); // Cache identity, not a cryptographic checksum.
}
inline std::vector<std::vector<Armor>> readDetectionCache(const std::string& path,const std::string& signature) {
    std::ifstream f(path);std::string header;std::getline(f,header);
    if(header!=signature) throw std::runtime_error("Detection cache signature mismatch: "+path);
    std::vector<std::vector<Armor>> result;int frame,count;
    while(f>>frame>>count) {
        if(frame!=(int)result.size() || count<0 || count>100) throw std::runtime_error("Invalid detection cache frame");
        std::vector<Armor> detections;
        for(int i=0;i<count;++i) {
            Armor a;int big;if(!(f>>a.number>>big>>a.confidence>>a.label)) throw std::runtime_error("Truncated detection cache");
            a.size=big?ArmorSize::BIG:ArmorSize::SMALL;a.corners.resize(4);
            for(auto& p:a.corners) if(!(f>>p.x>>p.y)) throw std::runtime_error("Truncated cached corners");
            detections.push_back(a);
        }result.push_back(detections);
    }
    if(!f.eof()) throw std::runtime_error("Invalid detection cache tail");
    return result;
}
inline void writeDetectionCache(const std::string& path,const std::string& signature,
                                const std::vector<std::vector<Armor>>& frames) {
    auto parent=std::filesystem::path(path).parent_path();if(!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream f(path+".tmp");if(!f) throw std::runtime_error("Cannot write detection cache");
    f<<signature<<'\n'<<std::setprecision(9);
    for(size_t n=0;n<frames.size();++n) {
        f<<n<<' '<<frames[n].size()<<'\n';
        for(const auto& a:frames[n]) {
            f<<a.number<<' '<<(a.size==ArmorSize::BIG)<<' '<<a.confidence<<' '<<a.label;
            for(auto p:a.corners) f<<' '<<p.x<<' '<<p.y;
            f<<'\n';
        }
    }
    f.flush();if(!f) throw std::runtime_error("Failed writing detection cache");f.close();
    std::filesystem::rename(path+".tmp",path);
}
}
