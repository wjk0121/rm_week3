#pragma once
#include <string>
#include <vector>
namespace rmv {
// CSV header frame,t_image_s; exact sequential indices and strictly increasing timestamps.
std::vector<double> loadFrameTimes(const std::string& path);
}
