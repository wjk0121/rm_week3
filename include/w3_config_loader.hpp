#pragma once 
#include "w3_types.hpp"
#include<string>
using namespace std;

namespace rmv
{
    bool loadW3Config(const string& path,W3Config& cfg);

}   //namespace rmv