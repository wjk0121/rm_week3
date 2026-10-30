//IMU 姿态序列：读imu.txt文件，存储IMU姿态序列
#pragma once
#include"math3d.hpp"
#include<string>
#include<vector>
using namespace std;
namespace rmv
{
    struct ImuSample
    {
        double t=0.0;       //时间戳，来自imu.txt第一列
        Quaternion q;       //载体系姿态四元数(w,x,y,z)
    };
    class ImuSeries
    {
        public:
            bool load(const string& path);
            bool empty() const { return samples_.empty(); }
            size_t size() const { return samples_.size(); }
            double tBegin() const;
            double tEnd() const;
            //查询t时刻的载体系姿态
            //offset的含义：t_query = t_sample + offset
            rmv::Mat3 rotationAt(double t,double offset) const;
            
            //查询原始四元数
            rmv::Quaternion quatAt(double t,double offset) const;

            //直接读取第idx条imu数据
            double timeAt(int idx) const;

            //平均采样率=（n-1)/(t_end-t_begin),用于给输出视频的帧率参考
            double meanRate() const;
        
        private:
            vector<ImuSample> samples_;


    
    };

}                           //namespace rmv
