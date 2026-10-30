#pragma once
#include"ekf9.hpp"
#include"w3_types.hpp"
#include<vector>
using namespace std;

namespace rmv
{
    class Tracker
    {
        public:
         Tracker(const PlateGeometry& geo,bool use_theta_obs = true,double gate_pos = 0.6,double gate_ang_deg = 40.0,int max_skip = 12);
         
         //一帧：为本帧所有量测
         const EkfState* step(double t,const vector<Measurement>& ms,TrackerDiag* diag = nullptr);

         bool initialized() const {return init_;}
         const Ekf9& ekf() const {return ekf_;}
         int initCount()const {return n_init_;}

        private:
         struct Pair
         {
            double cost = 0.0;
            int meas = -1;
            int plate = -1;
         };
         void initFrom(const Measurement& m,double t);
         vector<Pair>match(const vector<Measurement>& ms)const;
         void fillZ(const Measurement& m,double* z)const;
         PlateGeometry geo_;
         bool use_theta_obs_ = true;
         double gate_pos_ = 0.6;        //几何预门限
         double gate_ang_ = 0.7;
         int max_skip_ = 12;

         Ekf9 ekf_;
         bool init_ = false;
         double last_t_ = 0.0;
         int skip_ = 0;
         int n_init_ = 0;
         int target_id_ = -1;
    };
}                                       //namespace rmv
