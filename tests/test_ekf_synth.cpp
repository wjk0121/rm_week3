//  合成数据自检:EKF + 跟踪器
//
//  造一辆"匀速平动 + 匀速自旋"的车,按四板模型生成量测(同时可见的板都喂进去),
//  中间塞两段遮挡和一段野值,看滤波器能否跟住、丢失后能否重捕。
//
//  跑两组工况:
//    A. 真值半径 == 配置初值(0.200 m)     -> 看纯跟踪精度
//    B. 真值半径 != 配置初值(0.232 m)     -> 看 r 能不能自己估出来
//  只跑 A 的话 r 一开局就是对的,"r 收敛"这句话等于没验。
//
//  编译(不依赖 OpenCV):
//      g++ -std=c++17 -O2 -Iinclude tests/test_ekf_synth.cpp src/ekf9.cpp src/tracker.cpp -o build/test_ekf_synth

#include "ekf9.hpp"
#include "measurement.hpp"
#include "tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace rmv;

namespace
{

//  哪几块板朝向相机。
//  注意 theta 的方向是"板->车心",所以 (cos,sin) 是**内法线**;
//  外法线(车心->板)是它的反向。板朝向相机 <=> 外法线·(相机-板) > 0。
//  相机在世界原点,(相机-板) = -p,展开后等价于 内法线·p > 0。
//  真车在 45° 附近通常同时看到 2 块;只喂 1 块的话,
//  跟踪器里"多条量测依次更新"那条路径永远跑不到,等于没测。
std::vector<int> visiblePlates(const EkfState& s, const PlateGeometry& g)
{
    std::vector<int> out;
    for (int j = 0; j < 4; ++j)
    {
        const Vec3 p = plateWorldPosition(s, j, g);
        const double phi = s.theta + deg2rad(g.phase_deg[j]);
        const double nx = std::cos(phi), ny = std::sin(phi);   //  内法线(板->车心)
        if (nx * p.x + ny * p.y > 0.0) out.push_back(j);       //  这块板正对相机
    }
    if (out.empty()) out.push_back(0);
    return out;
}

double median(std::vector<double> v)
{
    if (v.empty()) return 1e9;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

double percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 1e9;
    std::sort(v.begin(), v.end());
    const double idx = p * (v.size() - 1);
    const std::size_t i = (std::size_t)std::floor(idx);
    const std::size_t j = (std::size_t)std::min<double>(i + 1, v.size() - 1);
    const double f = idx - i;
    return v[i] * (1.0 - f) + v[j] * f;
}

struct CaseResult
{
    int evaluated = 0;
    double pos_med = 0.0, pos_p95 = 0.0, ang_med = 0.0;
    double omega_final = 0.0, r_final = 0.0;
    int n_init = 0;
    bool ok = true;
};

//  r_true: 真值半径。滤波器自己只知道 geo.radius(= 配置初值),两者的差要被估出来。
CaseResult runCase(double r_true)
{
    const double dt = 1.0 / 24.09;          //  与实测 IMU 采样率一致
    const int n = 480;

    PlateGeometry geo;                      //  配置初值 r=0.200,相位 0/90/180/270

    //  ---- 真值轨迹 ----
    const double cx0 = 1.5, cy0 = -0.2;
    const double theta0 = wrapAngle(std::atan2(cy0, cx0));   //  0 号板开局正对相机
    const double vx = 0.30, vy = 0.08, wz = 1.6;

    std::mt19937 rng(7);                    //  固定种子,结果可复现
    std::normal_distribution<double> n_pos(0.0, 0.003);    //  3 mm 位置噪声
    std::normal_distribution<double> n_ang(0.0, deg2rad(0.5));
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    Tracker tracker(geo, true);
    std::vector<double> pos_err, ang_err;
    CaseResult res;

    for (int k = 0; k < n; ++k)
    {
        const double t = k * dt;

        EkfState truth;
        truth.cx = cx0 + vx * t;
        truth.cy = cy0 + vy * t;
        truth.za = 0.24;
        truth.theta = wrapAngle(theta0 + wz * t);
        truth.omega = wz;
        truth.r = r_true;

        //  ---- 遮挡:150~167、300~319 帧;野值:220~231 帧 ----
        const bool occluded = (k >= 150 && k < 168) || (k >= 300 && k < 320);
        const bool garbage = (k >= 220 && k < 232);

        std::vector<Measurement> ms;
        if (!occluded)
        {
            for (int j : visiblePlates(truth, geo))
            {
                Measurement m;
                m.p_w = plateWorldPosition(truth, j, geo);
                m.theta = plateThetaMeas(truth, j, geo);
                m.p_w.x += n_pos(rng);
                m.p_w.y += n_pos(rng);
                m.p_w.z += n_pos(rng);
                m.theta += n_ang(rng);
                m.is_big = (j % 2 == 1);
                ms.push_back(m);
            }
            if (garbage && u01(rng) < 0.9 && !ms.empty())
            {
                ms[0].p_w.x += 3.0 * (u01(rng) - 0.5);     //  几米级野值
                ms[0].p_w.y += 3.0 * (u01(rng) - 0.5);
            }
        }

        TrackerDiag diag;
        const EkfState* est = tracker.step(t, ms, &diag);
        if (!est) continue;

        //  ---- 误差统计:先让滤波器收敛 30 帧 ----
        if (k < 30) continue;
        ++res.evaluated;
        const double dx = est->cx - truth.cx;
        const double dy = est->cy - truth.cy;
        pos_err.push_back(std::sqrt(dx * dx + dy * dy));

        //  重初始化后"哪块板算 0 号"会重新猜,整车 theta 可能整体差 k*90°,
        //  观测上等价(theta 转 90° = 板号平移一块),所以对 90° 取模比较
        const double raw = std::fabs(wrapAngle(est->theta - truth.theta));
        double best = raw;
        for (int kk = 1; kk < 4; ++kk)
            best = std::min(best, std::fabs(wrapAngle(raw - kk * kPi / 2.0)));
        ang_err.push_back(rad2deg(best));
    }

    const EkfState& fs = tracker.ekf().state();
    res.pos_med = median(pos_err);
    res.pos_p95 = percentile(pos_err, 0.95);
    res.ang_med = median(ang_err);
    res.omega_final = fs.omega;
    res.r_final = fs.r;
    res.n_init = tracker.initCount();

    if (res.pos_med > 0.02) res.ok = false;
    if (res.pos_p95 > 0.05) res.ok = false;
    if (res.ang_med > 2.0) res.ok = false;
    if (std::fabs(res.omega_final - wz) > 0.15) res.ok = false;
    if (std::fabs(res.r_final - r_true) > 0.015) res.ok = false;
    return res;
}

void report(const char* tag, const CaseResult& r, double r_true)
{
    std::printf("---- %s ----\n", tag);
    std::printf("评估帧数        : %d\n", r.evaluated);
    std::printf("位置误差 中位数 : %.4f m\n", r.pos_med);
    std::printf("位置误差 95%%   : %.4f m\n", r.pos_p95);
    std::printf("自转角误差 中位 : %.2f deg (对 90° 取模)\n", r.ang_med);
    std::printf("最终 omega      : %.3f rad/s (真值 1.600)\n", r.omega_final);
    std::printf("最终 r          : %.3f m   (真值 %.3f, 配置初值 0.200)\n", r.r_final,
                r_true);
    std::printf("重新初始化次数  : %d\n", r.n_init);
    std::printf("%s\n\n", r.ok ? "[PASS]" : "[FAIL]");
}

}   //  namespace

int main()
{
    //  A: 真值 == 配置初值, 看纯跟踪精度
    const CaseResult a = runCase(0.200);
    //  B: 真值比配置初值大 3.2 cm, 看 r 能不能自己收敛过去
    const CaseResult b = runCase(0.232);

    report("A. 半径与配置一致 (0.200 m)", a, 0.200);
    report("B. 半径与配置不符 (0.232 m)", b, 0.232);

    PlateGeometry geo;
    Tracker id_tracker(geo);
    Measurement own;
    own.p_w = {1.0, 0.0, 0.2};
    own.theta = 0.0;
    own.target_id = 3;
    TrackerDiag id_diag;
    id_tracker.step(0.0, {own}, &id_diag);
    Measurement other = own;
    other.target_id = 4;
    id_tracker.step(0.04, {other}, &id_diag);
    const bool id_ok = id_diag.n_matched == 0 && id_diag.has_prediction;

    const double z[4] = {0.9, 0.0, 0.2, 0.0};
    const auto low_r = diagonalCovariance(.0001,.0001,.0001,.0001);
    const auto high_r = diagonalCovariance(1,1,1,1);
    const bool noise_ok = id_tracker.ekf().mahalanobis(0, z, 4, high_r.data())
                        < id_tracker.ekf().mahalanobis(0, z, 4, low_r.data());
    std::printf("目标类别门控: %s; 量测协方差权重: %s\n",
                id_ok ? "PASS" : "FAIL", noise_ok ? "PASS" : "FAIL");

    Tracker multi_tracker(geo);
    EkfState known;
    known.cx = 1.5;
    known.cy = 0.0;
    known.za = 0.2;
    known.theta = 0.0;
    known.r = geo.radius;
    Measurement small, big;
    small.p_w = plateWorldPosition(known, 0, geo);
    small.theta = plateThetaMeas(known, 0, geo);
    big.p_w = plateWorldPosition(known, 1, geo);
    big.theta = plateThetaMeas(known, 1, geo);
    big.is_big = true;
    multi_tracker.step(0.0, {small}, &id_diag);
    multi_tracker.step(0.04, {big, small}, &id_diag);
    const std::vector<Measurement> two_plates{big, small};
    std::array<bool, 4> sizes{{false, false, false, false}};
    for (int j = 0; j < 4; ++j)
    {
        const int i = id_diag.matched_meas_by_plate[j];
        if (i >= 0) sizes[j] = two_plates[i].is_big;
    }
    const bool multi_ok = id_diag.n_matched == 2 &&
        id_diag.matched_meas_by_plate[0] == 1 &&
        id_diag.matched_meas_by_plate[1] == 0 &&
        !sizes[0] && sizes[1];
    std::printf("双板关联与大小传递: %s\n", multi_ok ? "PASS" : "FAIL");

    const bool ok = a.ok && b.ok && id_ok && noise_ok && multi_ok;
    std::printf(ok ? "EKF + 跟踪器自检通过\n" : "自检失败\n");
    return ok ? 0 : 1;
}
