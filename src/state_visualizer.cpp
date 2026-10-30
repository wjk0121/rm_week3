#include "state_visualizer.hpp"

#include "measurement.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace rmv
{

std::array<cv::Point3d, 4> plateCornersWorld(const EkfState& s, int j,
                                             const PlateGeometry& g,
                                             double width_m, double height_m)
{
    const double a = s.theta + deg2rad(g.phase_deg[j]);

    //  板中心
    const Vec3 c_b = plateWorldPosition(s, j, g);

    //  板坐标系:Z 指向车心(内法线),Y 向下,X = Y x Z
    const Mat3 rotation=plateRotationWorld(a,deg2rad(g.inward_pitch_deg));
    const Vec3 X=rotation.col(0),Yu=rotation.col(1);

    const double hw = width_m / 2.0;
    const double hh = height_m / 2.0;

    //  与 second week 的物体点顺序一致:左上、右上、右下、左下(Y 向下)
    std::array<cv::Point3d, 4> out;
    const Vec3 tl = c_b - X * hw - Yu * hh;
    const Vec3 tr = c_b + X * hw - Yu * hh;
    const Vec3 br = c_b + X * hw + Yu * hh;
    const Vec3 bl = c_b - X * hw + Yu * hh;
    out[0] = cv::Point3d(tl.x, tl.y, tl.z);
    out[1] = cv::Point3d(tr.x, tr.y, tr.z);
    out[2] = cv::Point3d(br.x, br.y, br.z);
    out[3] = cv::Point3d(bl.x, bl.y, bl.z);
    return out;
}

void drawPlates(cv::Mat& img, const WorldFrame& wf, double t, const EkfState& s,
                const PlateGeometry& g, const std::array<bool, 4>& is_big,
                double small_w_m, double small_h_m, double big_w_m, double big_h_m,
                int highlight)
{
    //  四色严禁与检测层撞色: 绿=检测四边形, 黄=角点与文字, 两者都留给第二周的可视化
    static const cv::Scalar colors[4] = {
        cv::Scalar(255, 255, 255),  //  白   0号槽
        cv::Scalar(255, 128, 0),    //  蓝   1号槽
        cv::Scalar(0, 128, 255),    //  橙   2号槽
        cv::Scalar(255, 0, 255)};   //  品红 3号槽

    for (int j = 0; j < 4; ++j)
    {
        const double w = is_big[j] ? big_w_m : small_w_m;
        const double h = is_big[j] ? big_h_m : small_h_m;

        const auto corners = plateCornersWorld(s, j, g, w, h);

        std::vector<cv::Point> pts;
        bool behind = false;
        for (const auto& c : corners)
        {
            const Vec3 p_w(c.x, c.y, c.z);
            if (wf.world_to_cam(t, p_w).z <= 0.05)    //  在相机背后就不画
            {
                behind = true;
                break;
            }
            const Vec2 uv = wf.project(t, p_w);
            pts.emplace_back((int)std::lround(uv.x), (int)std::lround(uv.y));
        }
        if (behind || pts.size() != 4) continue;

        const int thick = (highlight == j) ? 3 : 1;
        const cv::Scalar color = colors[j];
        //  先描黑边再画彩色,深色背景上才看得清
        cv::polylines(img, pts, true, cv::Scalar(0, 0, 0), thick + 2, cv::LINE_AA);
        cv::polylines(img, pts, true, color, thick, cv::LINE_AA);
        cv::putText(img, std::to_string(j), pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    color, 2, cv::LINE_AA);
    }
}

void drawStateText(cv::Mat& img, const EkfState& s, int frame_id, int matched_j,
                   int n_meas)
{
    char buf[128];
    const int x = std::max(10, img.cols - 330);
    int y = 26;

    auto put = [&](const char* text) {
        cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        y += 26;
    };

    std::snprintf(buf, sizeof(buf), "cx %+.3f  vx %+.3f", s.cx, s.vx);
    put(buf);
    std::snprintf(buf, sizeof(buf), "cy %+.3f  vy %+.3f", s.cy, s.vy);
    put(buf);
    std::snprintf(buf, sizeof(buf), "za %+.3f  vz %+.3f", s.za, s.vz);
    put(buf);
    std::snprintf(buf, sizeof(buf), "th %+.1f  w %+.2f", rad2deg(wrapAngle(s.theta)),
                  s.omega);
    put(buf);
    std::snprintf(buf, sizeof(buf), "r  %.3f", s.r);
    put(buf);
    std::snprintf(buf, sizeof(buf), "dr %+.3f dh %+.3f",s.dr,s.dh);
    put(buf);
    //  没有融合成功的帧,四块板全是按状态外推出来的,必须标出来;
    //  否则"预测"会被误读成"检测框标错了位置"
    if (matched_j < 0)
        std::snprintf(buf, sizeof(buf), "f%d  pred  n%d", frame_id, n_meas);
    else
        std::snprintf(buf, sizeof(buf), "f%d  plate%d  n%d", frame_id, matched_j, n_meas);
    put(buf);
}

}   //  namespace rmv
