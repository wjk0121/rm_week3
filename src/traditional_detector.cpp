#include "traditional_detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

TraditionalDetector::TraditionalDetector(const CameraParams& params,
                                         const NumberClassifier& clf)
    : params_(params), clf_(clf) {}

//  灯条长轴单位向量(由长边决定, 方向取旋转矩形第一条边的正方向)
static cv::Point2f barAxis(const cv::RotatedRect& r)
{
    cv::Point2f pts[4];
    r.points(pts);
    const cv::Point2f d1 = pts[1] - pts[0], d2 = pts[2] - pts[1];
    cv::Point2f axis = (cv::norm(d1) > cv::norm(d2)) ? d1 : d2;
    const float n = cv::norm(axis);
    return (n > 1e-6f) ? axis / n : cv::Point2f(0.f, 1.f);
}

//  取旋转矩形的两个长轴端点(灯条两端)
static std::pair<cv::Point2f, cv::Point2f> barEndpoints(const cv::RotatedRect& r)
{
    const cv::Point2f axis = barAxis(r);
    const float half = std::max(r.size.width, r.size.height) / 2.f;
    return {r.center - axis * half, r.center + axis * half};
}

//  四个角点是否构成凸四边形
//  交叉配对(两根斜杆呈 V/X 形)会产生自交的"蝴蝶结"四边形, 角点顺序再漂亮也是假的
static bool isConvexQuad(const std::vector<cv::Point2f>& q)
{
    if (q.size() != 4) return false;
    double sign = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f a = q[i], b = q[(i + 1) % 4], c = q[(i + 2) % 4];
        const double cross = static_cast<double>(b.x - a.x) * (c.y - b.y)
                           - static_cast<double>(b.y - a.y) * (c.x - b.x);
        if (std::abs(cross) < 1e-3) return false;          //  退化(三点共线)
        if (sign == 0.0) sign = cross;
        else if (cross * sign < 0.0) return false;         //  转向不一致 -> 非凸
    }
    return true;
}

//  数字区白色像素占比(Otsu 后)
//  真数字的笔画占比约 0.1~0.4; 全黑或全白说明这个 ROI 根本不是数字区(躯干/背景)
static float digitWhiteRatio(const cv::Mat& warp, int num_w)
{
    const int x0 = (warp.cols - num_w) / 2;
    if (x0 < 0) return 0.f;
    const cv::Mat roi = warp(cv::Rect(x0, 0, num_w, warp.rows));
    cv::Mat gray, bin;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    return static_cast<float>(cv::countNonZero(bin))
         / static_cast<float>(bin.total());
}

std::vector<cv::RotatedRect> TraditionalDetector::findLightBars(
        const cv::Mat& frame,int color_diff_thresh) const
{
    //  颜色判据: 敌方通道减去另外两个通道中的较大者(dominance)
    //  旧版只减 B(R-B), 皮肤/暖色高光的 R 同样远大于 B, 手臂会被当成红灯条;
    //  改取 R-max(G,B) 后, 皮肤因 G 接近 R 被压掉, 只有纯色发光体留得下来
    //  (实拍视频为橙红灯条 -> red; 仿真测试图为蓝灯条 -> blue)
    std::vector<cv::Mat> ch;
    cv::split(frame, ch);
    cv::Mat others, diff;
    if (params_.enemy_color == EnemyColor::RED)
    {
        cv::max(ch[1], ch[0], others);          //  max(G,B)
        cv::subtract(ch[2], others, diff);      //  R - max(G,B)
    }
    else
    {
        cv::max(ch[2], ch[1], others);          //  max(R,G)
        cv::subtract(ch[0], others, diff);      //  B - max(R,G)
    }

    //  饱和度/亮度门: 排除白墙、灰地、暗处暖色; 灯条是高饱和、高亮的发光体
    cv::Mat hsv, s_ok, v_ok, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hs;
    cv::split(hsv, hs);
    cv::threshold(hs[1], s_ok, kMinBarSat, 255, cv::THRESH_BINARY);
    cv::threshold(hs[2], v_ok, kMinBarVal, 255, cv::THRESH_BINARY);
    cv::bitwise_and(s_ok, v_ok, mask);
    cv::bitwise_and(diff, mask, diff);

    cv::threshold(diff, diff, color_diff_thresh, 255, cv::THRESH_BINARY);
    //  轻微膨胀黏合发光边缘的碎斑, 使灯条轮廓连续
    cv::dilate(diff, diff, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> bars;
    for (const auto& c : contours)
    {
        if (cv::contourArea(c) < kMinBarArea) continue;       //  滤噪点
        const cv::RotatedRect r = cv::minAreaRect(c);
        const float ls = std::max(r.size.width, r.size.height);
        const float ss = std::min(r.size.width, r.size.height);
        if (ss < 2.f) continue;                               //  排除细线
        if (ls / ss < kMinBarAspect) continue;                //  必须是细长条
        //  长度/宽度上限: 1080p 下真实灯条约 40~90px; 超过 0.14*rows 的"长条"
        //  是手臂、挂绳、门框一类背景结构(本次误检的根因之一)
        if (ls > kMaxBarLenRatio * frame.rows) continue;
        if (ss > kMaxBarWidthRatio * frame.rows) continue;
        //  填充率: 实心灯条约 0.8; 手臂高光/衣褶边缘是细弧线, 轮廓面积远小于外接矩形
        if (cv::contourArea(c) / (ls * ss) < kMinBarFill) continue;
        bars.push_back(r);
    }
    return bars;
}

std::vector<Armor> TraditionalDetector::pairLightBars(
        const std::vector<cv::RotatedRect>& bars, const cv::Mat& frame) const
{
    std::vector<Armor> armors;

    for (size_t i = 0; i < bars.size(); ++i)
    {
        for (size_t j = i + 1; j < bars.size(); ++j)
        {
            const auto [a1, b1] = barEndpoints(bars[i]);
            const auto [a2, b2] = barEndpoints(bars[j]);
            const float len1 = cv::norm(b1 - a1), len2 = cv::norm(b2 - a2);
            const cv::Point2f u1 = barAxis(bars[i]), u2 = barAxis(bars[j]);

            //  约束1: 长度接近(透视下同板两灯条像素长度近似)
            const float lr = std::min(len1, len2) / std::max(len1, len2);
            if (lr < 0.4f) continue;
            //  约束2: 轴向平行(带符号点积). 旧版比较 |角度| 差, 无法区分"平行"与
            //  "镜像成 V 字"; 人体手臂/挂绳恰好常呈 V 形交叉, 会骗过无符号检查
            if (std::abs(u1.dot(u2)) < kMinAxisParallel) continue;
            //  约束2b: 灯条必须接近竖直. 装甲板灯条在画面里总是立着的,
            //  横向条纹(桌沿/衣摆/手臂轮廓)不是灯条
            if (std::abs(u1.y) < kMinVerticalCos || std::abs(u2.y) < kMinVerticalCos)
                continue;
            //  约束2c: 两根灯条在纵向必须有足够的重叠高度
            const float top1 = std::min(a1.y, b1.y), bot1 = std::max(a1.y, b1.y);
            const float top2 = std::min(a2.y, b2.y), bot2 = std::max(a2.y, b2.y);
            const float min_h = std::min(bot1 - top1, bot2 - top2);
            if (min_h <= 1.f) continue;
            const float overlap = std::min(bot1, bot2) - std::max(top1, top2);
            if (overlap / min_h < kMinVerticalOverlap) continue;
            //  约束3: 中心距约为 1.0~4 倍灯条长(太近是同一根, 太远不相关)
            const cv::Point2f dc = bars[j].center - bars[i].center;
            const float dist = cv::norm(dc), avg_len = (len1 + len2) / 2.f;
            if (dist < kMinPairDistRatio * avg_len || dist > 4.f * avg_len) continue;
            //  约束4: 连线应近似垂直于灯条方向(两灯条左右排布)
            if (std::abs(dc.dot(cv::Point2f(b1.y - a1.y, a1.x - b1.x)))
                    / (dist * avg_len) < 0.3f) continue;

            //  左/右灯条按中心 m 坐标区分; 端点按 n 坐标分上下
            const bool i_left = bars[i].center.x < bars[j].center.x;
            const auto& le = i_left ? std::make_pair(a1, b1) : std::make_pair(a2, b2);
            const auto& re = i_left ? std::make_pair(a2, b2) : std::make_pair(a1, b1);
            const cv::Point2f lt = le.first.y < le.second.y ? le.first  : le.second;
            const cv::Point2f lb = le.first.y < le.second.y ? le.second : le.first;
            const cv::Point2f rt = re.first.y < re.second.y ? re.first  : re.second;
            const cv::Point2f rb = re.first.y < re.second.y ? re.second : re.first;

            //  约束5: 四角点必须构成凸四边形(挡住交叉杆拼出的蝴蝶结)
            const std::vector<cv::Point2f> quad = {lt, rt, rb, lb};   //  契约序
            if (!isConvexQuad(quad)) continue;

            //  数字区透视矫正: 契约序角点 -> 32x28 竖直版式
            //  灯条纵向映射到两侧 y∈[7,19] 的短段, 数字居中(已实测验证的几何)
            const std::vector<cv::Point2f> dst = {{0, 7}, {31, 7}, {31, 19}, {0, 19}};
            const cv::Mat M = cv::getPerspectiveTransform(quad, dst);
            cv::Mat warp;
            cv::warpPerspective(frame, warp, M, {32, 28});

            //  约束6: 数字区必须真的像数字(白色占比在合理区间)
            //  躯干/背景的 warp 经 Otsu 后常全黑或全白, 提前否决, 省掉 49 次推理
            const float white_ratio = digitWhiteRatio(warp, NumberClassifier::numberWidth());
            if (white_ratio < kMinDigitWhiteRatio || white_ratio > kMaxDigitWhiteRatio)
                continue;

            //  数字网络辅助配对: negative 或低置信度 -> 否决这对灯条
            //  TTA: 远距离/大斜视时灯条端点估计有 ±3px 抖动, 且数字区在 warp 后
            //  相对 ROI 存在 ~2px 的系统偏移; mlp 对该偏移敏感(同一张数字, 平移
            //  相位不同判定可完全相反), 故在 [-3,3]^2 的整数格点上遍历全部相位。
            //  旧版取"全部相位里的最大置信度"——49 次独立尝试取最大值, 垃圾 ROI 也
            //  有很高概率碰出一次 0.7+ (多重比较), 这是 outpost 0.73 虚警的来源。
            //  改为"投票": 只有同一类别在多个相位上稳定被判出, 才认为配对成立
            const int nlabels = static_cast<int>(clf_.numLabels());
            std::vector<int> votes(nlabels, 0);
            std::vector<float> conf_max(nlabels, 0.f);
            for (int dy = -3; dy <= 3; ++dy)
            {
                for (int dx = -3; dx <= 3; ++dx)
                {
                    cv::Mat probe = warp.clone();
                    if (dx != 0 || dy != 0)
                    {
                        const cv::Mat shift = (cv::Mat_<double>(2, 3)
                                               << 1, 0, dx, 0, 1, dy);
                        cv::warpAffine(warp, probe, shift, warp.size(),
                                       cv::INTER_LINEAR, cv::BORDER_REPLICATE);
                    }
                    const auto [idx, conf] = clf_.classify(probe);
                    if (idx < 0) continue;
                    if (idx == nlabels - 1) continue;                  //  negative
                    if (conf < kPairConfThresh) continue;
                    ++votes[idx];
                    conf_max[idx] = std::max(conf_max[idx], conf);
                }
            }

            //  取票数最多的类别; 票数不足说明各相位判断互相矛盾, 大概率不是装甲板
            int best_idx = -1;
            for (int c = 0; c < nlabels - 1; ++c)
                if (best_idx < 0 || votes[c] > votes[best_idx]) best_idx = c;
            if (best_idx < 0 || votes[best_idx] < kMinSameClassVotes) continue;
            const float best_conf = conf_max[best_idx];
            if (best_conf < kPairConfThresh) continue;

            Armor armor;
            armor.corners = quad;                 //  契约: 左上、右上、右下、左下
            armor.number = best_idx;
            armor.confidence = best_conf;
            armor.label = clf_.label(best_idx);
            //  尺寸启发: 1号(英雄)、outpost(前哨站)、base(基地)为大装甲板
            armor.size = (armor.label == "1" || armor.label == "outpost" ||
                          armor.label == "base")
                         ? ArmorSize::BIG : ArmorSize::SMALL;
            //  板型一致的距比上限: 小板(135x55)的 中心距/杆长 最大为正面
            //  135/55≈2.45, 斜视只按 cos 压缩不会更大; 取 10% 像素测量余量 2.7。
            //  超过 2.7 的对物理上不可能是小板 —— 属跨板误配对或背景结构,
            //  其数字判定必为误读; 大板(225/55≈4.09)沿用配对阶段的 4.0 上限
            if (armor.size != ArmorSize::BIG && dist > kMaxPairRatioSmall * avg_len)
                continue;
            armors.push_back(armor);
        }
    }
    return armors;
}

std::vector<Armor> TraditionalDetector::detect(const cv::Mat& frame) const
{
    if (frame.empty()) return {};
    std::vector<cv::RotatedRect> bars = findLightBars(frame, kColorDiffThresh);
    for (const auto& weak : findLightBars(frame, kColorDiffThreshLow))
    {
        //  弱召回的灯条若与已有灯条重叠, 则忽略(避免重复配对)
        bool dup = false;
        for (const auto& b : bars)
        {
            const float r=0.5f*std::max(b.size.width, b.size.height);
            if(std::abs(weak.center.x - b.center.x) < r && std::abs(weak.center.y - b.center.y) < r)
            {
                dup = true;
                break;
            }

        }
        if (!dup) bars.push_back(weak);
    }
    return pairLightBars(bars, frame);
}
