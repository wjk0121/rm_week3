#pragma once

//  第三周(整车状态估计)使用的轻量三维数学工具
//  不依赖 OpenCV / Eigen, 让 EKF 与坐标变换可以单独编译、单独自检。

#include <algorithm>
#include <array>
#include <cmath>

namespace rmv
{

constexpr double kPi = 3.14159265358979323846;

//  角度 <-> 弧度
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

//  把角度折回 (-pi, pi]; EKF 的角度残差必须走这里
inline double wrapAngle(double a)
{
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a < 0.0) a += 2.0 * kPi;
    return a - kPi;
}

struct Vec2
{
    double x = 0.0, y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

struct Vec3
{
    double x = 0.0, y = 0.0, z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 cross(const Vec3& o) const
    {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }

    double norm() const { return std::sqrt(dot(*this)); }

    Vec3 normalized() const
    {
        const double n = norm();
        return n > 1e-12 ? (*this) / n : Vec3();
    }
};

struct Mat3
{
    //  默认单位阵, 避免"看似合法但其实是零矩阵"的隐性问题
    double m[3][3] = {{1.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0},
                      {0.0, 0.0, 1.0}};

    static Mat3 identity() { return Mat3(); }

    static Mat3 fromArray(const double a[9])
    {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = a[i * 3 + j];
        return r;
    }

    void toArray(double a[9]) const
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                a[i * 3 + j] = m[i][j];
    }

    Vec3 operator*(const Vec3& v) const
    {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 operator*(const Mat3& o) const
    {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k)
                    sum += m[i][k] * o.m[k][j];
                r.m[i][j] = sum;
            }
        return r;
    }

    Mat3 transposed() const
    {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    //  与已手写代码兼容; 新代码统一用 transposed()
    Mat3 transpose() const { return transposed(); }

    Vec3 col(int i) const { return {m[0][i], m[1][i], m[2][i]}; }
};

//  四元数顺序固定为 (w, x, y, z), 与题目 IMU 文本一致
struct Quaternion
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_)
    {
    }

    double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quaternion normalized() const
    {
        const double n = norm();
        return n > 1e-12 ? Quaternion(w / n, x / n, y / n, z / n) : Quaternion();
    }

    //  单位四元数 -> 旋转矩阵(主动旋转, body -> world)
    Mat3 toMat3() const
    {
        const Quaternion q = normalized();
        const double ww = q.w * q.w, xx = q.x * q.x;
        const double yy = q.y * q.y, zz = q.z * q.z;

        Mat3 r;
        r.m[0][0] = ww + xx - yy - zz;
        r.m[0][1] = 2.0 * (q.x * q.y - q.w * q.z);
        r.m[0][2] = 2.0 * (q.x * q.z + q.w * q.y);
        r.m[1][0] = 2.0 * (q.x * q.y + q.w * q.z);
        r.m[1][1] = ww - xx + yy - zz;
        r.m[1][2] = 2.0 * (q.y * q.z - q.w * q.x);
        r.m[2][0] = 2.0 * (q.x * q.z - q.w * q.y);
        r.m[2][1] = 2.0 * (q.y * q.z + q.w * q.x);
        r.m[2][2] = ww - xx - yy + zz;
        return r;
    }
};

//  手册与后续代码统一使用短名 Quat; 保留 Quaternion 兼容已写代码
using Quat = Quaternion;

//  单位四元数球面线性插值, t in [0,1]
inline Quaternion slerp(const Quaternion& a, const Quaternion& b, double t)
{
    Quaternion q0 = a.normalized();
    Quaternion q1 = b.normalized();

    double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
    if (dot < 0.0)          //  取同一旋转的短路径
    {
        q1 = Quaternion(-q1.w, -q1.x, -q1.y, -q1.z);
        dot = -dot;
    }
    dot = std::min(1.0, std::max(-1.0, dot));

    if (dot > 0.9995)       //  两姿态几乎相同, 退化为线性插值避免除零
    {
        return Quaternion(q0.w + t * (q1.w - q0.w),
                          q0.x + t * (q1.x - q0.x),
                          q0.y + t * (q1.y - q0.y),
                          q0.z + t * (q1.z - q0.z)).normalized();
    }

    const double theta_0 = std::acos(dot);
    const double theta = theta_0 * t;
    const double s0 = std::sin(theta_0 - theta) / std::sin(theta_0);
    const double s1 = std::sin(theta) / std::sin(theta_0);
    return Quaternion(s0 * q0.w + s1 * q1.w,
                      s0 * q0.x + s1 * q1.x,
                      s0 * q0.y + s1 * q1.y,
                      s0 * q0.z + s1 * q1.z).normalized();
}

}   //  namespace rmv
