#pragma once
#include <algorithm>
#include <cmath>
namespace rmv::small {
// Cholesky on a normalized correlation matrix avoids metre/radian scaling issues.
inline bool inverseSpd(const double* a,int n,double* inverse) {
    if(n<1 || n>16) return false;
    double scale[16],l[256]={},y[16],x[16];
    for(int i=0;i<n;++i) {
        if(!std::isfinite(a[i*n+i]) || a[i*n+i]<=0) return false;
        scale[i]=std::sqrt(a[i*n+i]);
    }
    for(int i=0;i<n;++i) for(int j=0;j<=i;++j) {
        if(!std::isfinite(a[i*n+j]) || !std::isfinite(a[j*n+i])) return false;
        if(std::abs(a[i*n+j]-a[j*n+i])>1e-8*scale[i]*scale[j]) return false;
        double s=.5*(a[i*n+j]+a[j*n+i])/(scale[i]*scale[j]);
        for(int k=0;k<j;++k) s-=l[i*n+k]*l[j*n+k];
        if(i==j) { if(s<=1e-13) return false; l[i*n+j]=std::sqrt(s); }
        else l[i*n+j]=s/l[j*n+j];
    }
    for(int c=0;c<n;++c) {
        for(int i=0;i<n;++i) {
            double s=(i==c?1.:0.);
            for(int k=0;k<i;++k) s-=l[i*n+k]*y[k];
            y[i]=s/l[i*n+i];
        }
        for(int i=n-1;i>=0;--i) {
            double s=y[i];
            for(int k=i+1;k<n;++k) s-=l[k*n+i]*x[k];
            x[i]=s/l[i*n+i]; inverse[i*n+c]=x[i]/(scale[i]*scale[c]);
        }
    }
    return true;
}
inline double chiSquareGate(int dof) {
    constexpr double gates[17]={0,10.828,13.816,16.266,18.467,20.515,22.458,24.322,
        26.124,27.877,29.588,31.264,32.909,34.528,36.123,37.697,39.252};
    return gates[std::clamp(dof,1,16)]; // 99.9% upper quantile
}
}
