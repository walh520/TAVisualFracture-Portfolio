#pragma once
#include "TVFExport.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tvf {
constexpr double Pi = 3.14159265358979323846;
struct Vec3 {
    double x=0,y=0,z=0;
    Vec3()=default;
    constexpr Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
    Vec3 operator+(Vec3 b)const{return {x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(Vec3 b)const{return {x-b.x,y-b.y,z-b.z};}
    Vec3 operator-()const{return {-x,-y,-z};}
    Vec3 operator*(double s)const{return {x*s,y*s,z*s};}
    Vec3 operator/(double s)const{return {x/s,y/s,z/s};}
    Vec3& operator+=(Vec3 b){x+=b.x;y+=b.y;z+=b.z;return *this;}
    Vec3& operator-=(Vec3 b){x-=b.x;y-=b.y;z-=b.z;return *this;}
};
inline Vec3 operator*(double s,Vec3 a){return a*s;}
inline double Dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 Cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double LengthSquared(Vec3 a){return Dot(a,a);}
inline double Length(Vec3 a){return std::sqrt(LengthSquared(a));}
inline Vec3 Normalize(Vec3 a,Vec3 fallback={0,0,1}){double l=Length(a);return l>1e-15?a/l:fallback;}
inline bool Finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline Vec3 Min(Vec3 a,Vec3 b){return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};}
inline Vec3 Max(Vec3 a,Vec3 b){return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}
struct Bounds {Vec3 min,max;Vec3 Size()const{return max-min;}Vec3 Center()const{return (min+max)*0.5;}};
struct Metric {
    Vec3 axis={0,0,1};
    double axisScale=1; // >1 long along axis; <1 slab normal to axis. Dimensionless.
    Vec3 Apply(Vec3 x)const {Vec3 a=Normalize(axis);return x+a*(Dot(a,x)*(1.0/(axisScale*axisScale)-1.0));}
    Vec3 Whiten(Vec3 x)const {Vec3 a=Normalize(axis);return x+a*(Dot(a,x)*(1.0/axisScale-1.0));}
    double DistanceSquared(Vec3 x)const{return Dot(x,Apply(x));}
};
inline uint32_t Hash(uint32_t x){x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;x^=x>>16;return x;}
struct Random {uint32_t state;explicit Random(uint32_t seed):state(seed){}uint32_t Next(){state+=0x9e3779b9u;return Hash(state);}double Unit(){return double(Next()>>8)/16777216.0;}};
}
