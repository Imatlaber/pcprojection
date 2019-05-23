/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: math helper functions
 */

#ifndef _MATH_HELPER_HPP_
#define _MATH_HELPER_HPP_

#include <assert.h>
#include <cmath>
#include <complex>
#include <iostream>
#include <type_traits>
#include <vector>
#include <bot_core/rotations.h>
#include <bot_core/trans.h>
#include <bot_frames/bot_frames.h>

// use STL functions
using std::abs;
using std::atan2;
using std::cos;
using std::exp;
using std::hypot;
using std::log;
using std::max;
using std::min;
using std::sin;
using std::sqrt;
using std::swap;
using std::tan;

/**
* PI = pi
*/
constexpr double PI = 3.141592653589793238462643383279502884197169;

/**
* PI_HALF = pi * 0.5
*/
constexpr double PI_HALF = 1.570796326794896619231321691639751442098585;

/**
* TWO_PI = 2 * pi
*/
// TODO (need Francis for that) try to cast the rhs to double(...) and then make obsolete the other casts that use TWO_PI
constexpr double TWO_PI = 6.283185307179586476925286766559005768394339;


/**
 * TWO_PI_INV = 1.0 / (2.0 * pi)
 */
constexpr double TWO_PI_INV = 1.0 / TWO_PI;

/**
* E = exp(1)
*/
constexpr double E = 2.71828182845904523536;

/**
* SQRT_TWO = sqrt(2)
*/
constexpr double SQRT_TWO = 1.41421356237309504880;


// Suppliers

/**
* returns 0
*/
template<class type>
inline type zero()
{
    return 0;
}

/**
* returns 1
*/
template<class type>
inline type one()
{
    return 1;
}

// Functions

/**
* returns input
*/
template<class type>
inline type identity(const type& a)
{
    return a;
}


/**
* Mathematica default tolerance for chop is 1e-10;
* Mathematica: Chop
*/
template<class type>
inline type getChopThreshold(const type& a)
{
    constexpr double CHOP_FLOAT = 1E-6;
    constexpr double CHOP_DOUBLE = 1E-10;
    return sizeof(a) <= 4 ? CHOP_FLOAT : CHOP_DOUBLE;
}


/**
* function modeled after Mathematica function "Chop"
*/
template<class type>
inline type chop(const type& a)
{
    return abs(a) < getChopThreshold(a) ? 0 : a;
}


/**
* returns a * a
*/
template<class type>
inline type square(const type& a)
{
    return a * a;
}


/**
* returns the angle between a vector [a, b] and x-axis in radians
* both arguments are const type&
* return value is between [0, TWO_PI]
*/
template<class type>
inline type atan2_pos(const type& a, const type& b)
{
    type ret = atan2(a, b);
    if (ret < 0.0)
        return ret + TWO_PI;

    return ret;
}


// BiFunctions

/**
* returns sum a + b
*/
template<class type>
inline type sum(const type& a, const type& b)
{
    return a + b;
}


/**
* returns product a * b
*/
template<class type>
inline type product(const type& a, const type& b)
{
    return a * b;
}


/**
* return value is non-negative
*/
inline int mod(const int& val, const int& mod)
{
    assert(0 < mod);
    int ret = val % mod;
    return ret < 0 ? ret + mod : ret;
}


/**
* return value is non-negative
*/
inline long mod(const long& val, const long& mod)
{
    assert(0 < mod);
    int ret = val % mod;
    return ret < 0 ? ret + mod : ret;
}


/**
* return value is non-negative
*/
template<class type>
inline type mod(const type& val, const type& mod)
{
    assert(0 < mod);
    type res = std::fmod(val, mod);
    return res < 0 ? res + mod : res;
}


/**
* sign function that returns {-1, 0, 1}
* https://en.wikipedia.org/wiki/Sign_function
*/
template<class type>
inline type sign(const type& a)
{
    return 0 < a ? 1 : (a < 0 ? -1 : 0);
}


/**
* signum function that returns +1 for non-negative input and -1 else
*/
template<class type>
inline type binarySign(const type& a)
{
    return 0 <= a ? 1 : -1;
}


/**
* SIGN(A,B) returns the value of A with the sign of B.
* https://gcc.gnu.org/onlinedocs/gfortran/SIGN.html
*/
template<class type>
inline type fortranSign(const type& a, const type& b)
{
    return 0 <= b ? (0 <= a ? a : -a) : (0 <= a ? -a : a);
}


/**
* maps num into interval [lo, hi]
* attention to the parameter ordering (!)
*/
template<class type>
inline type clamp(const type& lo, const type& num, const type& hi)
{
    assert(lo <= hi);
    return min(max(lo, num), hi);
}


/**
* principalValue(angle, min) maps angle into interval [min, min + 2*PI)
* the implementation is only intended to be used for "small" values of angle, i.e.
* |angle| < 1e2
*
* typically, min == -PI, or min == 0.
*
* if min is not specified the value min == -PI is taken
*/
template<class type>
inline type principalValue(const type& angle, const double& min = -PI)
{
    static_assert(std::is_floating_point<type>::value, "principalValue only for floating point type");
    assert(-1e2 < angle && angle < 1e2);
    type lhs = angle;
    while (lhs < min)
        lhs += double(TWO_PI);

    while (min + double(TWO_PI) <= lhs)
        lhs -= double(TWO_PI);

    return lhs;
}


/**
* function principalValueUsingMod() has constant runtime
* and the result is within 1e-15 for "small" angle values
* when compared to principalValue()
*/
template<class type>
inline type principalValueUsingMod(const type& angle, const double& min = -PI)
{
    static_assert(std::is_floating_point<type>::value, "principalValueUsingMod only for floating point type");
    // - min compensates the later addition of min
    // mod maps the result into the interval [0, 2*PI)
    // + min maps the result into the interval [min, min + 2*PI)
    return mod(angle - min, double(TWO_PI)) + min;
}

//reference: https://en.wikipedia.org/wiki/Weighted_arithmetic_mean

template<class type>
static void meanStddevWeighted(const std::vector<std::pair<type, double>>& values_weighted, type& mean, type& stddev)
{
    type value_sum = 0.0;
    double total_weight = 0.0;
    for (std::size_t i = 0; i < values_weighted.size(); i++)
    {
        value_sum += values_weighted[i].first * values_weighted[i].second;
        total_weight += values_weighted[i].second;        
    }
    mean = value_sum / total_weight;
    type total_var = 0.0;
    for (std::size_t i = 0; i < values_weighted.size(); i++)
    {
        double diff_tmp = values_weighted[i].first- mean;
        total_var += diff_tmp*diff_tmp*values_weighted[i].second;
        //std::cout<<total_var<<":"<<diff_tmp<<","<<values_weighted[i].second<<std::endl;
    }
    stddev = std::sqrt(total_var/total_weight);

    //std::cout<<"value_sum & total_weight & mean & total_var & stddev:"<<value_sum<<","<<total_weight<<","<<mean<<","<<total_var<<","<<stddev<<std::endl;
}

// Square the input. Prefer nutils::Math::square.
template <class Type>
inline Type SQ(const Type& x)
{
    return x * x;
}


template<class T>
inline T cross(const T& p, const T& q){
    assert(p.size() ==3 && q.size() == 3);

    return T(p[1]*q[2]-p[2]*q[1],
             p[2]*q[0]-p[0]*q[2],
             p[0]*q[1]-p[1]*q[0]);
}

template<class T>
inline void cross(const T* p, const T* q, T* s)
{
	s[0] = p[1]*q[2]-p[2]*q[1] ;
	s[1] = p[2]*q[0]-p[0]*q[2] ;
	s[2] = p[0]*q[1]-p[1]*q[0] ;
}

template<class T>
inline T diff(const T& p, const T& q){
    assert(p.size()==q.size());
    T out = p;
    for(int i =0; i< p.size(); i++){
        out[i] = p[i]-q[i];
    }
    return out;
}

template<class T>
inline double dotprod3d(const T* p, const T* q)
{
	return p[0]*q[0]+p[1]*q[1]+p[2]*q[2] ;
}

template<class T>
inline double norm3d(const T* p)
{
    double out = 0. ;
    for(int i=0; i<3; ++i)
        out += p[i]*p[i] ;
    return sqrt(out) ;
}

template<class T>
inline double angleBtwVectors3d(const T* p,  const T* q)
{
    /// angle = arccos(dotprod(p,q)/norm(p)/norm(q))
    double dotp = dotprod3d(p, q) ;
    double normp = norm3d(p) ;
    double normq = norm3d(q) ;
    double cosangle = dotp/normp/normq ;
    cosangle = clamp(-1.0, cosangle, 1.0) ;
    return acos(cosangle) ;
}

// sqrt(dotprod(p,p))
template<class T>
inline double norm(const T& p){
    double out = 0;
    for(auto c:p){
        out += c*c;
    }
    return sqrt(out);
}

template<class T>
inline double norm3d(const T& p){
    double out = 0;
    for(int i=0; i<3; ++i)
    {
        out += p[i]*p[i] ;
    }
    return sqrt(out);
}

template<class T>
inline T normalize(const T& p){
    double n = std::max(1e-6, norm(p));
    T out = p;
    for(auto& c:out){
        c/=n;
    }
    return out;
}

template<class T>
inline T normalize3d(const T& p){
    double n = std::max(1e-6, norm3d(p));
    T out = p;
    for(int i=0; i<3; ++i)
    {
        out[i] /= n ;
    }
    return out;
}

template<class T>
inline int sign(const T& num){
    return (num==0)?0:(num>0)?1:-1;
}

template<class T>
inline double sinc(const T& num){
    return (num==0)?1.0:sin(num)/num;
}

// TODO use functions in math/Conversion.hpp instead of templates below

template<class type>
double deg_to_radians(const type& x) {
  return (x) * (PI / 180.0);
}

template<class type>
double rad_to_degrees(const type& x) {
  return (x) * (180.0 / PI);
}

template<class type>
double rpm_to_rad_per_sec(const type& x) {
  return ((x)*2.0 * PI / 60.0);
}

template<class type>
double rad_per_sec_to_rpm(const type& x) {
  return ((x) / 2.0 / PI * 60.0);
}

// TODO function name too short; remark that sgn(0)==1 instead of sgn(0)==0 
double sgn(double v);

/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

/** valid only for v > 0 **/
double mod2pi_positive(double vin);


/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

/** Map v to [-PI, PI] **/
// TODO use return atan2(sin(vin), cos(vin)); [code from amcl.cpp]
double mod2pi(double vin);

/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

// TODO linear algebra code below should go into separate file/class
double norm(const double* const arr, int ndim=3);

// assumes that r is allocated and deleted by the parent
void diff(double* r, const double* const p , const double* const q, int ndim=3);

// TODO function computes norm of (p-q) thus function name is not intuitive => rename
double norm(const double* const p, const double* const q, int ndim=3);
double yawFromMat3x4(const double mat[12]);
double yawFromMat4x4(const double mat[]);
double yawFromQuad(const double rot_quad[4]);
void rotFromMat3x4(double mat[12], double rot_quat[4]);
void RPYTransToMat4x4(double rpy[3], double pos[3], double mat[16]);
void YawTransToMat4x4(double yaw, double pos[3], double mat[16]);
void transform_position(BotTrans * trans, const double * src, double * dst) ;
void transform_position(BotFrames * frames, const char* from_frame, const char* to_frame, int64_t utime, const double * src, double * dst) ;
void transform_velocity(BotTrans * trans, const double * src, double * dst) ;
void transform_velocity(BotFrames * frames, const char* from_frame, const char* to_frame, int64_t utime, const double * src, double * dst) ;
void transform_velocity_by_yaw(double yaw, const double * src, double * dst) ;
int szudzik_map(int a, int b) ;


#endif
