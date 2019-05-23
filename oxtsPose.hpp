/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: OxtsPose
 */
#ifndef _OXTSPOSE_HPP_
#define _OXTSPOSE_HPP_

class OxtsPose
{
public:
    OxtsPose(long long int t,
             double x,
             double y,
             double z)
        : time_(t),
          x_(x),
          y_(y),
          z_(z) {}

    double getx()
    {
        return x_;
    }

    double gety()
    {
        return y_;
    }

    double getz()
    {
        return z_;
    }

    long long getTime()
    {
        return time_;
    }

private:
    long long int time_;
    double x_;
    double y_;
    double z_;
};

#endif // _OXTSPOSE_HPP_