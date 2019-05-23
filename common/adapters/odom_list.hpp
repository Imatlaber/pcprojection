#ifndef ODOM_LIST_HPP
#define ODOM_LIST_HPP


#include <iostream>
#include <list>
#include <string>


struct HDTrans
{
    HDTrans()
    {
        rot_quat[0] = rot_quat[1] = rot_quat[2] = rot_quat[3] =  0.;
        trans_vec[0] = trans_vec[1] = trans_vec[2]= 0.;
    }
    double rot_quat[4] ;
    double trans_vec[3] ;
};

struct HDTimesTrans
{
    int64_t utime ;
    double rms_xyz;
    double rms_rpy;
    HDTrans hdtrans;
};

#endif // ODOM_LIST_HPP
