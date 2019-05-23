/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: misc helper functions
 */

#ifndef _MISC_HELPER_HPP_
#define _MISC_HELPER_HPP_

#include <assert.h>
#include <cmath>
#include <complex>
#include <iostream>
#include <type_traits>
#include <vector>
#include <bot_core/rotations.h>
#include <bot_core/trans.h>
#include <bot_frames/bot_frames.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

inline void db_array_2_fl_array(double * db, float * fl, size_t n)
{
	for(size_t i=0; i<n; ++i)
		fl[i] = (float)db[i] ;
}

inline void fl_array_2_db_array(float * fl, double * db, size_t n)
{
	for(size_t i=0; i<n; ++i)
		db[i] = (double)fl[i] ;
}

inline void ext_bot_trans_set_from_mat_4x4(BotTrans * dest, const double src[16])
{
	double quat[4] ;
	double rot[9] = {src[0], src[1], src[2],
					 src[4], src[5], src[6],
					 src[8], src[9], src[10]} ;
	bot_matrix_to_quat(rot, quat) ;
	double vec[3] = {src[3], src[7], src[11]} ;
	bot_trans_set_from_quat_trans(dest, quat, vec) ;
}

inline void ext_bot_trans_get_rpy_trans(const BotTrans * src, double rpy[3], double vec[3]) 
{
	bot_trans_get_trans_vec(src, vec) ;
	bot_quat_to_roll_pitch_yaw(src->rot_quat, rpy) ;	
}

Eigen::Matrix4d ext_bot_trans_to_matrix4d(const BotTrans * src)
{
    double trans_db[16] = {0.0} ;
    bot_trans_get_mat_4x4(src, trans_db) ;
    Eigen::Matrix4d trans(trans_db) ;
    trans.transposeInPlace() ;
    return trans ;
}













#endif
