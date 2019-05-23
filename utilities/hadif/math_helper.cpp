/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: math helper functions
 */

#include "math_helper.hpp"

double yawFromMat3x4(const double mat[12])
{
    return yawFromMat4x4(mat);
}

double yawFromMat4x4(const double mat[])
{
    double rot_matrix[9] = {mat[0], mat[1], mat[2],
                            mat[4], mat[5], mat[6],
                            mat[8], mat[9], mat[10]};
    double rot_quat[4];
    bot_matrix_to_quat(rot_matrix, rot_quat);

    return yawFromQuad(rot_quat);
}

double yawFromQuad(const double rot_quad[4])
{
    double rot_rpy[3];
    bot_quat_to_roll_pitch_yaw(rot_quad, rot_rpy);

    return rot_rpy[2];
}

void rotFromMat3x4(double mat[12], double rot_quat[4])
{
    double rot_matrix[9] = {mat[0], mat[1], mat[2],
                            mat[4], mat[5], mat[6],
                            mat[8], mat[9], mat[10]};
    bot_matrix_to_quat(rot_matrix, rot_quat);
}

void RPYTransToMat4x4(double rpy[3], double pos[3], double mat[16])
{
    double quat[4];
    bot_roll_pitch_yaw_to_quat(rpy, quat);
    bot_quat_pos_to_matrix(quat, pos, mat);
}

void YawTransToMat4x4(double yaw, double pos[3], double mat[16])
{
    double rpy[3] = {0.0, 0.0, yaw};
    RPYTransToMat4x4(rpy, pos, mat);
}

void transform_position(BotTrans * trans, const double * src, double * dst)
{
     bot_trans_apply_vec(trans, src, dst) ;
}

void transform_position(BotFrames * frames, const char* from_frame, const char* to_frame, int64_t utime, const double * src, double * dst)
{
     BotTrans trans ;
     int status = bot_frames_get_trans_with_utime(frames, from_frame, to_frame, utime, &trans) ;
     if(!status)
     {
         printf("failed to transform from %s to %s frame\n", from_frame, to_frame) ;
         bot_trans_set_identity(&trans) ;
     }   
     transform_position(&trans, src, dst) ;
}

void transform_velocity(BotTrans * trans, const double * src, double * dst)
{
     bot_trans_rotate_vec(trans, src, dst) ;
}

void transform_velocity(BotFrames * frames, const char* from_frame, const char* to_frame, int64_t utime, const double * src, double * dst)
{
     BotTrans trans ;
     int status = bot_frames_get_trans_with_utime(frames, from_frame, to_frame, utime, &trans) ;
     if(!status)
     {
         printf("failed to transform from %s to %s frame\n", from_frame, to_frame) ;
         bot_trans_set_identity(&trans) ;
     } 
     transform_velocity(&trans, src, dst) ;
}

void transform_velocity_by_yaw(double yaw, const double * src, double * dst)
{
    double quat[4] = {0.0}, rpy[3] = {0.0, 0.0, yaw}, vec[3]={0.0} ;
    bot_roll_pitch_yaw_to_quat(rpy, quat) ;
    BotTrans trans ;
    bot_trans_set_from_quat_trans(&trans, quat, vec) ;
    bot_trans_rotate_vec(&trans, src, dst) ;
}

// TODO function name too short; remark that sgn(0)==1 instead of sgn(0)==0 
double sgn(double v) { return (v >= 0) ? 1 : -1; }

/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

/** valid only for v > 0 **/
double mod2pi_positive(double vin) {
    double q = vin * TWO_PI_INV + 0.5;
    int qi = (int)q;

    return vin - qi * TWO_PI;
}

/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

/** Map v to [-PI, PI] **/
// TODO use return atan2(sin(vin), cos(vin)); [code from amcl.cpp]
double mod2pi(double vin) {
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

/**
 * code adapted from
 * libbot/bot2-core/src/bot_core/math_util.h
 */

// TODO linear algebra code below should go into separate file/class
double norm(const double* const arr, int ndim /*=3*/)
{
    double sum = 0;
    for(int i=0; i<ndim; i++)
        sum += SQ(arr[i]);
    return sqrt(sum);
}


// assumes that r is allocated and deleted by the parent
void diff(double* r, const double* const p , const double* const q, int ndim /*=3*/)
{
    for(int i=0; i<ndim; i++)
        r[i] = p[i] - q[i];
}

// TODO function computes norm of (p-q) thus function name is not intuitive => rename
double norm(const double* const p, const double* const q, int ndim /*=3*/)
{
    double* t1 = new double[ndim];
    diff(t1, p, q, ndim);

    double t2 = norm(t1, ndim);
    delete[] t1;
    return t2;
}

/**
 * map from Z x Z -> Z
 *
 * references:
 * "an elegant pairing function" by Mathew Szudzik
 * http://stackoverflow.com/questions/919612/mapping-two-integers-to-one-in-a-unique-and-deterministic-way
 */
int szudzik_map(int a, int b) 
{
    int A = 0 <= a ? 2 * a : -2 * a - 1;
    int B = 0 <= b ? 2 * b : -2 * b - 1;
    int C = (B <= A ? A * A + A + B : A + B * B) / 2;
    return (a < 0 && b < 0) || (0 <= a && 0 <= b) ? C : -C - 1;
}

