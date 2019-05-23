/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: wrapper of bot frames
 */

#ifndef _TRANS_HPP_
#define _TRANS_HPP_

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcmtypes/bot_frames_update_t.h>
#include <lcm/lcm.h>
#include <glib.h>
#include <iostream>
#include "param.hpp"

typedef struct {
    int frame_num;
    char * frame_name;
    char * relative_to;
    char * update_channel;
    bot_core_rigid_transform_t_subscription_t * transform_subscription;
    bot_core_pose_t_subscription_t * pose_subscription;
    BotCTransLink * ctrans_link;
    int was_updated;
} frame_handle_t;

struct _BotFrames {
    BotCTrans * ctrans;
    lcm_t *lcm;
    BotParam *bot_param;

    GMutex * mutex;
    int num_frames;
    char * root_name;
    GHashTable* frame_handles_by_name;
    GHashTable* frame_handles_by_channel;

    bot_frames_update_t_subscription_t * update_subscription;
    GList * update_callbacks;
};

class trans_t 
{
public:
    trans_t(lcm_t*, param_t*);

    ~trans_t();

    int get_local_pos(double pos[3], double* heading = NULL);
    int get_local_pose(bot_core_pose_t* pose);
    int vehicle_in_local(double pos[3]);
    inline bool getTf(std::string from_frame, std::string to_frame, int64_t utime, double *bot_tf)
	{
		if(!bot_frames_get_trans_mat_3x4_with_utime(frames, from_frame.c_str(),
    	                                            to_frame.c_str(), utime, bot_tf))
    	{
    	    std::cout<<"Warning: transform point failed to find frame from "<<from_frame<<"-->"<<to_frame<<std::endl;
    	    return false;
    	}
    	return true;
	}
    inline void transformPoint(double v[3], double bot_tf[12])
	{
		double result[3];
    	bot_vector_affine_transform_3x4_3d(bot_tf, v, result);
    	memcpy(v, result, sizeof(result));
	}

    inline void transformPoint(double v[3], const BotTrans * trans)
	{
		double result[3];
    	bot_trans_apply_vec(trans, v, result);
    	memcpy(v, result, sizeof(result));
	}

    BotFrames* frames;


public:
    static int curr_printfs_;
    const static int max_printfs_ = 25;

    static void on_pose(const lcm_recv_buf_t* rbuf, const char* channel,
                        const bot_core_pose_t* msg, void* user);
    static void on_frames_update(BotFrames* _frames, const char* frame,
                                 const char* relative_to, int64_t utime,
                                 void* user);
	static void updateTransform(BotFrames *bot_frame, BotTrans *trans, int64_t utime, std::string channel_name);

    lcm_t* lcm;
    param_t* param_;

    // save last pose here
    bot_core_pose_t_subscription_t* pose_subscription_;
    bot_core_pose_t last_pose_;
    int have_last_pose_;

    std::string channel_pose_;
};

#endif
