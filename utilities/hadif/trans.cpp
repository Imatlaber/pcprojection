/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: wrapper of bot frames
 */

#include "trans.hpp"
#include "assert.h"

int trans_t::curr_printfs_;
trans_t::trans_t(lcm_t* _lcm, param_t* _param) {
    lcm = _lcm;
    param_ = _param;

    param_->getParam("channel.pose", channel_pose_, std::string("POSE"));

    have_last_pose_ = 0;
    pose_subscription_ =
        bot_core_pose_t_subscribe(lcm, channel_pose_.c_str(), trans_t::on_pose, this);

    // we won't need param and frames in every instantiation of nutrans
    if (param_) {
        frames = bot_frames_get_global(lcm, param_->getBotParam());
        bot_frames_add_update_subscriber(frames, trans_t::on_frames_update,
                                         this);
    } else {
        frames = NULL;
    }
}

trans_t::~trans_t() {
    //if (frames) bot_frames_destroy(frames);
    if (pose_subscription_) bot_core_pose_t_unsubscribe(lcm, pose_subscription_);
}

void trans_t::on_pose(const lcm_recv_buf_t* rbuf, const char* channel,
                        const bot_core_pose_t* msg, void* user)
{
    if(nullptr == rbuf || nullptr == channel || nullptr == msg || nullptr == user)
        return;

    trans_t* self = (trans_t*)user;
    self->last_pose_ = *msg;
    self->have_last_pose_ = 1;
}

/**
 * get vehicle positions, i.e., [0,0,0] in body frame
 * in local frame
 * Returns 1 if success, 0 if failure
 */
int trans_t::vehicle_in_local(double pos[3]) { 
    return get_local_pos(pos);
}

void trans_t::on_frames_update(BotFrames* /*_frames*/, const char* /*frame*/,
                                 const char* /*relative_to*/, int64_t /*utime*/,
                                 void* /*user*/) {
    // printf("link %s->%s updated, user=%p\n", frame, relative_to, user);
}

/**
 * get the local (x,y,z) position along with heading
 */
int trans_t::get_local_pos(double pos[3], double* heading) {
    assert(frames != NULL);

    BotTrans bt;
    if (!bot_frames_get_trans(frames, "body", "local", &bt)) {
        printf("get_local_pos() did not get body to local transform\n");
        return 0;
    }
    // bot_trans_print_trans(&bt);
    bot_trans_get_trans_vec(&bt, pos);

    if (heading) {
        double rpy[3];
        bot_quat_to_roll_pitch_yaw(bt.rot_quat, rpy);
        *heading = rpy[2];
    }
    return 1;
}

int trans_t::get_local_pose(bot_core_pose_t* pose) {
    if (have_last_pose_) {
        memcpy(pose, &last_pose_, sizeof(bot_core_pose_t));
        // printf("pose: pos-(%.3f,%.3f,%.3f)\n", pose->pos[0], pose->pos[1],
        // pose->pos[2]);
        // printf("pose: last_pose-(%.3f,%.3f,%.3f)\n", last_pose.pos[0],
        // last_pose.pos[1], last_pose.pos[2]);
        return 1;
    } else {
            printf("get_local_pose(): does not have pose, is POSE message being "
                "published?\n");
        return 0;
    }
}

void trans_t::updateTransform(BotFrames *bot_frame, BotTrans *trans, int64_t utime, std::string channel_name)
{
    BotTrans link_transf;
    bot_trans_set_from_quat_trans(&link_transf, trans->rot_quat, trans->trans_vec);
    frame_handle_t * frame_handle = (frame_handle_t *) g_hash_table_lookup(bot_frame->frame_handles_by_channel, channel_name.c_str());
    assert(frame_handle != NULL);
    frame_handle->was_updated = 1;
    bot_ctrans_link_update(frame_handle->ctrans_link, &link_transf, utime);
}

