#ifndef ODOM_ADAPTER_HPP
#define ODOM_ADAPTER_HPP

#include <iostream>
#include <boost/graph/graph_concepts.hpp>


#include <future>
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
//#include <mrpt/math/math_frwds.h>
//#include <mrpt/math/eigen_frwds.h>
// #include <boost/program_options.hpp>
// #include <boost/filesystem.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <bot_core/small_linalg.h>
#include <liblas/liblas.hpp>
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/trans.hpp"
#include "lcmtypes/had/pointcloud_t.hpp"
#include "../../utilities/drivers/velodyne/pcl_point_types.h"
#include "../../utilities/drivers/velodyne/pc_type_translator.hpp"
#include "../../utilities/hadif/time_helper.hpp"
#include "../../utilities/hadif/reader_helper.hpp"
#include "../../utilities/hadif/gps_helper.hpp"
#include "odom_list.hpp"

struct RawOdomPoint
{
    double weeks;
    double weekseconds;
    double lat ;
    double lon ;
    double height;
    double vel_north ;
    double vel_east ;
    double vel_up ;
    double roll ;
    double pitch ;
    double heading ;
    double rms_xyz;
    double rms_rpy;
//     double utc_time ;
} ;

struct OdomPoint
{
    double time ;
    double pos[3] ;
    double rpy[3] ;
    double rms_xyz;
    double rms_rpy;
} ;

class OdomFrames
{
public:
    int loopcount_;
    int64_t mintime_;
    int64_t maxtime_;
    std::list<HDTimesTrans> * odom_list_;
    //std::string record_dir_ ;
    std::string record_tra_file_ ;
    CSVReader<RawOdomPoint> reader_raw_ ;
    double min_x_ ;
    double max_x_ ;
    double min_y_ ;
    double max_y_ ;
    double min_z_ ;
    double max_z_ ;
    std::string pub_channel_ ;
    double lat_ ;
    double lon_ ;
    double height_ ;
    GpsHelperBase * gps_helper_ ;
    std::string module_name_ ;
    const int odom_file_offset_ ;
    const int raw_odom_file_offset_ ;
    int mode_ ;
    double begin_time_ ; // sec
    double end_time_ ; // sec
    std::vector<double> latlonheight_;
    int64_t last_utime ;
public:
    OdomFrames(std::string dir,
               std::list<HDTimesTrans> * odom_list,
               std::vector<double> latlonheight,
               double begin_time = 0.,
               double end_time = DBL_MAX):
        odom_list_(odom_list),
        record_tra_file_(dir),
        reader_raw_(9),
        min_x_(DBL_MAX),
        max_x_(-DBL_MAX),
        min_y_(DBL_MAX),
        max_y_(-DBL_MAX),
        min_z_(DBL_MAX),
        max_z_(-DBL_MAX),
        module_name_("OdomFrames"),
        odom_file_offset_(4),
        raw_odom_file_offset_(0),
        begin_time_(begin_time),
        end_time_(end_time),
        latlonheight_(latlonheight)
    {
        loopcount_ = 0;
        mintime_ = 0;
        maxtime_ = 0;
        // config parameters
        if(latlonheight_.size() != 3)
        {
            printf("Cannot find origin from config! Exiting... \n") ;
            exit(-1) ;
        }

        lat_ = latlonheight_[0] ;
        lon_ = latlonheight_[1] ;
        height_ = latlonheight_[2] ;

        printf("============================ \n") ;
        printf("Module: %s \n", module_name_.c_str()) ;
        printf("lat %f, lon %f, height %f \n", lat_, lon_, height_) ;
        printf("============================ \n") ;

        // initialize gps helper

        gps_helper_ = dynamic_cast<GpsHelperBase*>(new GpsHelperProj()) ;

        latlonheight_t origin ;
        origin.lat = lat_ ;
        origin.lon = lon_ ;
        origin.height = height_ ;

        gps_helper_->compute_local_transformation(origin) ;

        last_utime = 0;
        read_to_frames();
    }

//    bool read_to_frames(std::string file_to_read)
//    {
//        std::string traj_file = record_dir_ + "/" + file_to_read ;

//        if(!reader_raw_.open(traj_file))
//        {
//            printf("Failed to read trajectory file %s \n", traj_file.c_str()) ;
//            return false ;
//        }
//        // skip header
//        reader_raw_.next_line(raw_odom_file_offset_) ;
//        read_all(reader_raw_) ;

//        return true ;
//    }
//     1522135237.0,116.2490943974,40.2008696110,47.359,0.4734980000,
//     0.6766470000,276.9954960000,-0.003,0.005;

    bool read_to_frames()
    {
        std::string traj_file = record_tra_file_ ;

        if(!reader_raw_.open(traj_file))
        {
            printf("Failed to read trajectory file %s \n", traj_file.c_str()) ;
            return false ;
        }
        // skip header
        reader_raw_.next_line(raw_odom_file_offset_) ;

        read_all(reader_raw_);

        return true ;
    }

    template<class T>
    void read_all(CSVReader<T>& reader)
    {
        T point ;

        while(next_point(point))
        {
            read_step(point) ;
        }
        reader.close() ;
    }

    bool next_point(RawOdomPoint& raw_point)
    {
        return reader_raw_ >> raw_point ;
    }

    void read_step(RawOdomPoint& raw_point)
    {
        OdomPoint point ;

        rawodompoint_to_odompoint(raw_point, point) ;
// 	printf("lon= %f,lat=%f,h=%f,roll=%f,pitch=%f,yaw=%f \n",
//  	       raw_point.lon, raw_point.lat,raw_point.height,
//  	       raw_point.roll, raw_point.pitch,raw_point.heading);
        read_step(point) ;
    }

    void read_step(OdomPoint& point)
    {
        double factor = 3.1415927/180.0 ;
//         printf("x= %f,y=%f,z=%f \n",point.pos[0], point.pos[1],point.pos[2]) ;
/// record x, y, z span
//         if(point.pos[0] < min_x_)
//             min_x_ = point.pos[0] ;
//         if(point.pos[0] > max_x_)
//             max_x_ = point.pos[0] ;
//         if(point.pos[1] < min_y_)
//             min_y_ = point.pos[1] ;
//         if(point.pos[1] > max_y_)
//             max_y_ = point.pos[1] ;
//         if(point.pos[2] < min_z_)
//             min_z_ = point.pos[2] ;
//         if(point.pos[2] > max_z_)
//             max_z_ = point.pos[2] ;

        // INSPVA protocal: azimuth - left handed rotation around z-axis clockwise from North
        point.rpy[2] = 360-point.rpy[2] ; // to counter-clockwise
        bot_vector_scale_3d(point.rpy, factor) ;

        int64_t curr_utime = static_cast<int64_t>(point.time) ;
// 	printf("time = %f ,curr_utime=%lld\n", point.time,curr_utime) ;
        double curr_stime = curr_utime ;

        if(curr_stime < begin_time_ || curr_stime > end_time_)
            return ;
        int64_t udtime = last_utime ? curr_utime-last_utime : 0 ;
        if(udtime < 0)
        {
            printf("time difference %zd channel \n", udtime) ;
            printf("warning: out of sequence measurements detected for odometry - skip!\n") ;
            return ; // note: old timestamp will clear all frame buffer in this channel!
        }
        last_utime = curr_utime ;

        HDTrans trans ;
        bot_roll_pitch_yaw_to_quat(point.rpy, trans.rot_quat) ;

        memcpy(trans.trans_vec, point.pos, sizeof(double)*3) ;
        HDTimesTrans timetrans;
        timetrans.utime = static_cast<int64_t>(point.time);
        timetrans.hdtrans = trans;
        timetrans.rms_xyz = point.rms_xyz;
        timetrans.rms_rpy = point.rms_rpy;
        odom_list_->push_back(timetrans);

        if(loopcount_ == 0)
        {
            mintime_ = curr_utime;
            loopcount_ ++;
        }
        maxtime_ = curr_utime;
    }

    void rawodompoint_to_odompoint(const RawOdomPoint& raw_point, OdomPoint& point)
    {
        point.time = utc_gps2sec(raw_point.weeks,raw_point.weekseconds)*1e6/*(raw_point.weeks*7*24*3600+raw_point.weekseconds+315964800)*1e6*//*raw_point.utc_time * 1e6*/ ;
//         printf("point.time = %f\n",point.time);
        double llh[3] = {raw_point.lat, raw_point.lon, raw_point.height} ;
        gps_helper_->latlonheight2local(llh, point.pos) ;
        double rpy[3] = {raw_point.roll, raw_point.pitch, raw_point.heading} ;
        memcpy(point.rpy, rpy, sizeof(double)*3) ;
        point.rms_xyz = raw_point.rms_xyz;
        point.rms_rpy = raw_point.rms_rpy;
    }

    double get_min_x() const
    {
        return min_x_ ;
    }

    double get_max_x() const
    {
        return max_x_ ;
    }

    double get_min_y() const
    {
        return min_y_ ;
    }

    double get_max_y() const
    {
        return max_y_ ;
    }

    double get_min_z() const
    {
        return min_z_ ;
    }

    double get_max_z() const
    {
        return max_z_ ;
    }


};
#endif // ODOM_ADAPTER_HPP
