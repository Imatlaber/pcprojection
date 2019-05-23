/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: playback lidar pcap and odometry file accoording to timestamps
 */

#include <iostream>
#include <thread>
#include <condition_variable>
#include <stdint.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../../utilities/drivers/velodyne/pcl_point_types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/trans.hpp"
#include "../../../common/adapters/pc_adapter.hpp"
#include "../../../common/adapters/odom_adapter.hpp"
#include "../../utilities/drivers/misc/images_from_file.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class VeloOdomPlayer
{
    public:
        
        VeloOdomPlayer(lcm::LCM * lcm,
                       param_t * param,
                       trans_t * trans,
                       PCAdapterBase * adapter, 
                       OdomFrames * odom,
                       ImageReader * img_reader,
					   int begin_time,
					   int max_time):
            lcm_(lcm),
            param_(param),
            trans_(trans),
            adapter_(adapter),
            odom_(odom),
            img_reader_(img_reader),
            curr_lidar_utime_(0),
            curr_odom_utime_(0),
            curr_image_utime_(0),
			begin_time_(begin_time),
			max_time_(max_time)
        {
            lidar_th_ = std::thread(&VeloOdomPlayer::lidar_reader, this) ;
            printf("mode %d \n", odom_->get_mode()) ;
            if(odom_->get_mode() == 0)
                odom_th_ = std::thread(&VeloOdomPlayer::odometry_reader, this) ;
            else if(odom_->get_mode() == 1)
                odom_th_ = std::thread(&VeloOdomPlayer::raw_odometry_reader, this) ;
            image_th_ = std::thread(&VeloOdomPlayer::image_reader, this) ;
			offset_ = 0. ;         
        }
    
        ~VeloOdomPlayer()
        {
            lidar_th_.join() ;
            odom_th_.join() ;
        }
                
        void lidar_reader()
        {
            pcl::PointCloud<PCLVelodyne> one_rev ;
            bool finished = false ;
            while(!finished)
            {
                std::unique_lock<std::mutex> lck(mtx_) ;
                if(curr_lidar_utime_+offset_ >= curr_odom_utime_ || curr_lidar_utime_+offset_ >= curr_image_utime_)
                    cv_pub_lidar_.wait(lck) ;
                if(adapter_->next_rev(one_rev))
                {
					static bool initialized = false ;
					if(initialized)
					{
						int curr_lidar_time = curr_lidar_utime_/1e6 ;
						if(curr_lidar_time > begin_time_ && curr_lidar_time < max_time_)
							adapter_->publish_onerev(one_rev) ;
						printf("lida time %zu \n", curr_lidar_utime_) ;
					}
                    curr_lidar_utime_ = one_rev.header.stamp ; 
					initialized = true ;
                }
                else
                {
                    finished = true ;
                    curr_lidar_utime_ = LLONG_MAX ;
                }
                
                if(curr_lidar_utime_+offset_ >= curr_odom_utime_ && curr_image_utime_ >= curr_odom_utime_ )
                    cv_pub_odom_.notify_all() ;
                if(curr_lidar_utime_+offset_ >= curr_image_utime_ && curr_odom_utime_ >= curr_image_utime_)
                    cv_pub_image_.notify_all() ;
                
            }

        }

        void odometry_reader()
        {
            OdomPoint point ;
            bool finished = false ;
            while(!finished)
            {
                std::unique_lock<std::mutex> lck(mtx_) ;
                if(curr_odom_utime_ > curr_lidar_utime_+offset_ || curr_odom_utime_ > curr_image_utime_)
                    cv_pub_odom_.wait(lck) ;
                if(odom_->next_point(point))
                {
					static bool initialized = false ;
					if(initialized)
					{
						int curr_odom_time = curr_odom_utime_/1e6 ;
						if(curr_odom_time > begin_time_ && curr_odom_time < max_time_)
							odom_->publish_step(point) ;
                    	printf("odom time % zd \n", curr_odom_utime_) ; 
					}
                    curr_odom_utime_ =  static_cast<int64_t>(point.time) ;
					initialized = true ;
                }
                else
                {
                    finished = true ;
                    curr_odom_utime_ = LLONG_MAX ;
                }
                
                //printf("curr_odom_utime_ %zd, curr_lidar_time %zd, curr_image_time %zd \n", curr_odom_utime_, curr_lidar_utime_, curr_image_utime_) ;
                if(curr_odom_utime_ > curr_lidar_utime_+offset_ && curr_image_utime_ > curr_lidar_utime_ + offset_)
                    cv_pub_lidar_.notify_all() ;
                if(curr_odom_utime_ > curr_image_utime_ && curr_lidar_utime_ + offset_ > curr_image_utime_)
                    cv_pub_image_.notify_all() ;
            }
        }
       
        void raw_odometry_reader()
        {
            RawOdomPoint raw_point ;
            bool finished = false ;
            while(!finished)
            {
                std::unique_lock<std::mutex> lck(mtx_) ;
                if(curr_odom_utime_ > curr_lidar_utime_+offset_ || curr_odom_utime_ > curr_image_utime_)
                    cv_pub_odom_.wait(lck) ;
                if(odom_->next_point(raw_point))
                {
                     
					static bool initialized = false ;
					if(initialized)
					{
						int curr_odom_time = curr_odom_utime_/1e6 ;
						if(curr_odom_time > begin_time_ && curr_odom_time < max_time_)
							odom_->publish_step(raw_point) ;
                    	printf("odom time % zd \n", curr_odom_utime_) ; 
					}
                    curr_odom_utime_ =  static_cast<int64_t>(raw_point.utc_time*1e6) ;
					initialized = true ;
                }
                else
                {
                    finished = true ;
                    curr_odom_utime_ = LLONG_MAX ;
                }

                if(curr_odom_utime_ > curr_lidar_utime_+offset_ && curr_image_utime_ > curr_lidar_utime_ + offset_)
                    cv_pub_lidar_.notify_all() ;
                if(curr_odom_utime_ > curr_image_utime_ && curr_lidar_utime_ + offset_ > curr_image_utime_)
                    cv_pub_image_.notify_all() ;
            }
        }
       
        void image_reader()
        {
            SimpleImage si ;
            bool finished = false ;
            while(!finished)
            {
                std::unique_lock<std::mutex> lck(mtx_) ;
                if(curr_image_utime_ > curr_lidar_utime_+offset_ || curr_image_utime_ > curr_odom_utime_)
                    cv_pub_image_.wait(lck) ;
                if(img_reader_->next_image(si))
                {
					static bool initialized = false ;
					if(initialized)
					{
						int curr_image_time = curr_image_utime_/1e6 ;
						if(curr_image_time > begin_time_ && curr_image_time < max_time_)
							img_reader_->publish_image(si) ;
                    	printf("image time % zd \n", curr_image_utime_) ; 
					}
                    curr_image_utime_ =  static_cast<int64_t>(si.utime) ;
					initialized = true ;
                }
                else
                {
                    finished = true ;
                    curr_image_utime_ = LLONG_MAX ;
                }

                if(curr_image_utime_ > curr_lidar_utime_+offset_ && curr_odom_utime_ > curr_lidar_utime_+offset_)
                    cv_pub_lidar_.notify_all() ;
                if(curr_image_utime_ > curr_odom_utime_ && curr_lidar_utime_ + offset_ > curr_odom_utime_)
                    cv_pub_odom_.notify_all() ;
            }
        }
       

        lcm::LCM * lcm_ ;
        param_t * param_ ;
        trans_t * trans_ ;
        PCAdapterBase * adapter_ ;
        OdomFrames * odom_ ;
        ImageReader * img_reader_ ;
        std::thread lidar_th_ ;
        std::thread odom_th_ ;
        std::thread image_th_ ;
        std::condition_variable cv_pub_lidar_ ;
        std::condition_variable cv_pub_odom_ ;
        std::condition_variable cv_pub_image_ ;

        int64_t curr_lidar_utime_ ;
        int64_t curr_odom_utime_ ;
        int64_t curr_image_utime_ ;
        std::mutex mtx_ ;
		int begin_time_ ;
		int max_time_ ;
		int64_t offset_ ;

} ;

int main(int argc, char** argv)
{ 
    po::options_description desc("Allowed options");
    po::variables_map vm;
	
	desc.add_options()
		("help,h", "help message")
		("type,t", po::value<std::string>(), "supported type: VLP16, HDL32, HDL64")
		("pcap,p", po::value<std::string>(), "pcap file path") 
		("odom,o", po::value<std::string>(), "odometry file path") 
		("odom_mode,d", po::value<int>(), "0:engineering, 1:lat lon height (default)") 
		("image,i", po::value<std::string>(), "image file path (optional)") 
		("begin_time,b", po::value<int>(), "beginning time in secs (optional)") 
		("max_time,m", po::value<int>(), "max time in secs to process (optional)") ;
	po::store(po::parse_command_line(argc, argv, desc), vm) ;

	if(vm.count("type")==0 || vm.count("pcap")==0 || vm.count("odom")==0 || vm.count("help"))
	{
		std::cout << desc << std::endl ;
		return 1 ;
	}

	std::string pcap_dir = vm["pcap"].as<std::string>() ;
	if(pcap_dir.back() == '/') 
		pcap_dir.pop_back() ;

	std::string odom_dir = vm["odom"].as<std::string>() ;
	if(odom_dir.back() == '/') 
		odom_dir.pop_back() ;

    int odom_mode = 1 ;
    if(vm.count("odom_mode") != 0)
        odom_mode = vm["odom_mode"].as<int>() ;

	std::string type = vm["type"].as<std::string>() ;
	int num_lasers = 0 ;
	if(type == "VLP16")
		num_lasers = 16 ;
	else if(type == "HDL32") 
		num_lasers = 32 ;
	else if(type == "HDL64")
		num_lasers = 64 ;
	else
	{
		printf("Error: %s type is not supported \n", type.c_str()) ;
		return 1 ;
	}

	int max_time = INT_MAX ;
	if(vm.count("max_time") != 0)
		max_time = vm["max_time"].as<int>() ;
	int begin_time = 0 ;
	if(vm.count("begin_time") != 0)
		begin_time = vm["begin_time"].as<int>() ;
    
	std::string image_dir ;
    if(vm.count("image") != 0)
        image_dir = vm["image"].as<std::string>() ;
	if(image_dir.back() == '/') 
		image_dir.pop_back() ;

	// create lcm related objects
	lcm::LCM lcm ;
	param_t param(&lcm) ;
	trans_t trans(lcm.getUnderlyingLCM(), &param) ;

	// create lidar reader
    double time_ratio = 0.5 ;
    std::map<int, std::string> pcap_files ;
    pcap_files.insert(std::make_pair(0, pcap_dir)) ;
	PCAdapterBase * adapter = dynamic_cast<PCAdapterBase*>(new PCAdapterPCAP(pcap_files, num_lasers, time_ratio, &lcm, &param, &trans, "VELO_RAW")) ;  

	// create odometry reader
	OdomFrames odom_frames(&lcm, "", odom_mode) ;
    odom_frames.open_frames(odom_dir, "ODOM") ;

    // create image reader
    ImageReader img_reader(&lcm, &param, image_dir) ; 

    VeloOdomPlayer player(&lcm, &param, &trans, adapter, &odom_frames, &img_reader, begin_time, max_time) ;

	return 0 ;
}
