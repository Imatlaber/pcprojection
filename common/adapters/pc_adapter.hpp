/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: mapping using pointclouds, odometry and corrections
 */

#ifndef _PC_ADAPTER_HPP_
#define _PC_ADAPTER_HPP_

#include <future>
// #include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
// #include <boost/program_options.hpp>
// #include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <bot_core/small_linalg.h>
#include <liblas/liblas.hpp>
#include "../../../utilities/hadif/param.hpp"
#include "../../../utilities/hadif/trans.hpp"
#include "lcmtypes/had/pointcloud_t.hpp"
#include "../../utilities/drivers/velodyne/pcl_point_types.h"
#include "../../utilities/drivers/velodyne/pc_type_translator.hpp"
#include "../../utilities/drivers/velodyne/velodyne_driver.hpp"
#include "../../utilities/drivers/velodyne/hdl32_driver.hpp"
#include "../../utilities/drivers/velodyne/hdl64_driver.hpp"
#include "../../utilities/hadif/time_helper.hpp"
#include "../../utilities/hadif/reader_helper.hpp"

struct LidarPoint
{
	LidarPoint():
	pos{0,0,0},
	intensity(0),
	laser_id(0),
	azimuth(0),
	distance(0),
	adjusted_time(0),
	utime(0)
	{
	}

	double pos[3] ;
	double intensity ;
	double laser_id ;
	double azimuth ; //  [0, 36000)
	double distance ; // m
	double adjusted_time ;	
	double utime ; // us
} ;

struct LidarTrajectoryPoint
{
	LidarTrajectoryPoint():
			pos{0,0,0},
			intensity(0),
			laser_id(0),
			azimuth(0),
			distance(0),
			adjusted_time(0),
			utime(0),
			quat0(0),
			quat1(0),
			quat2(0),
			quat3(0),
			translation0(0),
			translation1(0),
			translation2(0)

	{
	}

	double pos[3] ;
	double intensity ;
	double laser_id ;
	double azimuth ; //  [0, 36000)
	double distance ; // m
	double adjusted_time ;
	double utime ; // us
	double quat0;
	double quat1;
	double quat2;
	double quat3;
	double translation0;
	double translation1;
	double translation2;
} ;

class PCAdapterBase
{
	public:

	PCAdapterBase()
	{
	}

	PCAdapterBase(lcm::LCM * lcm,
				  param_t * param,
				  trans_t * trans,
				  std::string channel
				  ):
		lcm_(lcm),
		param_(param),
		trans_(trans),
		channel_(channel)
	{
		
	}

	virtual bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev){} ;
	virtual bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev, std::vector<BotTrans>& trajectory_onerev){};
	virtual bool next_covs(std::vector<Eigen::Matrix3d>& covs) {} 
	virtual bool next_normals(std::vector<Eigen::Vector3d>& normals) {}  	
	virtual int get_num_lasers() = 0 ;
	virtual bool restart()  {}

	void publish_onerev(const pcl::PointCloud<PCLVelodyne>& one_rev)
	{
		had::pointcloud_t had_pc ;
		fromPCLVelodyne(one_rev, had_pc) ;
		
		int64_t curr_utime = one_rev.header.stamp ;
		had_pc.header.utime = curr_utime ;
		had_pc.coord_frame = "VELO_P2" ;
		lcm_->publish(channel_, &had_pc) ;
		
		//static int64_t last_utime = 0 ;
		//int64_t dutime = last_utime ? curr_utime-last_utime : 0 ; 
		//last_utime = curr_utime ;
		//usleep(dutime) ;
	}

	public:
	lcm::LCM * lcm_ ;
	param_t * param_ ;
	trans_t * trans_ ;
	std::string channel_ ;
} ;

class PCAdapterPCAP: public PCAdapterBase
{
	public:
    std::map<int, std::string> filenames_ ;
    std::vector<VelodyneInput*> vis_ ;			
	const int num_lasers_ ;
	int64_t time_offset_ ;
	bool initialized_ ;
    double rev_time_ratio_ ;

	public:
	PCAdapterPCAP(std::map<int, std::string> files, 
				  const int num_lasers, 
				  double ratio=0.5,
				  lcm::LCM * lcm = nullptr,
				  param_t * param = nullptr,
				  trans_t * trans = nullptr,
				  std::string channel = ""
				  ) :
	PCAdapterBase(lcm, param, trans, channel),
	filenames_(files),
	num_lasers_(num_lasers),
	time_offset_(0),
	initialized_(false),
    rev_time_ratio_(ratio)
	{
        clamp(0., rev_time_ratio_, 1.) ;
        for(const auto& ele : filenames_)
        {
            std::string filename = ele.second ;
		    switch(num_lasers_)
		    {
		    	case 16:
		    		vis_.push_back(new VelodyneInput(filename, new VLP16Calibration())) ;
		    		break ;
		    	case 32:
		    		vis_.push_back(dynamic_cast<VelodyneInput *>(new HDL32Input(filename, new HDL32Calibration()))) ;
		    		break ;
		    	case 64: 
		    		vis_.push_back(dynamic_cast<VelodyneInput *>(new HDL64Input(filename, new HDL64Calibration()))) ;
		    	    break ;	
		    	default:
		    		vis_.push_back(nullptr) ;
		    }
        }
	}

	int get_num_lasers() override
	{
		return num_lasers_ ;
	}
	
	bool restart() override
	{
        for(auto& vi : vis_)
        {
		    if(!vi->restart())
                return false ;
        }
        return true ;
	}


	bool next_rev(pcl::PointCloud<PCLVelodyne> & pc_out) override
	{
		std::vector<uint8_t> data ;
		pc_out.clear() ;
		int64_t utime = 0 ;
		static pcl::PointCloud<PCLVelodyne> one_rev ;
        for(auto& vi : vis_)
        {
		    while(vi->getPacket(data))
		    {
		    	size_t packet_size = data.size() ;
		    	//printf("packdet size %zu \n", packet_size) ;
		    	if(packet_size == 512)
		    	{
		    		if(vi->decode_packet_pep(data, time_offset_))
		    			initialized_ = true ;
		    		
		    		//printf("time offset %lld sec (utc) \n", time_offset_) ;
		    	}
		    	else if(packet_size == 1206)
		    	{
		    		if(!initialized_)
		    		{
		    			printf("waiting for gps time to initialize...\n") ;
		    			continue ;
		    		}
		    		std::vector< pcl::PointCloud<PCLVelodyne> > pc_packet ;
		    		int block_index = -1 ;
		    		if(vi->decode_packet(data, pc_packet, block_index, time_offset_))
		    		{
                        static int64_t utime_start = pc_packet[0].header.stamp ; 
		    			if(block_index >= 0)	
		    			{
		    				for(int idx=0; idx<block_index; ++idx)
		    				{
		    					one_rev += pc_packet[idx] ;
		    					utime = pc_packet[idx].header.stamp ;
		    				}

		    				pc_out += one_rev ;	
		    				pc_out.height = vi->getVelodyneRing() ;
		    				pc_out.width = one_rev.size()/vi->getVelodyneRing() ;
		    				pc_out.is_dense = true ;
		    				pc_out.header.stamp = (1-rev_time_ratio_)*utime_start + rev_time_ratio_ * utime ;
		    				one_rev.clear() ;
                            
                            /// starting blocks for the next rev
		    				for(int idx=block_index; idx<pc_packet.size(); ++idx)
		    					one_rev += pc_packet[idx] ;
                            utime_start = pc_packet[block_index].header.stamp ; 

		    				break ;
		    			}
		    			else
		    			{
		    				for(const auto& pc : pc_packet)
		    				{
		    					one_rev += pc ;
		    					utime = pc.header.stamp ;
		    				}
		    			}
		    			
		    		}
		    		else
					{
						pc_out.clear();
						return false;
					}
		    	}
		    	else
		    	{
		    		printf("failed to decode this packet. skip \n") ;
		    		continue ;
		    	}
		    }
        }
		
        return true ;
	}

} ;

class PCAdapterCSV : public PCAdapterBase
{
	public:
	std::map<int, std::string> cloud_files_ ;
	CSVReader<LidarPoint> cloud_reader_ ;
	const int num_lasers_ ;
	std::map<int, std::string>::iterator it;

	public:
	PCAdapterCSV(std::map<int, std::string> files, const int num_lasers):
	cloud_files_(files),
	cloud_reader_(9),
	num_lasers_(num_lasers)
	{
		it = cloud_files_.begin() ;
	}

	int get_num_lasers() override
	{
		return num_lasers_ ;
	}

	inline void  point2pcl_point(const LidarPoint& point, PCLVelodyne& pp)
	{
		pp.x = point.pos[0] ;
		pp.y = point.pos[1] ;
		pp.z = point.pos[2] ;
		pp.distance = point.distance ;
		pp.intensity = point.intensity ;
		pp.ring = (int)point.laser_id ;
		int64_t utime = static_cast<int64_t>(point.utime) ;
		memcpy(&pp.utime1, &utime, sizeof(int64_t)) ;
	}

	bool restart() override
	{
		it = cloud_files_.begin() ;
		return true ;
	}

	bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev) override	
	{
		one_rev.clear() ;

		if(it == cloud_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!cloud_reader_.open(file_name))
		{
			printf("Failed to read lidar file %s \n", file_name.c_str()) ;
			return false ;
		}
		
		cloud_reader_.next_line() ; // skip header
		
		LidarPoint point ;
		// start from laser 0

		int laser_index = 0 ;
		one_rev.clear() ;
		int counter = 0 ;
		while(cloud_reader_ >> point)
		{
			// temporary fix for lidar timeoffset
			//point.utime /= 1e6 ;
			//point.utime += 1481274000 ;	
			//point.utime *= 1e6 ;

			//if((int)point.laser_id == 0)
			//{
			//	std::cout << "count " << counter << std::endl ;	
			//	if(counter != 32)
			//		printf("<<<<<<<<<<<<<<<<<< counter %d \n", counter) ;
			//	counter = 0 ;
			//}
			//++counter ;
			
			PCLVelodyne pp ;
			point2pcl_point(point, pp) ; 
			one_rev.push_back(pp) ;
			++laser_index ;
			laser_index %= num_lasers_ ;
		}
	
		one_rev.height = 1 ;
		one_rev.width = one_rev.size() ;
		one_rev.is_dense = true ;
		one_rev.header.stamp = static_cast<int64_t>(point.utime) ;

		cloud_reader_.close() ;
	
		++it ;

		return true ;
	}


} ;

class PCAdapterCalibUnsupervisedCSV : public PCAdapterBase
{
public:
	std::map<int, std::string> cloud_files_ ;
	CSVReader<LidarPoint> cloud_reader_ ;
	CSVReader<LidarTrajectoryPoint> cloud_tra_reader_ ;
	const int num_lasers_ ;
	std::map<int, std::string>::iterator it ;

public:
	PCAdapterCalibUnsupervisedCSV(std::map<int, std::string> files, const int num_lasers):
			cloud_files_(files),
			cloud_reader_(9),
			cloud_tra_reader_(16),
			num_lasers_(num_lasers)
	{
		it = cloud_files_.begin() ;
	}

	int get_num_lasers() override
	{
		return num_lasers_ ;
	}

	inline void  point2pcl_point(const LidarPoint& point, PCLVelodyne& pp)
	{
		pp.x = point.pos[0] ;
		pp.y = point.pos[1] ;
		pp.z = point.pos[2] ;
		pp.distance = point.distance ;
		pp.intensity = point.intensity ;
		pp.ring = (int)point.laser_id ;
		int64_t utime = static_cast<int64_t>(point.utime) ;
		memcpy(&pp.utime1, &utime, sizeof(int64_t)) ;
	}

	inline void  point2pcl_point(const LidarTrajectoryPoint& point, PCLVelodyne& pp)
	{
		pp.x = point.pos[0] ;
		pp.y = point.pos[1] ;
		pp.z = point.pos[2] ;
		pp.distance = point.distance ;
		pp.intensity = point.intensity ;
		pp.ring = (int)point.laser_id ;
		int64_t utime = static_cast<int64_t>(point.utime) ;
		memcpy(&pp.utime1, &utime, sizeof(int64_t)) ;
	}

    bool restart() override
    {
        it = cloud_files_.begin() ;
        return true ;
    }

	bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev) override
	{
		one_rev.clear() ;

		if(it == cloud_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!cloud_reader_.open(file_name))
		{
			printf("Failed to read lidar file %s \n", file_name.c_str()) ;
			return false ;
		}

		cloud_reader_.next_line() ; // skip header

		LidarPoint point ;
		// start from laser 0

		int laser_index = 0 ;
		one_rev.clear() ;
		int counter = 0 ;
		while(cloud_reader_ >> point)
		{
			// temporary fix for lidar timeoffset
			//point.utime /= 1e6 ;
			//point.utime += 1481274000 ;
			//point.utime *= 1e6 ;

			//if((int)point.laser_id == 0)
			//{
			//	std::cout << "count " << counter << std::endl ;
			//	if(counter != 32)
			//		printf("<<<<<<<<<<<<<<<<<< counter %d \n", counter) ;
			//	counter = 0 ;
			//}
			//++counter ;

			PCLVelodyne pp ;
			point2pcl_point(point, pp) ;
			one_rev.push_back(pp) ;
			++laser_index ;
			laser_index %= num_lasers_ ;
		}

		one_rev.height = 1 ;
		one_rev.width = one_rev.size() ;
		one_rev.is_dense = true ;
		one_rev.header.stamp = static_cast<int64_t>(point.utime) ;

		cloud_reader_.close() ;

		++it ;

		return true ;
	}


	bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev, std::vector<BotTrans>& trajectory_onerev) override
	{
		one_rev.clear() ;
		trajectory_onerev.clear();

		if(it == cloud_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!cloud_tra_reader_.open(file_name))
		{
			printf("Failed to read lidar file %s \n", file_name.c_str()) ;
			return false ;
		}

        cloud_tra_reader_.next_line() ; // skip header

		LidarTrajectoryPoint point ;
		// start from laser 0

		int laser_index = 0 ;
		one_rev.clear() ;
		int counter = 0 ;
		while(cloud_tra_reader_ >> point)
		{
			PCLVelodyne pp ;
			point2pcl_point(point, pp) ;
			one_rev.push_back(pp) ;
			++laser_index ;
			laser_index %= num_lasers_ ;

			BotTrans tmptra;
			tmptra.rot_quat[0] = point.quat0;
			tmptra.rot_quat[1] = point.quat1;
			tmptra.rot_quat[2] = point.quat2;
			tmptra.rot_quat[3] = point.quat3;
			tmptra.trans_vec[0] = point.translation0;
			tmptra.trans_vec[1] = point.translation1;
			tmptra.trans_vec[2] = point.translation2;
			trajectory_onerev.push_back(tmptra);
		}

		one_rev.height = 1 ;
		one_rev.width = one_rev.size() ;
		one_rev.is_dense = true ;
		one_rev.header.stamp = static_cast<int64_t>(point.utime) ;

        cloud_tra_reader_.close() ;

		++it ;

		return true ;
	}
} ;


class PCAdapterCalibCSV : public PCAdapterBase
{
	public:
	std::map<int, std::string> cloud_files_ ;
	std::map<int, std::string> covs_files_ ;
	std::map<int, std::string> normals_files_ ;
	CSVReader<LidarPoint> cloud_reader_ ;
	CSVReader<LidarPoint> covs_reader_ ;
	CSVReader<LidarPoint> normals_reader_ ;
	const int num_lasers_ ;
	

	public:
	PCAdapterCalibCSV(const std::map<int, std::string>& pc_files,
                      const std::map<int, std::string>& cov_files,
                      const std::map<int, std::string>& normal_files,
                      const int num_lasers):
	cloud_files_(pc_files),
	covs_files_(cov_files),
	normals_files_(normal_files),
	cloud_reader_(9),
	covs_reader_(10),
	normals_reader_(4),
	num_lasers_(num_lasers)
	{
		
	}

	int get_num_lasers() override
	{
		return num_lasers_ ;
	}

	inline void  point2pcl_point(const LidarPoint& point, PCLVelodyne& pp)
	{
		pp.x = point.pos[0] ;
		pp.y = point.pos[1] ;
		pp.z = point.pos[2] ;
		pp.distance = point.distance ;
		pp.intensity = point.intensity ;
		pp.ring = (int)point.laser_id ;
		int64_t utime = static_cast<int64_t>(point.utime) ;
		memcpy(&pp.utime1, &utime, sizeof(int64_t)) ;
	}

	bool next_rev(pcl::PointCloud<PCLVelodyne>& one_rev) override	
	{
		one_rev.clear() ;

		static std::map<int, std::string>::iterator it = cloud_files_.begin() ;

		if(it == cloud_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!cloud_reader_.open(file_name))
		{
			printf("Failed to read lidar file %s \n", file_name.c_str()) ;
			return false ;
		}
		
		cloud_reader_.next_line() ; // skip header
		
		LidarPoint point ;
		// start from laser 0
		//while(cloud_reader_ >> point && (int)point.laser_id != 0)
		//{
		//}
		int laser_index = 0 ;
		one_rev.clear() ;
		int counter = 0 ;
		while(cloud_reader_ >> point)
		{
			// temporary fix for lidar timeoffset
			//point.utime /= 1e6 ;
			//point.utime += 1481274000 ;	
			//point.utime *= 1e6 ;

			//if((int)point.laser_id == 0)
			//{
			//	std::cout << "count " << counter << std::endl ;	
			//	if(counter != 32)
			//		printf("<<<<<<<<<<<<<<<<<< counter %d \n", counter) ;
			//	counter = 0 ;
			//}
			//++counter ;
			
			PCLVelodyne pp ;
			point2pcl_point(point, pp) ; 
			one_rev.push_back(pp) ;
			++laser_index ;
			laser_index %= num_lasers_ ;
		}
	
		one_rev.height = 1 ;
		one_rev.width = one_rev.size() ;
		one_rev.is_dense = true ;
		one_rev.header.stamp = static_cast<int64_t>(point.utime) ;

		cloud_reader_.close() ;
	
		++it ;

		return true ;
	}

	bool next_covs(std::vector<Eigen::Matrix3d>& covs) override	
	{
		covs.clear() ;

		static std::map<int, std::string>::iterator it = covs_files_.begin() ;

		if(it == covs_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!covs_reader_.open(file_name))
		{
			printf("Failed to read covariance file %s \n", file_name.c_str()) ;
			return false ;
		}
		
		covs_reader_.next_line() ; // skip header
		
        std::vector<std::string> tokens ;
		while(covs_reader_.next_raw_tokens(tokens))
		{
            Eigen::Matrix3d cov ;
            cov(0,0) = std::stod(tokens[0]) ; 
            cov(0,1) = std::stod(tokens[1]) ; 
            cov(0,2) = std::stod(tokens[2]) ; 
            cov(1,0) = std::stod(tokens[3]) ; 
            cov(1,1) = std::stod(tokens[4]) ; 
            cov(1,2) = std::stod(tokens[5]) ; 
            cov(2,0) = std::stod(tokens[6]) ; 
            cov(2,1) = std::stod(tokens[7]) ; 
            cov(2,2) = std::stod(tokens[8]) ; 
			covs.push_back(cov) ;
		} 
	
		covs_reader_.close() ;
	
		++it ;

		return true ;
	}

	bool next_normals(std::vector<Eigen::Vector3d>& normals) override	
	{
		normals.clear() ;

		static std::map<int, std::string>::iterator it = normals_files_.begin() ;

		if(it == normals_files_.end())
		{
			printf("reached end of file \n") ;
			return false ;
		}

		std::string file_name = it->second ;
		std::cout << "===> reading " << file_name << std::endl ;
		if(!normals_reader_.open(file_name))
		{
			printf("Failed to read covariance file %s \n", file_name.c_str()) ;
			return false ;
		}
		
		normals_reader_.next_line() ; // skip header
		
        std::vector<std::string> tokens ;
		while(normals_reader_.next_raw_tokens(tokens))
		{
            Eigen::Vector3d normal ;
            normal[0] = std::stod(tokens[0]) ; 
            normal[1] = std::stod(tokens[1]) ; 
            normal[2] = std::stod(tokens[2]) ; 
			normals.push_back(normal) ;
		} 
	
		normals_reader_.close() ;
	
		++it ;

		return true ;
	}

} ;

#endif 
