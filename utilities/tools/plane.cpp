/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: point cloud publisher from reading a file
 */

#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <bot_core/small_linalg.h>
#include <bot_core/bot_core.h>
#include <bot_core/trans.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/trans.hpp"
#include "lcmtypes/had/pointcloud_t.hpp"
#include "../../utilities/drivers/velodyne/pcl_point_types.h"
#include "../../utilities/drivers/velodyne/pc_type_translator.cpp"
#include "../../utilities/hadif/time_helper.hpp"
#include "../../utilities/hadif/reader_helper.hpp"

struct LidarPoint
{
	double pos[3] ;
	double intensity ;
	double laser_id ;
	double azimuth ; //  [0, 36000)
	double distance ; // m
	double adjusted_time ;	
	double time ; // us
} ;

bool get_plane(pcl::PointCloud<PCLVelodyne>::Ptr pc, pcl::PointCloud<PCLVelodyne>& pc_in)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
	std::vector<int> cloud_indices ;
	for(size_t i=0; i<pc->points.size(); ++i)
	{
		const auto& p = pc->points[i] ;

		double psi = atan2(p.y, p.x) ;
		if(psi > 0 && psi < M_PI/2)
		{
			cloud->points.emplace_back(pc->points[i].x, pc->points[i].y, pc->points[i].z) ;
			cloud_indices.push_back(i) ;
		}
	}
	cloud->width = cloud->points.size() ;
	cloud->height = 1 ;

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients) ;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices) ;

	pcl::SACSegmentation<pcl::PointXYZ> seg ;
	seg.setModelType(pcl::SACMODEL_PLANE) ;
	seg.setMethodType(pcl::SAC_RANSAC) ;
	seg.setDistanceThreshold(0.01) ;
	seg.setInputCloud(cloud) ;
	seg.segment(*inliers, *coeff) ;

	if(inliers->indices.size() == 0)
	{
		printf("ransac failed \n") ;
		return false ;
	}

	std::cout << "plane coefficients " 
			  << coeff->values[0] << " "
			  << coeff->values[1] << " "
			  << coeff->values[2] << " "
			  << coeff->values[3] << " " << std::endl ;


	double normal[3] = {coeff->values[0], coeff->values[1], coeff->values[2] } ;
	double mag = sqrt(normal[0]*normal[0]+ normal[1]*normal[1]+normal[2]*normal[2]) ;
	bot_vector_scale_3d(normal, 1/mag) ;

	double theta = asin(normal[0])  ;
	double phi = atan2(-normal[1], normal[2]) ;
	
	std::cout << "normal coefficients " 
			  << normal[0] << " "
			  << normal[1] << " "
			  << normal[2] << " "  << std::endl ;

	double normal_recal[3] = {sin(theta), -cos(theta)*sin(phi), cos(theta)*cos(phi)} ;
	std::cout << "normal coefficients recaluclated " 
			  << normal_recal[0] << " "
			  << normal_recal[1] << " "
			  << normal_recal[2] << " "  << std::endl ;

	std::cout << "make sure 'normal' and 'normal recalulated' are the same"
			  << std::endl ;

	for(size_t i=0; i<inliers->indices.size(); ++i)
	{
		pc_in.push_back(pc->points[cloud_indices[inliers->indices[i]]]) ;
	}

	double rpy[3] = {-phi, -theta, 0} ;
	double quat[4] ;
	
	bot_roll_pitch_yaw_to_quat(rpy, quat) ;
	double rot[9] ;
	bot_quat_to_matrix(quat, rot) ;
	printf("----------\n") ;
	printf("%f %f %f \n", rot[0], rot[1], rot[2]) ;
	printf("%f %f %f \n", rot[3], rot[4], rot[5]) ;
	printf("%f %f %f \n", rot[6], rot[7], rot[8]) ;
	
	BotTrans trans ;	
	memcpy(trans.rot_quat, quat, sizeof(quat)) ;
	memset(trans.trans_vec, 0, sizeof(double[3])) ;

	//for(auto& p : pc_in.points)
	//{
	//	double v[3] = {p.x, p.y, p.z} ;
	//	double r[3] ;
	//	bot_trans_apply_vec(&trans, v, r) ;
	//	p.x = r[0] ;
	//	p.y = r[1] ;
	//	p.z = r[2] ;
	//}

	double factor = 180/M_PI ;
	phi = phi*factor ;
	theta = theta* factor ;
	std::cout << "phi " << -phi << " "
			  << "theta " << -theta << " " << std::endl ;

	
	return true ;
}

inline bool check_onerev(const LidarPoint& point)
{
	int64_t curr_mdegree = static_cast<int64_t>(point.azimuth) ;
	static int64_t last_mdegree = curr_mdegree ;
	//printf("curr_degree %ld last_degree %ld \n", curr_mdegree, last_mdegree) ;
	if(curr_mdegree-last_mdegree < 0)
	{
		last_mdegree = curr_mdegree ;
		return true ;
	}
	last_mdegree = curr_mdegree ;

	return false ;
}

void publish_onerev_pc(lcm::LCM * lcm, pcl::PointCloud<PCLVelodyne>& pc, std::string channel)
{
	had::pointcloud_t had_pc ;
	fromPCLVelodyne(pc, had_pc) ;
	had_pc.health.header.sender = "VELO_P2" ;
	had_pc.health.header.utime = pc.header.stamp ;

	had_pc.header.utime = pc.header.stamp ;
	had_pc.coord_frame = "VELO_P2" ;
	lcm->publish(channel, &had_pc) ;	
}

namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    po::variables_map vm;
	
	desc.add_options()
		("help,h", "help message")
		("file,f", po::value<std::string>(), "name of pointcloud file")
		("sleep,s", po::value<double>(), "sleep time for one revolution of points (s). Default: 1s")
		("dir,d", po::value<std::string>(), "directory of pointcloud files (overwrite --file option)");
	po::store(po::parse_command_line(argc, argv, desc), vm) ;

	if((vm.count("file")==0 && vm.count("dir")==0) || vm.count("help"))
	{
		std::cout << desc << std::endl ;
		return 1 ;
	}
	
	std::string pc_dir ;
	std::map<int, std::string> pc_files ;
	std::string pc_file("pc.cvs") ;

	if(vm.count("dir"))
	{
		pc_dir = vm["dir"].as<std::string>() ;
		if(fs::is_directory(pc_dir))
		{
			for(fs::directory_iterator di(pc_dir); di != fs::directory_iterator(); ++di)
			{
				if(di->path().extension() == ".csv")
				{
					std::string path = di->path().string() ;
					std::string index_str = path.substr(path.length()-9, 4) ;
					int index = atoi(index_str.c_str()) ;
					pc_files[index] = path ;
				}
				//std::cout << "<< file " << pc_files.back() << std::endl ;
			}
		}
		else
		{
			printf("Error: %s is not a valid directory\n", pc_dir.c_str()) ;
			return 1 ;
		}
	}
	else if(vm.count("file"))
	{
		pc_file = vm["file"].as<std::string>() ;
		pc_files[0] = pc_file ;
	}

	int64_t onerev_sleep = 1000000 ; // us
	if(vm.count("sleep"))
	{
		double ssleep = vm["sleep"].as<double>() ;
		onerev_sleep = static_cast<int64_t>(ssleep * 1e6) ;
	}

	lcm::LCM lcm ;
	param_t param(&lcm) ;
	trans_t had_trans(lcm.getUnderlyingLCM(), &param) ;
	
	Stopwatch sw("velodyne") ;
	pcl::PointCloud<PCLVelodyne>::Ptr velodyne_onerev_pc(new pcl::PointCloud<PCLVelodyne>) ;
	int64_t last_utime = 0  ;
	for(const auto& file : pc_files)
	{
		std::string file_name = file.second ;
		std::cout << "===> reading " << file_name << std::endl ;
		CSVReader<LidarPoint> reader(9) ;
		if(!reader.open(file_name))
		{
			printf("Failed to read lidar file %s \n", file_name.c_str()) ;
			return 1 ;
		}
		
		reader.next_line() ; // skip header
		
		LidarPoint point ;
		while(reader >> point)
		{
			// temporary fix for lidar timeoffset
			//point.time /= 1e6 ;
			//point.time += 1481274000 ;	

			PCLVelodyne pp ;
			pp.x = point.pos[0] ;
			pp.y = point.pos[1] ;
			pp.z = point.pos[2] ;
			pp.distance = point.distance ;
			pp.intensity = point.intensity ;
			pp.ring = (int)point.laser_id ;
		
			int64_t utime = static_cast<int64_t>(point.time*1e6) ;
			memcpy(&pp.utime1, &utime, sizeof(int64_t)) ;
			velodyne_onerev_pc->points.push_back(pp) ;
			last_utime = utime ;
		}	
		{
			pcl::PointCloud<PCLVelodyne> pc_in ;
			velodyne_onerev_pc->header.stamp = last_utime ;					
			publish_onerev_pc(&lcm, *velodyne_onerev_pc, "VELO") ;
			std::string input ;
			if(get_plane(velodyne_onerev_pc, pc_in))
			{
				pc_in.header.stamp = last_utime ;
				publish_onerev_pc(&lcm, pc_in, "inliers") ;
				std::cout << "Please any key for next frame " << std::endl ;
				std::getline(std::cin, input) ;
			}
			std::cout << "onerev pointcloud published: " << velodyne_onerev_pc->points.size() << " points" << std::endl ;
			velodyne_onerev_pc->clear() ;
			long dutime = sw.utoc() ;
			long sleep_utime = onerev_sleep - dutime ; 
			if(sleep_utime > 0)
			{
				while(sleep_utime > 0)
				{
					long chunk = sleep_utime > 1000000 ? 1000000 : sleep_utime ;
					usleep(chunk) ;
					sleep_utime -= chunk ;
				}
			}
			else
				printf("elapsed time %g greater than 0.1s \n", dutime/1e6) ;
			sw.tic() ;
			//printf("time %.6f x %.6f y %.6f z %.6f distance %.6f intensity %.6f ring %d medgree %d \n", point.time, pp.x, pp.y, pp.z, pp.distance, pp.intensity, pp.ring, pp.mdegree) ;
		}
	}

	return 0 ;
}
