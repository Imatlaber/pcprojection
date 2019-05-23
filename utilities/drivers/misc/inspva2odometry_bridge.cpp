/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: inspva bridge
 */

#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <bot_core/small_linalg.h>
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/trans.hpp"
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <unistd.h>
#include "../../utilities/hadif/reader_helper.hpp"
#include "../../utilities/hadif/gps_helper.hpp"
#include "../../utilities/hadif/math_helper.hpp"
#include "../../utilities/hadif/time_helper.hpp"
#include <bot_core/rotations.h>

//GPS Week,GPS Seconds,Latitude,Longitude,Height,North Velocity,East Velocity,Up Velocity,Roll,Pitch,Azimuth,Status
struct INSPVA
{
	double gps_week ;
	double gps_second ;
	double lat ;
	double lon ;
	double height ;
	double vel_north ;
	double vel_east ;
	double vel_up ;
	double roll ;
	double pitch ;
	double yaw ;
	std::string status ;
} ;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    po::variables_map vm;
	
	desc.add_options()
		("help,h", "help message")
		("file,f", po::value<std::string>(), "name of INSPVA file");
	po::store(po::parse_command_line(argc, argv, desc), vm) ;

	if(vm.count("file")==0 || vm.count("help"))
	{
		std::cout << desc << std::endl ;
		return 1 ;
	}

	std::string in_file("input.txt") ;
	
	if(vm.count("file"))
		in_file = vm["file"].as<std::string>() ;

	CSVReader<INSPVA> reader(12) ;
	if(!reader.open(in_file))
	{
		printf("Failed to read input file %s \n", in_file.c_str()) ;
		return 1 ;
	}

	fs::path in_path(in_file) ;	
	std::string out_file = in_path.remove_leaf().string() + "/Trajectory.txt" ;
	std::ofstream ofs ;
	ofs.precision(10) ;
	ofs.open(out_file, std::ios::out) ;
	ofs << "Time[s],X[m],Y[m],Z[m],Roll[deg],Pitch[deg],Yaw[deg]" << std::endl ;

	reader.next_line() ; // skip header
	INSPVA point ;

    GpsHelperBase * gps_helper ;
    gps_helper = dynamic_cast<GpsHelperBase*>(new GpsHelperVanilla()) ;
    
	latlonheight_t origin ;
	origin.lat = 40.201834 ;
	origin.lon = 116.250286 ;
    origin.height = 0 ;
	
    gps_helper->compute_local_transformation(origin) ;
	std::vector<std::string> tokens ;
	while(reader.next_raw_tokens(tokens))
	{
		point.gps_week = std::stod(tokens[0]) ;
		point.gps_second = std::stod(tokens[1]) ;
		point.lat = std::stod(tokens[2]) ;
		point.lon = std::stod(tokens[3]) ;
		point.height = std::stod(tokens[4]) ;
		point.vel_north = std::stod(tokens[5]) ;
		point.vel_east = std::stod(tokens[6]) ;
		point.vel_up = std::stod(tokens[7]) ;
		point.roll = std::stod(tokens[8]) ;
		point.pitch = std::stod(tokens[9]) ;
		point.yaw = std::stod(tokens[10]) ;
		point.status.assign(tokens[11], 0, 17)  ; 
		if(point.status.compare("INS Solution Good") != 0)
			continue ;
		double llh[3] = {point.lat, point.lon, point.height} ;
		double pos[3] ;
		gps_helper->latlonheight2local(llh, pos) ;
		double rpy[3] = {point.roll, point.pitch, point.yaw} ;
		
		int64_t utc_sec = utc_gps2sec((int)point.gps_week, (int)point.gps_second) ;
		int64_t utc_usec = (utc_sec + point.gps_second-(int)point.gps_second)*1e6 ;

		ofs << utc_usec << ", " << pos[0] << ", " << pos[1]  << ", " << pos[2]  << ", " << rpy[0]  << ", " << rpy[1]  << ", " << rpy[2] << std::endl ;
	}
	ofs.close() ;
	return 0 ;
}
