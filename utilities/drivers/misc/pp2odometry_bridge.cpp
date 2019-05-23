/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: post processing from IE bridge
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
#include "../../utilities/hadif/param.hpp"
#include <bot_core/rotations.h>

//GPS Week,GPS Seconds,Latitude,Longitude,Height,North Velocity,East Velocity,Up Velocity,Roll,Pitch,Azimuth,Status
struct IE
{
	double utc_time ;
	double lat ;
	double lon ;
	double height ;
	double roll ;
	double pitch ;
	double heading ;
	double vel_north ;
	double vel_east ;
	double vel_up ;
} ;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    po::variables_map vm;

    // input parameters
    double begin_time=0, end_time=DBL_MAX ;
    int ctype = 1 ;
	desc.add_options()
		("help,h", "help message")
		("type,t", po::value<int>(), "conversion type 0: vanilla, 1: proj (default)")
		("begin_time,b", po::value<double>(), "beginning time (s)")
		("end_time,e", po::value<double>(), "ending time (s)")
		("file,f", po::value<std::string>(), "name of IE file");
	po::store(po::parse_command_line(argc, argv, desc), vm) ;

	if(vm.count("file")==0 || vm.count("help"))
	{
		std::cout << desc << std::endl ;
		return 1 ;
	}

	std::string in_file("input.txt") ;
	
	if(vm.count("file"))
		in_file = vm["file"].as<std::string>() ;

	if(vm.count("begin_time"))
		begin_time = vm["begin_time"].as<double>() ;

	if(vm.count("end_time"))
		end_time = vm["end_time"].as<double>() ;

	if(vm.count("type"))
	    ctype = vm["type"].as<int>() ;

    // config parameters
    lcm::LCM lcm ;
    param_t param(&lcm) ;
    std::vector<double> latlonheight = param.getDoubleArray("origin.latlonheight") ;
    double ref_lat=1e5, ref_lon=1e5, ref_height=1e5;
    if(latlonheight.size() == 3)
    {
        ref_lat = latlonheight[0] ;
        ref_lon = latlonheight[1] ;
        ref_height = latlonheight[2] ;
        printf("===================== \n") ;
        printf("ref_lat %f, ref_lon %f, ref_height %f \n", ref_lat, ref_lon, ref_height) ;
        printf("===================== \n") ;
    }
    else
        printf("warning: missing origin from config file. Use first point as the origin!\n") ;
    
    // read the trajectory file
	CSVReader<IE> reader(9) ;
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
    ofs << "conversion type " << ctype << std::endl ;
    ofs << "origin: longitude, lattitude, height " << std::endl ;

	reader.next_line(18) ; // skip header
	IE point ;

    GpsHelperBase * gps_helper ;
    if(ctype == 1)
    {
        gps_helper = dynamic_cast<GpsHelperBase*>(new GpsHelperProj()) ;
    }
    else
    {
        gps_helper = dynamic_cast<GpsHelperBase*>(new GpsHelperVanilla()) ;
    }
    
	std::vector<std::string> tokens ;
	bool initialized = false ;
    size_t cnt = 0 ;
	while(reader.next_raw_tokens(tokens))
	{
		point.utc_time = std::stod(tokens[0]) ;
        if(point.utc_time < begin_time || point.utc_time > end_time)
            continue ;
        ++cnt ;
		point.lon = std::stod(tokens[1]) ;
		point.lat = std::stod(tokens[2]) ;
		point.height = std::stod(tokens[3]) ;
		point.roll = std::stod(tokens[4]) ;
		point.pitch = std::stod(tokens[5]) ;
		point.heading = std::stod(tokens[6]) ;
		point.vel_north = std::stod(tokens[7]) ;
		point.vel_east = std::stod(tokens[8]) ;
		//point.vel_up = std::stod(tokens[9]) ;
		if(!initialized)
		{
			latlonheight_t origin ;
            if(fabs(ref_lat) > 1e4 || fabs(ref_lon) > 1e4 || fabs(ref_height) > 1e5)
            {
			    origin.lat = point.lat ;
			    origin.lon = point.lon ;
                origin.height = point.height-5 ;
                 ofs << origin.lon << ", " << origin.lat << ", " << origin.height << std::endl ;
	             ofs << "Time[s],X[m],Y[m],Z[m],Roll[deg],Pitch[deg],Yaw[deg]" << std::endl ;
            }
            else
            {
                origin.lat = ref_lat ;
                origin.lon = ref_lon ;
                origin.height = ref_height-5 ;
                ofs << origin.lon << ", " << origin.lat << ", " << origin.height << std::endl ;
	            ofs << "Time[s],X[m],Y[m],Z[m],Roll[deg],Pitch[deg],Yaw[deg]" << std::endl ;
            }
			gps_helper->compute_local_transformation(origin) ;
			initialized = true ;
		}
		double llh[3] = {point.lat, point.lon, point.height} ;
		double pos[3] ;
		gps_helper->latlonheight2local(llh, pos) ;
		double rpy[3] = {point.roll, point.pitch, point.heading} ;
		int64_t utc_usec = point.utc_time*1e6 ;
		ofs << utc_usec << ", " << pos[0] << ", " << pos[1]  << ", " << pos[2]  << ", " << rpy[0]  << ", " << rpy[1]  << ", " << rpy[2] << std::endl ;
	}
	ofs.close() ;

    printf("odometry size [ %zu ] \n", cnt) ;
	return 0 ;
}
















