/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <baowei.lbw@alibaba-inc.com>
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

namespace po = boost::program_options;
namespace fs = boost::filesystem;

//UTCTime    Longitude     Latitude      H-Ell         Roll        Pitch      Heading     VEast    VNorth
struct Trojectory
{
	double UTCTime ;
	double Longitude ;
	double Latitude ;
	double height ;
	double roll ;
	double pitch ;
	double heading ;
	double VEast ;
	double VNorth ;
} ;

/**
 * cpt:     z   y
 *          |  /   
 * 			| /
 * 			|/_ _x 
 * 
 * rt3000:     x
 *            /   
 * 			 /
 * 			/_ _ _y 
 *          |
 * 			|
 * 			z
 */
int main(int argc, char ** argv)
{
	
	/*double rpyrt[3] = {0.0,0.0,3.1415926/2} ;
	double rpycpt[3] = {rpyrt[0], rpyrt[1]-3.1415926, rpyrt[2]-3.1415926/2.} ;
	double prt[4] = {0.0};
	double pcpt[4] = {0.0};	
	double mrt[9] = {0.0};
	double mcpt[9] = {0.0};
	bot_roll_pitch_yaw_to_quat(rpyrt, prt);
	bot_quat_to_matrix(prt, mrt);
	bot_roll_pitch_yaw_to_quat(rpycpt, pcpt);
	bot_quat_to_matrix(pcpt, mcpt);
	std::cout << mrt[0] << " " << mrt[1] << " " << mrt[2] << std::endl
	<< mrt[3] << " " << mrt[4] << " " << mrt[5] << std::endl
	<< mrt[6] << " " << mrt[7] << " " << mrt[8] << std::endl << std::endl;
	std::cout << mcpt[0] << " " << mcpt[1] << " " << mcpt[2] << std::endl
	<< mcpt[3] << " " << mcpt[4] << " " << mcpt[5] << std::endl
	<< mcpt[6] << " " << mcpt[7] << " " << mcpt[8] << std::endl;
	*/			  
				  
	po::options_description desc("Allowed options");
    po::variables_map vm;

    // input parameters   
	desc.add_options()
		("help,h", "help message")
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
 // read the trajectory file
	CSVReader<Trojectory> reader(9) ;
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

	GpsHelperBase * gps_helper ;
	gps_helper = dynamic_cast<GpsHelperBase*>(new GpsHelperProj()) ;
	
	Trojectory point ;
	std::vector<std::string> tokens ;
	bool initialized = false ;
    size_t cnt = 0 ;
	for(int i = 0; i < 18; ++ i)
	{
		std::string nextline = reader.get_next_line();
		ofs << nextline << std::endl;
	}
	while(reader.next_raw_tokens(tokens))
	{
		point.UTCTime = std::stod(tokens[0]) ;
		point.Longitude = std::stod(tokens[1]) ;
		point.Latitude = std::stod(tokens[2]) ;
		point.height = std::stod(tokens[3]) ;
		point.roll = std::stod(tokens[4]) ;
		point.pitch = std::stod(tokens[5]) ;
		point.heading = std::stod(tokens[6]) ;
		point.VEast = std::stod(tokens[7]) ;
		point.VNorth = std::stod(tokens[8]) ;
		//point.vel_up = std::stod(tokens[9]) ;
		
		
		point.pitch += 180 ;
		point.heading += 90 ;
		
		char str_roll[200];
		sprintf(str_roll, "%.10f", point.roll);
		char str_pitch[200];
		sprintf(str_pitch, "%.10f", point.pitch);
		char str_heading[200];
		sprintf(str_heading, "%.8f", point.heading);
		

		ofs << tokens[0] << " " << tokens[1]
		<< " " << tokens[2]  << "  " << tokens[3]  
		<< " " << str_roll  << " " << str_pitch  
		<< " " << str_heading << " " <<tokens[7] 
		<< " " << tokens[8] << std::endl ;
		++cnt ;
	}
	ofs.close() ;

    printf("odometry size [ %zu ] \n", cnt) ;
	
	return 1;
}