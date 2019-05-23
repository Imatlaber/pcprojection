/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: publish ODOM message from a file 
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
#include <bot_core/rotations.h>

struct PoseMat
{
	double mat[16] ;
	double time ;
} ;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
int main(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    po::variables_map vm;
	
	desc.add_options()
		("help,h", "help message")
		("file,f", po::value<std::string>(), "name of relative pose file");
	po::store(po::parse_command_line(argc, argv, desc), vm) ;

	if(vm.count("file")==0 || vm.count("help"))
	{
		std::cout << desc << std::endl ;
		return 1 ;
	}

	std::string in_file("input.txt") ;
	
	if(vm.count("file"))
		in_file = vm["file"].as<std::string>() ;

	CSVReader<PoseMat> reader(17) ;
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
	PoseMat point ;

	double factor = 180.0/3.1415 ;
	double prev_mat[16] = {0} ;
	prev_mat[0] = 1.0 ;
	prev_mat[5] = 1.0 ;
	prev_mat[10] = 1.0 ;
	prev_mat[15] = 1.0 ;
	while(reader >> point)
	{
		double stime = point.time/1e6 ;
		double curr_mat[16] = {0} ;
		bot_matrix_multiply_4x4_4x4(prev_mat, point.mat, curr_mat) ;
		double pos[3] = {0} ;
		pos[0] = curr_mat[3] ;
		pos[1] = curr_mat[7] ;
		pos[2] = curr_mat[11] ;
		double rot[9] =         { curr_mat[0], curr_mat[1], curr_mat[2],	
								  curr_mat[4], curr_mat[5], curr_mat[6],	
							      curr_mat[8], curr_mat[9], curr_mat[10] } ;

		double quat[4] = {0} ;
		bot_matrix_to_quat(rot, quat) ;
		double rpy[3] = {0} ;
		bot_quat_to_roll_pitch_yaw(quat, rpy) ;
		bot_vector_scale_3d(rpy, factor) ;	

		memcpy(prev_mat, curr_mat, sizeof(curr_mat)) ;

		ofs << stime << ", " << pos[0] << ", " << pos[1]  << ", " << pos[2]  << ", " << rpy[0]  << ", " << rpy[1]  << ", " << rpy[2] << std::endl ;
	}
	ofs.close() ;

	return 0 ;
}
