/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: publish ODOM message from a file 
 */

#ifndef _READER_HELPER_HPP_
#define _READER_HELPER_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
// #include <lcm/lcm-cpp.hpp>
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <unistd.h>

template<class T>
class CSVReader
{
public:
	CSVReader(const int num_fields):
		num_fields_(num_fields)
	{

	}

	bool open(std::string name)
	{
		file_.open(name, std::ifstream::in) ;
		if(file_.is_open())
			return true ;
		else
			return false ;
	}

	void close()
	{
		file_.close() ;
	}

	void next_line()
	{
		std::string line ;
		std::getline(file_, line) ;
	}

	std::string get_next_line()
	{
		std::string line ;
		std::getline(file_, line) ;
		return line;
	}

	void next_line(int p)
	{
		std::string line ;
		for(size_t i=0; i<p; ++i)
		{
			std::getline(file_, line) ;
		}
	}

	bool next_line(T& point)
	{
		std::string line ;
		int cnt=0 ;
		if(std::getline(file_, line))
		{
			boost::char_separator<char> sep(", ");
			boost::tokenizer< boost::char_separator<char> > tokens(line, sep) ;
			double * target = (double*)&point ;
			double rmsxyz=0;
			double rmsrpy=0;
			double rms_t;
			for(const auto & t : tokens)
			{
				if(cnt<11){
					*target = atof(t.c_str()) ;
					++target ;
				}
				else if(cnt<14){
					rms_t = atof(t.c_str()) ;
					rmsxyz += rms_t * rms_t;
				}
				else if(cnt>16 && cnt<20){
					rms_t = atof(t.c_str()) * 3.1415926 * 25 / 180;
					rmsrpy += rms_t * rms_t;
				}
				++cnt;
			}
			*target = std::sqrt(rmsxyz);
			++target;
			*target = std::sqrt(rmsrpy);
		}
		else
		{
			printf("End of file \n") ;
			return false ;
		}

		if(cnt != num_fields_)
		{
			std::cout << line << std::endl;
			printf("error: number of fields read %d, expected %d !\n", cnt, num_fields_) ;
			//return false ;
		}

		return true ;
	}

	bool next_raw_tokens(std::vector<std::string>& members)
	{
		members.clear() ;
		std::string line ;
		int cnt=0 ;
		if(std::getline(file_, line))
		{
			boost::char_separator<char> sep(", ") ;
			boost::tokenizer< boost::char_separator<char> > tokens(line, sep) ;
			for(const auto & t : tokens)
			{
				members.push_back(t.c_str()) ;
				++cnt ;
			}
		}
		else
		{
			printf("End of file \n") ;
			return false ;
		}

		if(cnt != num_fields_)
		{
			std::cout << line << std::endl;
			printf("error: number of fields read %d, expected %d !\n", cnt, num_fields_) ;
			//return false ;
		}

		return true ;
	}

	bool operator >> (T& point)
	{
		return next_line(point)  ;
	}

private:
	int num_fields_ ;
	std::ifstream file_ ;
};



#endif //_READER_HELPER_HPP__
