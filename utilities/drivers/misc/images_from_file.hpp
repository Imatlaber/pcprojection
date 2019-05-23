/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: read images from a file
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
#include <bot_core/rotations.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../../utilities/hadif/reader_helper.hpp"
#include "../../utilities/hadif/time_helper.hpp"
#include "../../utilities/hadif/param.hpp"
#include "../../utilities/hadif/image_helper.hpp"

struct SimpleImage
{
    int64_t utime ;
    cv::Mat image ;
} ;

class ImageReader
{
    public:


    ImageReader()
    {

    }

    ImageReader(lcm::LCM * lcm, param_t * param, std::string name):
        lcm_(lcm),
        param_(param)
    {
        file_.open(name, std::ifstream::in) ;
        if(file_.is_open())
        {
            std::cout << name << " is opened. " << std::endl ;
            open_ = true ;
        }
        else
        {
            std::cout << "cannot open " << name << std::endl ;
            open_ = false ;
        }
    }

    bool next_raw_tokens(std::vector<std::string>& tokens)
    {
        if(!open_)
            return false ;
        
        tokens.clear() ;
        std::string line ;
        if(std::getline(file_, line))
        {
            boost::char_separator<char> sep(", ") ;
            boost::tokenizer< boost::char_separator<char> > items(line, sep) ; 
            for(const auto& item : items)
                tokens.push_back(item.c_str()) ;
        }
        else
        {
            std::cout << "End of file " << std::endl ;
            return false ;
        }

        return true ;
    }

    void parse_date(const std::string& date, int& yy, int& mm, int& dd, int& hh, int& min, int& sec, int& msec)
    {
        yy = std::stoi(date.substr(0,4)) ; 
        mm = std::stoi(date.substr(4,2)) ; 
        dd = std::stoi(date.substr(6,2)) ; 
        hh = std::stoi(date.substr(9,2)) ; 
        min = std::stoi(date.substr(11,2)) ; 
        sec = std::stoi(date.substr(13,2)) ; 
        msec = std::stoi(date.substr(16,3)) ;  
    }

    bool next_image(SimpleImage& img)
    {
        std::vector<std::string> tokens ;
        if(!next_raw_tokens(tokens) || tokens.size() != 2)
        {
            std::cout << "failed to read next image " << std::endl ;
            return false ;
        }
        
        std::string img_date = tokens[0] ;
        std::string img_name = tokens[1] ;
        
        int64_t stime, utime ;
        int yy, mm, dd, hh, min, sec, msec ;
        parse_date(img_date, yy, mm, dd, hh, min, sec, msec) ;
        stime = utc_date2sec(yy, mm, dd, hh, min, sec) ;
        utime = stime * 1e6 + msec * 1e3 ;
     
        //printf("img_date %s img_name %s \n", img_date.c_str(), img_name.c_str()) ;
        //printf("yy mm dd hh min sec msec %d %d %d %d %d %d %d \n", yy, mm, dd, hh, min, sec, msec) ;
        //printf("utc sec %zd \n", stime) ;

        img.utime = utime ;
        img.image = cv::imread(img_name, CV_LOAD_IMAGE_COLOR) ;

        if(!img.image.data)
        {
            std::cout << "cannot find image " << img_name << std::endl ;
            return false ;
        }
        
        return true ;
    }

    void publish_image(const SimpleImage& si)
    {
        had::header_t header ;
        header.utime = si.utime ;
        header.sender = "point_grey" ;
        had::image_t img ;
        ImageHelper::packageMatBgrAsJpegMessage(si.image, 100, header, img) ;
        lcm_->publish("IMAGES", &img) ;
    }

    lcm::LCM * lcm_ ;
    param_t * param_ ;
    std::ifstream file_ ; 
    bool open_ ;


} ;
