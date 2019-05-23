/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne driver
 */

#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <thread>
#include "velodyne_driver.hpp"
#include "lcmtypes/had/byteData_t.hpp"
#include "lcmtypes/had/pointcloud_t.hpp"
#include "pcl_point_types.h"
#include "../../utilities/hadif/time_helper.hpp"
#include "pc_type_translator.hpp"
#include "hdl32_driver.hpp"
#include "hdl64_driver.hpp"


lcm::LCM *lcm_;
int velo_heartbeat_ = 1;
bool driver_ok_ = true;
int port_number_;
VelodyneInput *vi_;
std::string channel_, channel_raw_;

enum class VelodyneConnection
{
    Waiting,
    Connected,
    Lost
};

VelodyneConnection connect_stat_;

void publishPoints(pcl::PointCloud<PCLVelodyne> vp)
{
    had::pointcloud_t had_pc;
    fromPCLVelodyne(vp, had_pc);
    switch(connect_stat_)
    {
    case VelodyneConnection::Connected:
        had_pc.health.status = had::hardware_health_t::OK;
        had_pc.health.message = "Velodyne connected";
        break;
    case VelodyneConnection::Lost:
        had_pc.health.message = "Velodyne connection lost";
        had_pc.health.status = had::hardware_health_t::ERROR;
        break;
    case VelodyneConnection::Waiting:
        had_pc.health.message = "Waiting for Velodyne connection";
        had_pc.health.status = had::hardware_health_t::STALE;
        break;
    }
    had_pc.health.header.sender = channel_;
    had_pc.health.header.utime = bot_timestamp_now();

    had_pc.coord_frame = channel_;
    lcm_->publish(channel_, &had_pc);
}



void VeloMon()
{
    Stopwatch velo_mon("VeloMon");
    connect_stat_ = VelodyneConnection::Waiting;
    std::cout << std::fixed<<std::setprecision(1)<<std::setfill('0');
    while(driver_ok_)
    {
        double last_connected_sec = velo_mon.toc();
        switch (connect_stat_)
        {
        case VelodyneConnection::Waiting:
            if(port_number_>0)
                std::cout<<"Waiting for connection. Listening to port "<<port_number_<<" "<<last_connected_sec<<"s    \xd"<<std::flush;
            if(velo_heartbeat_==0)
            {
                connect_stat_ = VelodyneConnection::Connected;
                std::cout<<std::endl<<"Velodyne connected. "<<last_connected_sec<<"s"<<std::endl;
            }
            velo_heartbeat_++;
            break;
        case VelodyneConnection::Connected:
            if(velo_heartbeat_>5)
                connect_stat_ = VelodyneConnection::Lost;
            else
            {
                velo_mon.tic();
                velo_heartbeat_++;
            }
            break;
        case VelodyneConnection::Lost:
            std::cout<<"Connection lost since "<<last_connected_sec<<"s    \xd"<<std::flush;
            if(velo_heartbeat_==0)
            {
                std::cout<<std::endl<<"Velodyne connection reestablished"<<std::endl;
                connect_stat_ = VelodyneConnection::Connected;
                velo_heartbeat_++;
            }
            break;
        }

        usleep(100000);
    }
}

int main(int argc, char** argv)
{
    if(argc==4)
    {
        if(strcmp(argv[1], "VLP16")!=0 && strcmp(argv[1], "HDL32")!=0 && strcmp(argv[1], "HDL64")!=0)
        {
            std::cout << "Invalid model, only VLP16, HDL32 and HDL64 are supported" << std::endl;
            exit(1);
        }

        std::istringstream ss(argv[2]);
        if (!(ss >> port_number_))
        {
            if(strcmp(argv[1], "VLP16")==0)
                vi_ = new VelodyneInput(argv[2], new VLP16Calibration());
            else if(strcmp(argv[1], "HDL32")==0)
                vi_ = new HDL32Input(argv[2] , new HDL32Calibration());
            else if(strcmp(argv[1], "HDL64")==0)
                vi_ = new HDL64Input(argv[2] , new HDL64Calibration());
            port_number_ = -1;
            std::cout << "Opening driver from file: " << argv[2] << std::endl;
        }
        else
        {
            if(strcmp(argv[1], "VLP16")==0)
                vi_ = new VelodyneInput(port_number_, new VLP16Calibration());
            else if(strcmp(argv[1], "HDL32")==0)
                vi_ = new HDL32Input(port_number_, new HDL32Calibration());
            else if(strcmp(argv[1], "HDL64")==0)
                vi_ = new HDL64Input(port_number_, new HDL64Calibration());
            std::cout << "Opening driver with port: " << port_number_ << std::endl;
        }
        channel_ = argv[3];
        channel_raw_ = channel_ + "_raw";
    }
    else
    {
        std::cout<<"velodyne_driver VLP16/HDL32 2368/PCAP channel"<<std::endl;
        exit(2);
    }

    lcm_ = new lcm::LCM();

    while ( !lcm_->good() )
    {
        sleep(1);
        std::cout << "Waiting for LCM connection..."<< std::endl;
    }
    std::vector<uint8_t> data;

    double sensor_sim_freq = 10.0;
    long sim_usleep = 1/sensor_sim_freq*1e6;

    // TODO sw is only used if vi_ is file based => probably, sw should be relocated to VelodyneInputPcap
    Stopwatch sw("Velodyne time");
    std::thread velodyne_monitor_thread(VeloMon);

    pcl::PointCloud<PCLVelodyne> velodyne_onerev_pc;
    while(vi_->getPacket(data))
    {
        had::byteData_t raw_data;
        raw_data.utime = bot_timestamp_now();
        raw_data.size = data.size();
        raw_data.data = data;
        //lcm_->publish(channel_raw_, &raw_data);
        std::vector<pcl::PointCloud<PCLVelodyne>> velodyne_pointclouds;
        int single_rev_idx = vi_->decodePacket(data, velodyne_pointclouds);
		
        
		if(single_rev_idx>=0)
        {
            VelodyneType vt_detected;
            int freq;
            if(vi_->determineVelodyneType(vt_detected, freq))
            {
                VelodyneType vt = vi_->getInitializedType();
                if(vt!=vt_detected)
                {
                    std::cout<<"Warning: detected ";
                    switch(vt_detected)
                    {
                    case VelodyneType::VLP16:
                        std::cout<<"VLP16 but using HDL32 driver"<<std::endl;
                        break;
                    case VelodyneType::HDL32:
                        std::cout<<"VLP32 but using VLP16 driver"<<std::endl;
                        break;
                    case VelodyneType::None:
                        std::cout<<"unspecified velodyne type"<<std::endl;
                        break;
                    }
                }
                sim_usleep = 1.0/freq*1e6;
            }
			printf("spin once \n") ;

            if(vi_->isFile())
            {
                long last_time_used = sw.utoc();
                long real_usleep = sim_usleep - (last_time_used);
                // non-positive sleep times will cause Thread::usleep to return immediately
				if(real_usleep > 0)
					usleep(real_usleep);
				else
				{
					printf("elapsed time per spin %g \n", last_time_used/1e6) ;
				}
            }
            sw.tic();

            for(int idx=0; idx<single_rev_idx; idx++)
                velodyne_onerev_pc+=velodyne_pointclouds[idx];
            velodyne_onerev_pc.height = vi_->getVelodyneRing();
            velodyne_onerev_pc.width = velodyne_onerev_pc.size()/vi_->getVelodyneRing();
            velodyne_onerev_pc.is_dense = true;

            publishPoints(velodyne_onerev_pc);
            velodyne_onerev_pc.clear();

            velodyne_onerev_pc.header.stamp = bot_timestamp_now();
			
            for(size_t idx = single_rev_idx; idx < velodyne_pointclouds.size(); ++idx)
                velodyne_onerev_pc += velodyne_pointclouds[idx];

            velo_heartbeat_ = 0;
        }
        else
        {
            for(auto velo_pc : velodyne_pointclouds)
                velodyne_onerev_pc+=velo_pc;
        }
    }
    driver_ok_ = false;
    velodyne_monitor_thread.join();
    return 0;
}
