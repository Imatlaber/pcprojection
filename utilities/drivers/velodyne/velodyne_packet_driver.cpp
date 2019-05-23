/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne packet driver
 */

#include <boost/program_options.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <boost/thread.hpp>
#include <bot_core/bot_core.h>
#include "velodyne_udp_receiver.hpp"

lcm::LCM *lcm_;
std::string channel_name_;
int offset_degree_;

class VelodynePacketReceiveHandler : public VelodyneUDPReceiver
{
public:
    VelodynePacketReceiveHandler(int data_port, int position_port,
                                 boost::asio::io_service &data_service,
                                 boost::asio::io_service &pos_service):
        VelodyneUDPReceiver(data_port, position_port, data_service, pos_service, offset_degree_*100)
    {

    }

    void publishVeloPos(had::byteData_t velo_pos) override
    {
        lcm_->publish(std::string(channel_name_+ "_POS"), &velo_pos);
    }

    void publishVeloRaw(had::byteDataArray_t velo_data) override
    {
        velo_data.utime = bot_timestamp_now();
        velo_data.coord_frame = channel_name_;
        lcm_->publish(std::string(channel_name_+"_raw"), &velo_data);
    }
};

namespace po = boost::program_options;
int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    po::variables_map vm;


    desc.add_options()
            ("help,h", "Produce this message")
            ("channel,c", po::value<std::string>(), "Channel name")
            ("dataport,d", po::value<int>(), "Velodyne data port. Default: 2368")
            ("posport,p", po::value<int>(), "Velodyne position port. Default: 8308")
            ("offset,o", po::value<int>(), "Velodyne zero offset in deg. Default: 180");

    po::store(po::parse_command_line(argc, argv, desc), vm);


    if(vm.count("dataport")==0 || vm.count("posport")==0 ||
            vm.count("channel")==0 || vm.count("help"))
    {
        std::cout<<desc<<std::endl;
        return 1;
    }

    lcm_ = new lcm::LCM() ;

    int data_port = 2368;
    int position_port = 8308;
    offset_degree_ = 180;

    if(vm.count("dataport")) data_port = vm["dataport"].as<int>();
    if(vm.count("posport")) position_port = vm["posport"].as<int>();
    if(vm.count("offset")) offset_degree_ = vm["offset"].as<int>();

    channel_name_ = vm["channel"].as<std::string>();

    std::cout<<"Velodyne channel: "<<channel_name_<<" data port: "<<data_port<<" position port: "<<position_port<<" offset: "<<offset_degree_<<std::endl;

    boost::asio::io_service data_service, pos_service;
    VelodynePacketReceiveHandler data_udp(data_port, position_port, data_service, pos_service);

    boost::thread_group threads;
    threads.create_thread(boost::bind(&boost::asio::io_service::run, &data_service));
    pos_service.run();
    threads.join_all();


    return 0;
}
