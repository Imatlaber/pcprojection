/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne packet to cloud
 */

#include <boost/program_options.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/had/byteDataArray_t.hpp"
#include "lcmtypes/had/pointcloud_t.hpp"
#include "../../../utilities/hadif/param.hpp"
#include "../../../utilities/hadif/trans.hpp"
#include "pcl_point_types.h"
#include "velodyne_driver.hpp"
#include "pc_type_translator.hpp"

std::string channel_name_;
VelodyneInput *vi_;
bool use_device_time_;
int offset_angle_degree_;


class VelodynePacketHandler
{
private:
    lcm::LCM * lcm_;
    param_t param_;
    trans_t trans_;

public:
    VelodynePacketHandler(lcm::LCM * lcm):
        lcm_(lcm),
        param_(lcm),
        trans_(lcm->getUnderlyingLCM(),&param_)
    {}

    void VelodynePacketArrayCallback(const lcm::ReceiveBuffer* /*rbuf*/, const std::string& /*channel*/,
                                     const had::byteDataArray_t* velo_packets)
    {
        pcl::PointCloud<PCLVelodyne> onerev_pc;
        vi_->getOneRevPointCloud(velo_packets, &trans_, onerev_pc, offset_angle_degree_*100, use_device_time_);
		//static int cnt = 0 ;
		//cnt++ ;
		//PCLVelodyne point ;
		//point.x = 1.0 ;
		//point.y = -10.0 ;
		//point.z = 0.0 ;
		//point.distance  = 12.0 ;
		//point.intensity = 0.5 ;
		//point.ring = 1 ;
		//point.mdegree = 0 ;
		//printf("====================\n") ;
		//if(cnt%2 == 0)
		//	point.y = 10.0 ;
		//onerev_pc.points.push_back(point) ;
		//onerev_pc.width = 1 ;
		//onerev_pc.height = 1 ;
        if(onerev_pc.size()>0)
        {
            had::pointcloud_t had_pc;
            fromPCLVelodyne(onerev_pc, had_pc);
			//printf("number of bytes %zd \n", had_pc.data.size()) ;
			//printf("width %d height %d size %d number_of_fields %d point_step %d row_step %d \n", had_pc.width, had_pc.height, had_pc.size, had_pc.num_fields, had_pc.point_step, had_pc.row_step) ;
			//for(auto &f : had_pc.fields)
			//{
			//	printf("name %s offset %d count %d datatype %d \n ", f.name.c_str(), f.offset, f.count, f.datatype) ;
			//}
            had_pc.coord_frame = velo_packets->coord_frame;
            lcm_->publish(channel_name_, &had_pc);
        }
    }
};

namespace po = boost::program_options;
int main(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    po::variables_map vm;

    desc.add_options()
            ("help,h", "Produce this message")
            ("type,t", po::value<std::string>(), "Velodyne type (VLP16, VLP16HD, HDL32)")
            ("channel,c", po::value<std::string>(), "Channel name")
            ("dtime,d", po::value<bool>(), "Use device timestamp. Default: on")
            ("offset,o", po::value<int>(), "Velodyne zero offset in deg. Default: 180");

    po::store(po::parse_command_line(argc, argv, desc), vm);

    if(vm.count("type")==0 || vm.count("channel")==0 || vm.count("help"))
    {
        std::cout<<desc<<std::endl;
        return 1;
    }
    channel_name_ = vm["channel"].as<std::string>();
    std::string velo_type{vm["type"].as<std::string>()};
    if(velo_type == "VLP16")
        vi_ = new VelodyneInput(new VLP16Calibration());
    else if(velo_type == "VLP16HD")
        vi_ = new VelodyneInput(new VLP16HDCalibration());
    else if(velo_type == "HDL32")
        vi_ = new VelodyneInput(new HDL32Calibration());
    else
    {
        std::cout << "Invalid model, only VLP16/HD and HDL32 are supported" << std::endl;
        return -1;
    }

    use_device_time_ = true;
    offset_angle_degree_ = 180;

    if(vm.count("dtime")) use_device_time_ = vm["dtime"].as<bool>();
    if(vm.count("offset")) offset_angle_degree_ = vm["offset"].as<int>();

    std::cout<<"Velodyne type: "<<velo_type<<" channel: "<<channel_name_<<" use device time: "<<use_device_time_<<" offset angle: "<<offset_angle_degree_<<std::endl;

    lcm::LCM lcm;
    VelodynePacketHandler vph(&lcm);
    lcm.subscribe(std::string(channel_name_ + "_raw"), &VelodynePacketHandler::VelodynePacketArrayCallback, &vph);
    while(lcm.handle()==0){}
    return 0;
}
