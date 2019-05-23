/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne HDL64 driver
 */

#ifndef _HDL64_DRIVER_HPP_
#define _HDL64_DRIVER_HPP_

#include <assert.h>
#include <cmath>
#include "velodyne_driver.hpp"
#include "../../hadif/math_helper.hpp"
#include "../../hadif/time_helper.hpp"
#include "pcl_point_types.h"
#include "velodyne_driver.hpp"

/*
 * HDL64
 */

class HDL64Input : public VelodyneInput
{
	public:
    
	HDL64Input(VelodyneCalibration * v_cal);

    HDL64Input(uint16_t udp_port, VelodyneCalibration * v_cal);

    HDL64Input(std::string filename, VelodyneCalibration * v_cal);

    int decodePacket(const std::vector<uint8_t> &data,
                    std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
                     int offset_angle_two_decimal = 0.0, 
					 int64_t timestamp=0) override ;
	
    void decodeSingleBlock(const raw_block& block0, 
						   const float total_azimuth, 
						   std::vector< pcl::PointCloud<PCLVelodyne> > &pcl) override;
	

} ;

HDL64Input::HDL64Input(VelodyneCalibration * v_cal) 
	: VelodyneInput(v_cal)
{
}

HDL64Input::HDL64Input(uint16_t udp_port, VelodyneCalibration * v_cal) 
	: VelodyneInput(udp_port, v_cal)
{
}

HDL64Input::HDL64Input(std::string filename, VelodyneCalibration * v_cal) 
	: VelodyneInput(filename, v_cal)
{
}

int HDL64Input::decodePacket(const std::vector<uint8_t> &data,
                    std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
                     int offset_angle_two_decimal,
					 int64_t timestamp) 
{
    out_velodyne_pointclouds.clear();
    int single_rev_idx = -1;
    const raw_packet_t *raw = reinterpret_cast<const raw_packet_t *>(&data[0]);
    float azimuth_diff;
    float last_azimuth_diff=0.0;

    //Use device timestamp when it is not provided
    if(timestamp == 0)
        timestamp = updateTypeAndTimestamp(raw);

    //Making sure all the blocks in single packet is valid
    for (int block = 0; block < BLOCKS_PER_PACKET; block++)
    {
        if(0xEEFF == raw->blocks[block].header)
        {
			//printf("+++header is Ox%x \n", raw->blocks[block].header) ;
        }
		else if(0xDDFF == raw->blocks[block].header)
		{
			//printf(">>>header is Ox%x \n", raw->blocks[block].header) ;
		}
		else
		{
			printf("!!!wrong header \n") ;
            return -1;
		}
    }

    int block_offset = 2;

    for (int block = 0; block < BLOCKS_PER_PACKET; block+=block_offset)
    {
		int firing = block/block_offset ;
        float azimuth = (float)(addOffset(raw->blocks[block].rotation, offset_angle_two_decimal));
        if (block < (BLOCKS_PER_PACKET-block_offset))
        {
            azimuth_diff = (float)(addOffset(raw->blocks[block+block_offset].rotation, raw->blocks[block].rotation));
            last_azimuth_diff = azimuth_diff;
        }
        else
        {
            azimuth_diff = last_azimuth_diff;
        }
        float cur_azimuth_diff = azimuth - last_azimuth_;

        //Velodyne produces repeated entry in return_mode = dual
        if(cur_azimuth_diff < 0)
        {
            double velo_timestamp = timestamp*1e-6;
            if(last_timestamp_>0)
                velo_freq_ = std::round(1.0/(velo_timestamp - last_timestamp_));

            last_timestamp_ = velo_timestamp;
            single_rev_idx = firing;
        }
        last_azimuth_ = azimuth;
        std::vector<pcl::PointCloud<PCLVelodyne>> decoded_pcl;
		
        decodeSingleBlock(raw->blocks[block], azimuth_diff, decoded_pcl);

        //each single block might contain 2 firings, as reported from vector size of a decoded_pcl
		if(decoded_pcl.size() > 0)
		{
			decoded_pcl[0].header.stamp = timestamp + firing * v_cal_->blockTDuration() ;
			out_velodyne_pointclouds.push_back(decoded_pcl[0]);
			for(PCLVelodyne & point : decoded_pcl[0].points)
			{
				//printf("ring %d distance %g intensity %g azimuth %d x %g y %g z %g \n", point.ring, point.distance, point.intensity, point.mdegree, point.x, point.y, point.z) ;
			}
		}
    }
    return single_rev_idx;
}

void HDL64Input::decodeSingleBlock(const raw_block & upper_block, 
									  float total_azimuth, 
									  std::vector< pcl::PointCloud<PCLVelodyne> > &pcl)
{
    float azimuth = (float)upper_block.rotation;
	
    pcl.resize(v_cal_->firingsPerBlock());
    for(auto &pc : pcl)
	{
		//printf("scans per firing is %d \n", v_cal_->scansPerFiring()*2) ;
        pc.points.resize(v_cal_->scansPerFiring()*2);
	}
    
	const raw_block * block_ptr = &upper_block ; 
	for (int i=0; i<2; ++i)
    {
		const raw_block & block = *block_ptr ;
        for (int dsr=0, k=0; dsr < v_cal_->scansPerFiring(); dsr++, k+=RAW_SCAN_SIZE)
        {
            VelodyneCal corrections = (*v_cal_)[dsr+i*32];
            /** Position Calculation */
            union two_bytes tmp;
            tmp.bytes[0] = block.data[k];
            tmp.bytes[1] = block.data[k+1];

            /** correct for the laser rotation as a function of timing during the firings **/
            //azimuth reading is available at every new block
            float azimuth_corrected_f = azimuth + (total_azimuth*v_cal_->getFiringTimeRatio(dsr+i*32, 0));
            uint16_t azimuth_corrected = (int)round(fmod(azimuth_corrected_f,36000.0));
            if(azimuth_corrected==36000) azimuth_corrected = 0;
            if(azimuth_corrected>35999)
                std::cout<<"Azimuth_corrected="<<azimuth_corrected<<std::endl;
            assert(azimuth_corrected<36000);

            double distance = tmp.uint * DISTANCE_RESOLUTION;

            float cos_vert_angle = corrections.vert_cos_table;
            float sin_vert_angle = corrections.vert_sin_table;


            float cos_rot_angle = cos_table[azimuth_corrected];
            float sin_rot_angle = sin_table[azimuth_corrected];

            // Compute the distance in the xy plane (w/o accounting for rotation)
            float xy_distance = distance * cos_vert_angle;
            float x = xy_distance * sin_rot_angle;
            float y = xy_distance * cos_rot_angle;
            float z = distance * sin_vert_angle;


            /** Use right hand rule coordinate system */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;

            /** Intensity Calculation */
            float intensity = block.data[k+2];
			
            // convert polar coordinates to Euclidean XYZ
            PCLVelodyne point;
            point.distance = distance;
            point.ring = corrections.ring;
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = intensity;
            // append this point to the cloud
			//printf("ring %d distance %g intensity %g x %g y %g z %g \n", point.ring, point.distance, point.intensity, point.x, point.y, point.z) ;
            pcl[0].points[point.ring] = point;
        }
		++block_ptr ;
    }
}

#endif
