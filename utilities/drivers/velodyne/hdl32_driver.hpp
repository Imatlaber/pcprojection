/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne HDL32 driver
 */

#ifndef _HDL32_DRIVER_HPP_
#define _HDL32_DRIVER_HPP_

#include <assert.h>
#include <cmath>
#include "velodyne_driver.hpp"
#include "../../hadif/math_helper.hpp"
#include "../../hadif/time_helper.hpp"
#include "pcl_point_types.h"
#include "velodyne_driver.hpp"

/*
 * HDL32
 */

class HDL32Input : public VelodyneInput
{
	public:

	HDL32Input(VelodyneCalibration * v_cal) 
		: VelodyneInput(v_cal)
	{
	}
	
	HDL32Input(uint16_t udp_port, VelodyneCalibration * v_cal) 
		: VelodyneInput(udp_port, v_cal)
	{
	}
	
	HDL32Input(std::string filename, VelodyneCalibration * v_cal) 
		: VelodyneInput(filename, v_cal)
	{
	}

	bool decode_packet_pep(const std::vector<uint8_t> &data, int64_t& utime) override
	{
		char * gprmc_string = (char*)&data[206]; 

		char latStr[32], lonStr[32];
		char dateStr[32];
		double Lat_v,Lon_v;
		char sn[2], ew[2], valid[2];
		int hh, mm, ss;
		int day, mon, year;

		int Lat_deg, Lon_deg;
		double Lat_mi,Lon_mi;
		double Latd, Lond;
		double speed, heading;

		int i;

		// Change all commas to spaces
		for(i = 0; i < (int)strlen(gprmc_string); i++) {
			if (gprmc_string[i]==',') {
				gprmc_string[i]=' ';
			}
		}

		// Scan the string
		sscanf(gprmc_string, "%*s %s %s %s %s %s %s %lf %lf %02d %02d %02d",
			dateStr, valid, latStr, sn, lonStr, ew,
			&speed, &heading, &day, &mon, &year);
		sscanf(dateStr, "%02d %02d %02d", &hh, &mm, &ss);
		sscanf(latStr, "%lf", &Lat_v);
		sscanf(lonStr, "%lf", &Lon_v);
		speed *= 0.514444;
		year += 2000;
		Lat_deg = (int) Lat_v /100;
		Lon_deg = (int) Lon_v /100;
		Lat_mi = Lat_v - Lat_deg * 100;
		Lon_mi = Lon_v - Lon_deg * 100;
		Latd = Lat_deg + Lat_mi / 60;
		Lond = Lon_deg + Lon_mi / 60;

		if(sn == "S") Latd = -Latd;
		if(ew == "W") Lond = -Lond;
	
		if(valid[0] != 'A')
		{
			//printf("!!!invalid gprmc message \n") ;
			return false ;	
		}
		utime = utc_date2sec(year, mon, day, hh, 0, 0) ;
		// Print out to the output CSV file
		//printf("%04d-%02d-%02d,%02d:%02d:%02d,%lf,%lf,%s,%lf,%lf\n",
		//	year, mon, day, hh, mm, ss,
		//	Latd, Lond,
		//	valid,
		//	speed,
		//	heading);		
		
		return true ;
	}
		
	bool decode_packet(const std::vector<uint8_t> &data,
	                   std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
					   int& block_index,
					   int64_t time_offset) override
	{
	    out_velodyne_pointclouds.clear();
	    const raw_packet_t *raw = reinterpret_cast<const raw_packet_t *>(&data[0]);
	    float azimuth_diff;
	    float last_azimuth_diff=0.0;
	
	    //Use device timestamp when it is not provided
	    uint32_t timestamp = updateTypeAndTimestamp(raw);
	
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
	            return false;
			}
	    }
	
	    for (int block = 0; block < BLOCKS_PER_PACKET; ++block)
	    {
			int firing = block ;
	        float azimuth = (float)(addOffset(raw->blocks[block].rotation, 0));
			static float last_azimuth = azimuth - 1e-3 ;
	        if (block < (BLOCKS_PER_PACKET-1))
	        {
	            azimuth_diff = (float)(addOffset(raw->blocks[block+1].rotation, raw->blocks[block].rotation));
	            last_azimuth_diff = azimuth_diff;
	        }
	        else
	        {
	            azimuth_diff = last_azimuth_diff;
	        }
	        float cur_azimuth_diff = azimuth - last_azimuth ;
            //printf("azimuth diff %f \n", cur_azimuth_diff) ;	
	        //Velodyne produces repeated entry in return_mode = dual
	        if(cur_azimuth_diff < 0)
	        {
	            double velo_timestamp = timestamp*1e-6;
	            if(last_timestamp_>0)
	                velo_freq_ = std::round(1.0/(velo_timestamp - last_timestamp_));
	
	            last_timestamp_ = velo_timestamp;
	            block_index = firing;
	        }
	        last_azimuth = azimuth;
	        pcl::PointCloud<PCLVelodyne> decoded_pcl;
			
			decoded_pcl.header.stamp = timestamp + v_cal_->packetStartOffset() + firing * v_cal_->blockTDuration() + time_offset*1e6 ;
			decode_singleBlock(raw->blocks[block], azimuth_diff, decoded_pcl);
	
			if(decoded_pcl.size() > 0)
			{
				out_velodyne_pointclouds.push_back(decoded_pcl);
				//for(PCLVelodyne & point : decoded_pcl.points)
				//{
				//	printf("ring %d distance %g intensity %g azimuth %d x %g y %g z %g \n", point.ring, point.distance, point.intensity, point.mdegree, point.x, point.y, point.z) ;
				//}
			}
	    }
	    return true ;
	}
	
	void decode_singleBlock(const raw_block & upper_block, 
							float total_azimuth, 
						    pcl::PointCloud<PCLVelodyne>& pcl) override
	{
	    float azimuth = (float)upper_block.rotation;
		
	    pcl.points.resize(v_cal_->scansPerFiring());
	    
		const raw_block * block_ptr = &upper_block ; 
		const raw_block & block = *block_ptr ;
		for (int dsr=0, k=0; dsr < v_cal_->scansPerFiring(); dsr++, k+=RAW_SCAN_SIZE)
		{
		    VelodyneCal corrections = (*v_cal_)[dsr];
		    /** Position Calculation */
		    union two_bytes tmp;
		    tmp.bytes[0] = block.data[k];
		    tmp.bytes[1] = block.data[k+1];
		
		    /** correct for the laser rotation as a function of timing during the firings **/
		    //azimuth reading is available at every new block
		    float azimuth_corrected_f = azimuth + (total_azimuth*v_cal_->getFiringTimeRatio(dsr, 0));
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
		    //float x_coord = y;
		    //float y_coord = -x;
		    float x_coord = x;
		    float y_coord = y;
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
            int64_t utime = pcl.header.stamp + dsr * v_cal_->dsrTOffset() ;
			memcpy(&point.utime1, &utime, sizeof(int64_t)) ;
		    // append this point to the cloud
			//printf("ring %d distance %g intensity %g x %g y %g z %g \n", point.ring, point.distance, point.intensity, point.x, point.y, point.z) ;
		    pcl.points[point.ring] = point;
		}
		++block_ptr ;
}

} ;

#endif
