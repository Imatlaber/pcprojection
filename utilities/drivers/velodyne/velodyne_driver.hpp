/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne driver
 */

#ifndef _VELODYNE_DRIVER_HPP_
#define _VELODYNE_DRIVER_HPP_

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include "lcmtypes/had/byteDataArray_t.hpp"
#include "../../hadif/trans.hpp"
#include "../../hadif/param.hpp"
#include "pcl_point_types.h"

const int RAW_SCAN_SIZE = 3;
const int SCANS_PER_BLOCK = 32;
const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

const int PACKET_SIZE = 1206;
const int BLOCKS_PER_PACKET = 12;
const int PACKET_TIMESTAMP_SIZE = 4;
const int FACTORY_SIZE = 2;

const float DISTANCE_RESOLUTION = 0.002f; /**< meters */


enum class VelodyneType
{
    None,
    VLP16,
    HDL32,
	HDL64
};

enum class VelodyneInputType
{
    None,
    UDP,
    PCAP
};

enum class VelodyneReturnMode
{
    Strongest,
    Last,
    Dual,
    Undetermined
};

struct VelodyneCal
{
	double angle;
    double vert_cos_table, vert_sin_table;
    uint16_t ring;
};

/** Class for storing different velodyne calibrations
 *  This is the base class that is used to derive classes
 *  that holds different calibration values for HDL-32 and VLP-16
 */
class VelodyneCalibration
{
    int firings_per_block;
    int scans_per_firing;
    double block_tduration;
    double dsr_t_offset;
    double firing_t_offset;
    double total_packet_time;
    double packet_start_offset;

public:
    std::vector<VelodyneCal> cal_;
    uint16_t total_ring;
    VelodyneType type_;

    template<size_t N>
    void initializeRing(std::array<double, N> arr, std::array<uint16_t, N> ring);

public:
    VelodyneCalibration(int fpb, 
                        int spf, 
                        double bt, 
                        double dt, 
                        double ft,
                        double pso);

    VelodyneCal& operator[](unsigned int i);

    virtual double getFiringTimeRatio(int dsr, int n_firing) const;
    VelodyneType getType() const;

    // TODO why const-by-ref return here?
    const int& firingsPerBlock() const;
    const int& scansPerFiring() const;
    const double& blockTDuration() const;
    const double& dsrTOffset() const;
    const double& firingTOffset() const;
    double packetStartOffset() const ;
    const uint16_t& totalRing() const;
};

// TODO (long-term) constructor of VLP16Calibration has lot of code in common with HDL32Calibration constructor
/** The derived class that holds calibration values for
 *  VLP-16. It initializes struct VelodyneCal for different
 *  ring orientation
 */
class VLP16Calibration: public VelodyneCalibration
{

public:
    VLP16Calibration();
};

class VLP16HDCalibration: public VelodyneCalibration
{
public:
    VLP16HDCalibration();
};


/** The derived class that holds calibration values for
 *  HDL-32. It initializes struct VelodyneCal for different
 *  ring orientation
 */
class HDL32Calibration: public VelodyneCalibration
{
public:

    HDL32Calibration();
};

/** The derived class that holds calibration values for
 *  HDL-64E S3. It initializes struct VelodyneCal for different
 *  ring orientation
 */
class HDL64Calibration: public VelodyneCalibration
{
public:

    HDL64Calibration();

    double getFiringTimeRatio(int dsr, int n_firing) const override;

};

union two_bytes
{
    uint16_t uint;
    uint8_t  bytes[2];
};

typedef struct raw_block
{
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
} raw_block_t;

typedef struct raw_packet
{
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint8_t timestamp[PACKET_TIMESTAMP_SIZE];
    uint8_t factory[FACTORY_SIZE];
} raw_packet_t;

/** Main class of Velodyne driver
 *  It contains 3 constructors and VelodyneCalibration is required in all cases.
 *  The first constructor allows the usage of decodePacket(), the second and
 *  third constructors adding getPacket() function. decodePacket() is useful
 *  as a generic binary packet to PointCloud conversion. For example, cvw_player
 *  uses this fuction to obtain Velodyne's PointCloud. Second constructor expects
 *  packet transmitting from a UDP port. This is where a live streaming packets
 *  from Velodyne are used. The last constructor expects a filename points to
 *  packets dump stored in a file. Normally the file has extension ".pcap" and
 *  this is the same format Velodyne distributes the sample data.
 */

enum class VelodyneRotStat
{
    WRONG_HEADER=0,
    NEW_ROTATION,
    NORMAL_ROTATION,
    REPEATED_ROTATION //Happens in new firmware where when out of order encoder reading is occur
};

class VelodyneInput
{

public:
    VelodyneInputType input_type_;
    pcap_t *pcap_;
    boost::asio::ip::udp::socket *socket_;
    boost::asio::deadline_timer *deadline_;
    boost::asio::io_service *io_service_;

    uint32_t timeout_count_; //reset to zero each received packet and increment when timeout occur

    //return pointcloud when a complete revolution is reached
    double last_azimuth_;
    double last_timestamp_;
    int velo_freq_;
    bool close_socket_;
    VelodyneCalibration * v_cal_;
    VelodyneType detected_type_;
    VelodyneType initialized_type_;
    VelodyneReturnMode return_mode_;
	std::string pcap_filename_ ;

    void initialize();

    void checkDeadline();

    static void handleRecv(
        const boost::system::error_code& ec, std::size_t length,
        boost::system::error_code* out_ec, std::size_t* out_length);

    virtual void decodeSingleBlock(const raw_block& block0, 
						   const float total_azimuth, 
						   std::vector< pcl::PointCloud<PCLVelodyne> > &pcl);

	virtual void decode_singleBlock(const raw_block & upper_block, 
							float total_azimuth, 
						    pcl::PointCloud<PCLVelodyne>& pcl) {}
   
    uint32_t updateTypeAndTimestamp(const raw_packet_t *raw);

    static VelodyneReturnMode determineVelodyneReturnMode(const raw_packet_t *raw);

    void outputPCLVelo(trans_t * trans, std::string frame_id, pcl::PointCloud<PCLVelodyne> &velodyne_onerev_pc, pcl::PointCloud<PCLVelodyne> &onerev_pc_return) const;

    static int addOffset(int original_rotation, int offset);
public:
    double cos_table[36000];
    double sin_table[36000];

    // TODO (refactor) these 3 constructors should be converted into 3 different classes that implement/extend "VelodyneInput"
    // for instance VelodyneInputUdp, VelodyneInputPcap, ...
    // this would get rid of switch (input_type_) {...} in the destructor, getPacket(), ...
    VelodyneInput(VelodyneCalibration * v_cal);

    VelodyneInput(uint16_t udp_port, VelodyneCalibration * v_cal);

    VelodyneInput(std::string filename, VelodyneCalibration * v_cal);

    bool isFile() const;

    void closeSocket();

    int getVelodyneRing() const;

    uint32_t getTimeoutCount() const;

    VelodyneType getInitializedType() const;

    bool determineVelodyneType(VelodyneType &velodyne, int &frequency) const;

    ~VelodyneInput();
	
	bool restart() ;

    bool getPacket(std::vector<uint8_t> &data);

    virtual int decodePacket(const std::vector<uint8_t> &data,
                      std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
                     int offset_angle_two_decimal = 0.0, int64_t timestamp=0);

	virtual bool decode_packet(const std::vector<uint8_t> &data,
	                   std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
					   int& block_index,
					   int64_t time_offset) 
	{
		return true ;
	}
    
	virtual bool decode_packet_pep(const std::vector<uint8_t> &data, int64_t & utime)
	{
		return true ;	
	}

	static VelodyneRotStat isNewSingleRot(const std::vector<uint8_t> &data, int offset_angle_two_decimal);

    void getOneRevPointCloud(const had::byteDataArray_t *velo_packets,
                             trans_t * trans,
                             pcl::PointCloud<PCLVelodyne> &onerev_pc_return,
                             int offset_angle_two_decimal = 0,
                             bool use_device_timestamp = false);

};


#endif
