/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne driver
 */

#include <assert.h>
#include <cmath>
#include "velodyne_driver.hpp"
#include "../../../utilities/hadif/math_helper.hpp"
#include "pcl_point_types.h"

VelodyneCalibration::VelodyneCalibration(int fpb, 
                                         int spf, 
                                         double bt, 
                                         double dt, 
                                         double ft,
                                         double pso) :
    firings_per_block{fpb},
    scans_per_firing{spf},
    block_tduration{bt},
    dsr_t_offset{dt},
    firing_t_offset{ft},
    packet_start_offset{pso},
    total_ring{},
    type_{VelodyneType::None}
{
    total_packet_time = 11*block_tduration + (spf/firings_per_block-1)*dsr_t_offset + fpb*ft;
}

VelodyneCal& VelodyneCalibration::operator[](unsigned int i)
{
    return cal_[i];
}

double VelodyneCalibration::getFiringTimeRatio(int dsr, int n_firing) const
{
    double fire_time = (dsr*dsr_t_offset + n_firing*firing_t_offset);
    double ratio = fire_time/block_tduration;
    return ratio;
}

VelodyneType VelodyneCalibration::getType() const
{
    return type_;
}

const int& VelodyneCalibration::firingsPerBlock() const
{
    return firings_per_block;
}
const int& VelodyneCalibration::scansPerFiring() const
{
    return scans_per_firing;
}
const double& VelodyneCalibration::blockTDuration() const
{
    return block_tduration;
}
const double& VelodyneCalibration::dsrTOffset() const
{
    return dsr_t_offset;
}
const double& VelodyneCalibration::firingTOffset() const
{
    return firing_t_offset;
}
double VelodyneCalibration::packetStartOffset() const 
{
    return packet_start_offset ;
}
const uint16_t& VelodyneCalibration::totalRing() const
{
    return total_ring;
}

VLP16Calibration::VLP16Calibration(): VelodyneCalibration(2, 16, 110.592, 2.304, 55.296, 0)
{
    type_ = VelodyneType::VLP16;
    static const std::array<double, 16> arr = {-15.0, 1.0, -13.0, 3.0,
                                               -11.0, 5.0, -9.0, 7.0,
                                               -7.0, 9.0, -5.0, 11.0,
                                               -3.0, 13.0, -1.0, 15.0};
    static const std::array<uint16_t, 16> ring_arr = {0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15};
    this->initializeRing(arr, ring_arr);
    std::cout<<"VLP16 calibration initialized with "<<cal_.size()<<" ring"<<std::endl;
}

VLP16HDCalibration::VLP16HDCalibration() : VelodyneCalibration(2, 16, 110.592, 2.304, 55.296, 0)
{
    type_ = VelodyneType::VLP16;
    static const std::array<double, 16> arr = {-10.000,0.667,-8.667,2.000,
                                               -7.333,3.333,-6.000,4.667,
                                               -4.667,6.000,-3.333,7.333,
                                               -2.000,8.667,-0.667,10.000};
    static const std::array<uint16_t, 16> ring_arr = {0,8,1,9,2,10,3,11,4,12,5,13,6,14,7,15};
    this->initializeRing(arr, ring_arr);
    std::cout<<"VLP16HD calibration initialized with "<<cal_.size()<<" ring"<<std::endl;
}

HDL32Calibration::HDL32Calibration(): VelodyneCalibration(1, 32, 46.08, 1.152, 0.0, -542.592)
{
    type_ = VelodyneType::HDL32;
    static const std::array<double, 32> arr = {-30.67, -9.33, -29.33, -8.00,
                                               -28.00, -6.66, -26.66, -5.33,
                                               -25.33, -4.00, -24.00, -2.67,
                                               -22.67, -1.33, -21.33, 0.00,
                                               -20.00, 1.33, -18.67, 2.67,
                                               -17.33, 4.00, -16.00, 5.33,
                                               -14.67, 6.67, -13.33, 8.00,
                                               -12.00, 9.33, -10.67, 10.67};

    static const std::array<uint16_t, 32> ring_arr = {0, 16, 1, 17, 2, 18, 3, 19,
                                                      4, 20, 5, 21, 6, 22, 7, 23,
                                                      8, 24, 9, 25, 10, 26, 11, 27,
                                                      12, 28, 13, 29, 14, 30, 15, 31};
    this->initializeRing(arr, ring_arr);
    std::cout<<"HDL32 calibration initialized with "<<cal_.size()<<" ring"<<std::endl;
}

HDL64Calibration::HDL64Calibration(): VelodyneCalibration(1, 32, 46.08, 1.152, 0.0, 0)
{
    type_ = VelodyneType::HDL64;

	static std::array<double, 64> arr = {
-7.15833, -6.81802, 0.317831, 0.658138, 
-6.47784, -6.13777, -8.52106, -8.18013, 
-5.79781, -5.45793, -7.83937, -7.49878, 
-3.0803, -2.74071, -5.11813, -4.7784, 
-2.40111, -2.06147, -4.43872, -4.09908, 
-1.72179, -1.38206, -3.75948, -3.41989, 
0.998584, 1.33918, -1.04226, -0.702384, 
1.67994, 2.02087, -0.362418, -0.0223504, 
-22.7386, -22.2267, -11.5143, -11.0024, 
-21.7153, -21.2043, -24.7118, -24.277, 
-20.6936, -20.1833, -23.7637, -23.2509, 
-16.6158, -16.1064, -19.6732, -19.1633, 
-15.597, -15.0874, -18.5584, -18.144, 
-14.5777, -14.0678, -17.6346, -17.1252, 
-10.4901, -9.97733, -13.5577, -13.0474, 
-9.46396, -8.94999, -12.5367, -12.0257 };

	static std::array<uint16_t, 64> ring_arr = {
36, 37, 58, 59, 
38, 39, 32, 33, 
40, 41, 34, 35, 
48, 49, 42, 43, 
50, 51, 44, 45, 
52, 53, 46, 47, 
60, 61, 54, 55, 
62, 63, 56, 57, 
4, 5, 26, 27, 
6, 7, 0, 1, 
8, 9, 2, 3, 
16, 17, 10, 11, 
18, 19, 12, 13, 
20, 21, 14, 15, 
28, 29, 22, 23, 
30, 31, 24, 25} ;

	for(int i=0; i<16; ++i)
		printf("%g ", arr[i]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%g ", arr[i+16]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%g ", arr[i+32]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%g ", arr[i+48]) ;
	printf("\n----------------\n") ;
	
	for(int i=0; i<16; ++i)
		printf("%d ", ring_arr[i]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%d ", ring_arr[i+16]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%d ", ring_arr[i+32]) ;
	printf("\n") ;
	for(int i=0; i<16; ++i)
		printf("%d ", ring_arr[i+48]) ;
	printf("\n") ;
	
	this->initializeRing(arr, ring_arr);
    std::cout<<"HDL64 calibration initialized with "<<cal_.size()<<" ring"<<std::endl;
}

double HDL64Calibration::getFiringTimeRatio(int dsr, int /* n_firing */) const 
{
	int index = dsr/2 ;
	return index/32.0 ;
}

template <size_t N>
void VelodyneCalibration::initializeRing(std::array<double, N> arr, std::array<uint16_t, N> ring)
{
    for(size_t i=0; i<ring.size(); i++)
    {
        double ring_rad = arr[i]/180.0*M_PI;
        VelodyneCal vlp_cal;
		vlp_cal.angle = ring_rad ;
        vlp_cal.vert_cos_table = cos(ring_rad);
        vlp_cal.vert_sin_table = sin(ring_rad);
        vlp_cal.ring = ring[i];
        cal_.push_back(vlp_cal);
    }
    total_ring = cal_.size();
}

void VelodyneInput::initialize()
{
    detected_type_ = VelodyneType::None;
    last_azimuth_ = 0.0;
    last_timestamp_ = 0.0;
    velo_freq_ = 10.0; //10Hz by default
    pcap_ = NULL;
    //preload the cos and sin table
    // TODO encode reoccurring number 36000 in a single location
    for(int i=0; i<36000; i++)
    {
        cos_table[i] = cos(i/18000.0*M_PI);
        sin_table[i] = sin(i/18000.0*M_PI);
    }
}

void VelodyneInput::checkDeadline()
{
    if (deadline_->expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
        if(close_socket_)
        {
            socket_->cancel();
            deadline_->expires_at(boost::posix_time::pos_infin);
        }
        else
        {
            deadline_->expires_from_now(boost::posix_time::milliseconds(100));
            timeout_count_++;
        }
    }
    deadline_->async_wait(boost::bind(&VelodyneInput::checkDeadline, this));
}

void VelodyneInput::handleRecv(const boost::system::error_code &ec, std::size_t length, boost::system::error_code *out_ec, std::size_t *out_length)
{
    *out_ec = ec;
    *out_length = length;
}

VelodyneInput::VelodyneInput(VelodyneCalibration * v_cal) : v_cal_(v_cal),
    socket_{NULL}, deadline_{NULL}, io_service_{NULL}, timeout_count_{0}, close_socket_{false},
    return_mode_{VelodyneReturnMode::Strongest}
{
    initialized_type_ = v_cal_->getType();
    input_type_ = VelodyneInputType::None;
    initialize();
}

VelodyneInput::VelodyneInput(uint16_t udp_port, VelodyneCalibration * v_cal) : v_cal_(v_cal) //2368 default port
{
    initialized_type_ = v_cal_->getType();
    input_type_ = VelodyneInputType::UDP;
    initialize();
    io_service_ = new boost::asio::io_service();
    socket_ = new boost::asio::ip::udp::socket(*io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), udp_port));
    deadline_ = new boost::asio::deadline_timer(*io_service_);
    std::cout<<"Socket opened with udp port: "<<udp_port<<std::endl;
    close_socket_ = false;
    checkDeadline();
}

VelodyneInput::VelodyneInput(std::string filename, VelodyneCalibration * v_cal) : v_cal_(v_cal),
  socket_{NULL}, deadline_{NULL}, io_service_{NULL}, timeout_count_{0}, close_socket_{false},
  return_mode_{VelodyneReturnMode::Strongest}
{
	pcap_filename_ = filename ;
    initialized_type_ = v_cal_->getType();
    input_type_ = VelodyneInputType::PCAP;
    char errbuf[PCAP_ERRBUF_SIZE];
    initialize();
    if((pcap_ = pcap_open_offline(pcap_filename_.c_str(), errbuf))==NULL)
    {
        std::cout<<"Error openning PCAP file"<<std::endl;
        exit(3);
    }
    else
        std::cout<<"PCAP file opened"<<std::endl;
}

bool VelodyneInput::restart()
{
	switch(input_type_)
	{
		case VelodyneInputType::PCAP:
			pcap_close(pcap_) ;
			char errbuf[PCAP_ERRBUF_SIZE];
			if((pcap_ = pcap_open_offline(pcap_filename_.c_str(), errbuf))==NULL)
    		{
    		    std::cout<<"Error openning PCAP file"<<std::endl;
    		    exit(3);
    		}
    		else
    		    std::cout<<"PCAP file reopened"<<std::endl;

			break ;

		default:
			return false ;
	}

	return true ;
}

bool VelodyneInput::isFile() const
{
    if(input_type_ == VelodyneInputType::PCAP)
        return true;
    else
        return false;
}

void VelodyneInput::closeSocket()
{
    close_socket_ = true;
}

int VelodyneInput::getVelodyneRing() const
{
    return v_cal_->totalRing();
}

uint32_t VelodyneInput::getTimeoutCount() const
{
    return timeout_count_;
}

VelodyneType VelodyneInput::getInitializedType() const
{
    return initialized_type_;
}

bool VelodyneInput::determineVelodyneType(VelodyneType &velodyne, int &frequency) const
{
    frequency = velo_freq_;
    if(detected_type_!=VelodyneType::None)
    {
        velodyne = detected_type_;
        return true;
    }
    return false;
}

VelodyneInput::~VelodyneInput()
{
    switch (input_type_)
    {
    case VelodyneInputType::UDP:
        close_socket_ = true;
        socket_->close();
        break;

    case VelodyneInputType::PCAP:
        pcap_close(pcap_);
        break;

    case VelodyneInputType::None:
        std::cout << "WARNING: Closing with VelodyneInputType::None" << std::endl;
        break;

    default:
        assert(false);
        break;
    }
}

bool VelodyneInput::getPacket(std::vector<uint8_t> &data)
{
    size_t buffer_size = 0;
    //        boost::asio::ip::udp::endpoint remote_endpoint;
    boost::system::error_code error;
    struct pcap_pkthdr *pcap_header;
    const u_char *pcap_data;
    
    switch (input_type_)
    {
    case VelodyneInputType::UDP:
        deadline_->expires_from_now(boost::posix_time::milliseconds(500));
        error = boost::asio::error::would_block;
        std::array<char, PACKET_SIZE> recv_buf;
        //            buffer_size = socket_->receive_from(boost::asio::buffer(recv_buf),remote_endpoint, 0, error);
        socket_->async_receive(boost::asio::buffer(recv_buf),
                               boost::bind(&VelodyneInput::handleRecv, _1, _2, &error, &buffer_size));
        do io_service_->run_one();
        while (error == boost::asio::error::would_block);
        if(buffer_size==PACKET_SIZE)
        {
            timeout_count_ = 0;
            data.clear();
            data.resize(PACKET_SIZE);
            memcpy(&data[0], &recv_buf[0], PACKET_SIZE);
            return true;
        }
        break;

    case VelodyneInputType::PCAP:
        data.clear();

        if((pcap_next_ex(pcap_, &pcap_header, &pcap_data)) >=0)
        {
			//printf("len %d caplen %d \n", pcap_header->len, pcap_header->caplen) ;		
			const uint32_t offset = 42 ;
			uint32_t packet_size = pcap_header->caplen - offset ;
			data.resize(packet_size);
            memcpy(&data[0], pcap_data+offset, packet_size);
            return true;
        }
        else
        {
            std::cout<<"End of file"<<std::endl;
            return false;
        }
        break;

    case VelodyneInputType::None:
        std::cout << "Invalid VelodyneInputType" << std::endl;
        return false;
        break;

    default:
        assert(false);
        break;
    }
    return false;
}

void VelodyneInput::decodeSingleBlock(const raw_block & block, 
									  float total_azimuth, 
									  std::vector< pcl::PointCloud<PCLVelodyne> > &pcl)
{
    float azimuth = (float)block.rotation;

    pcl.resize(v_cal_->firingsPerBlock());
    for(auto &pc : pcl)
        pc.points.resize(v_cal_->scansPerFiring());
    
	for (int firing=0, k=0; firing < v_cal_->firingsPerBlock(); firing++)
    {
        for (int dsr=0; dsr < v_cal_->scansPerFiring(); dsr++, k+=RAW_SCAN_SIZE)
        {
            VelodyneCal  corrections = (*v_cal_)[dsr];
            /** Position Calculation */
            union two_bytes tmp;
            tmp.bytes[0] = block.data[k];
            tmp.bytes[1] = block.data[k+1];

            /** correct for the laser rotation as a function of timing during the firings **/
            //azimuth reading is available at every new block
            float azimuth_corrected_f = azimuth + (total_azimuth*v_cal_->getFiringTimeRatio(dsr, firing));
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
            pcl[firing].points[point.ring] = point;
        }
    }
}

VelodyneReturnMode VelodyneInput::determineVelodyneReturnMode(const raw_packet_t *raw)
{
    switch (raw->factory[0])
    {
    case 0x37:
        return VelodyneReturnMode::Strongest;
    case 0x38:
        return VelodyneReturnMode::Last;
    case 0x39:
        return VelodyneReturnMode::Dual;
    }
    return VelodyneReturnMode::Undetermined;
}

uint32_t VelodyneInput::updateTypeAndTimestamp(const raw_packet_t *raw)
{
    uint32_t *timestamp = (uint32_t*)(raw->timestamp);

    std::cout.precision(5);
    static uint32_t timestamp_pre = *timestamp;
    static int counter = 0;
    ++counter;
    if(raw->blocks[0].header == 0xEEFF)
    {
        int64_t time_diff = *timestamp-timestamp_pre;
        //warn if time diff of more than 15ms
        if(abs(time_diff)>15e3)
            std::cout<<std::dec<<counter<<": Previous timestamp was "<<timestamp_pre*1e-6<<"s. Timestamp now is "<<*timestamp*1e-6<<"s. Time diff="<<time_diff*1e-6<<"s"<<std::endl;

        switch (raw->factory[1])
        {
        case 0x21:
            detected_type_ = VelodyneType::HDL32;
            break;
        case 0x22:
            detected_type_ = VelodyneType::VLP16;
            break;
        default:
            detected_type_ = VelodyneType::None;
            break;
        }
        switch (raw->factory[0])
        {
        case 0x37:
            return_mode_ = VelodyneReturnMode::Strongest;
            break;
        case 0x38:
            return_mode_ = VelodyneReturnMode::Last;
            break;
        case 0x39:
            return_mode_ = VelodyneReturnMode::Dual;
            break;
        default:
            return_mode_ = VelodyneReturnMode::Undetermined;
            break;
        }
    }
    timestamp_pre = *timestamp;
    return *timestamp;
}

VelodyneRotStat VelodyneInput::isNewSingleRot(const std::vector<uint8_t> &data, int offset_angle_two_decimal)
{
    const raw_packet_t *raw = reinterpret_cast<const raw_packet_t *>(&data[0]);

    static int last_encoder_value = 0;


    for (int block = 0; block < BLOCKS_PER_PACKET; block++)
    {
        if(0xEEFF != raw->blocks[block].header)
            return VelodyneRotStat::WRONG_HEADER;
    }

    VelodyneRotStat rotation_status;
    int block_offset = 1;

    VelodyneReturnMode return_mode = determineVelodyneReturnMode(raw);

    rotation_status = VelodyneRotStat::NORMAL_ROTATION;
    if(return_mode == VelodyneReturnMode::Dual) block_offset = 2;

    for(int block = 0; block < BLOCKS_PER_PACKET; block+=block_offset)
    {
        int encoder_value = addOffset(raw->blocks[block].rotation, offset_angle_two_decimal);
        int encoder_diff = encoder_value - last_encoder_value;
        if(encoder_diff < 0)
        {
            if(encoder_diff>-35000)
                rotation_status = VelodyneRotStat::REPEATED_ROTATION;
            else
                rotation_status = VelodyneRotStat::NEW_ROTATION;
        }
        last_encoder_value = encoder_value;
    }
    return rotation_status;
}

int VelodyneInput::addOffset(int original_rotation, int offset)
{
    return mod(original_rotation - offset, 36000);
}

int VelodyneInput::decodePacket(const std::vector<uint8_t> &data,
                                std::vector<pcl::PointCloud<PCLVelodyne>> &out_velodyne_pointclouds,
                                int offset_angle_two_decimal, int64_t timestamp)
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
        if(0xEEFF != raw->blocks[block].header)
        {
            if(input_type_ == VelodyneInputType::UDP)
                std::cout<<"Unexpected header"<<std::endl;
            return -1;
        }
    }

    int block_offset = 1;
    if(return_mode_ == VelodyneReturnMode::Dual) block_offset = 2;

    for (int block = 0; block < BLOCKS_PER_PACKET; block+=block_offset)
    {
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
            single_rev_idx = block*v_cal_->firingsPerBlock();
        }
        last_azimuth_ = azimuth;
        std::vector<pcl::PointCloud<PCLVelodyne>> decoded_pcl, dual_pcl;

        decodeSingleBlock(raw->blocks[block], azimuth_diff, decoded_pcl);
        if(return_mode_ == VelodyneReturnMode::Dual)
            decodeSingleBlock(raw->blocks[block+1], azimuth_diff, dual_pcl);

        //each single block might contain 2 firings, as reported from vector size of a decoded_pcl
        for(int i=0; i<v_cal_->firingsPerBlock(); i++)
        {
            decoded_pcl[i].header.stamp = timestamp +
                    block * v_cal_->blockTDuration() +
                    i*v_cal_->firingTOffset();
             out_velodyne_pointclouds.push_back(decoded_pcl[i]);

             if(return_mode_ == VelodyneReturnMode::Dual)
             {
                 dual_pcl[i].header.stamp = decoded_pcl[i].header.stamp;
                 out_velodyne_pointclouds.push_back(dual_pcl[i]);
             }
        }

    }
    return single_rev_idx;
}

void VelodyneInput::outputPCLVelo(trans_t *trans, std::string frame_id, pcl::PointCloud<PCLVelodyne> &velodyne_onerev_pc, pcl::PointCloud<PCLVelodyne> &onerev_pc_return) const
{
    double bot_tf[12];
    if(trans->getTf("local", frame_id, velodyne_onerev_pc.header.stamp, bot_tf))
    {
        for(PCLVelodyne &pc_pt : velodyne_onerev_pc.points)
        {
            double v[3] = {pc_pt.x, pc_pt.y, pc_pt.z};
            trans->transformPoint(v, bot_tf);
            pc_pt.x = v[0];
            pc_pt.y = v[1];
            pc_pt.z = v[2];
        }
    }

    velodyne_onerev_pc.height = getVelodyneRing();
    velodyne_onerev_pc.width = velodyne_onerev_pc.size()/getVelodyneRing();
    velodyne_onerev_pc.is_dense = true;

    onerev_pc_return = velodyne_onerev_pc;

    velodyne_onerev_pc.clear();
}

void VelodyneInput::getOneRevPointCloud(const had::byteDataArray_t *velo_packets,
                                        trans_t *trans,
                                        pcl::PointCloud<PCLVelodyne> &onerev_pc_return,
                                        int offset_angle_two_decimal, bool use_device_timestamp)
{
    static pcl::PointCloud<PCLVelodyne> velodyne_onerev_pc;

    onerev_pc_return.clear();

    bool got_first_timestamp = false;
    for(const had::byteData_t &velo_packet : velo_packets->data)
    {
        std::vector<pcl::PointCloud<PCLVelodyne>> velodyne_pointclouds;

        int64_t velo_utime = velo_packet.utime;

        if(use_device_timestamp)
            velo_utime = 0;

        int single_rev_idx = decodePacket(velo_packet.data, velodyne_pointclouds, offset_angle_two_decimal, velo_utime);
        if(!got_first_timestamp)
        {
            got_first_timestamp = true;
        }

        if(velodyne_pointclouds.size()==0)
            continue;
        int64_t total_usec_since_utc = 0;
        int64_t total_usec_hour_offset = 0;
        int64_t hour_in_usec = 3600*1e6;
        if(use_device_timestamp)
        {
            //The Velodyne is sync with PPS signal from GPS.
            //The timestamp available from the Velodyne contains a constant offset
            //Hence calculation below involve correction and making it into UTC time
            //as per reported by original velo_packets utime
            total_usec_since_utc = int64_t(velo_packet.utime/hour_in_usec)*hour_in_usec;
            total_usec_hour_offset = (velo_packet.utime-velodyne_pointclouds[0].header.stamp)%hour_in_usec;
            total_usec_hour_offset = int64_t(total_usec_hour_offset*1e-6)*1e6;

            //Inform user if offset value has updated. This could happen when
            //PPS from GPS signal is not available/not initialized properly
            static int64_t last_usec_hour_offset = total_usec_hour_offset;
            if(last_usec_hour_offset!=total_usec_hour_offset)
            {
                std::cout<<"Warning: Offset changed from "<<last_usec_hour_offset<<" to "<<
                           total_usec_hour_offset<<std::endl;
                last_usec_hour_offset = total_usec_hour_offset;
            }
        }

        for(pcl::PointCloud<PCLVelodyne> &pc : velodyne_pointclouds)
        {
            if(use_device_timestamp)
                pc.header.stamp = ((pc.header.stamp+total_usec_hour_offset)%hour_in_usec)+total_usec_since_utc;

            double bot_tf[12];
            if(trans->getTf(velo_packets->coord_frame, "local", pc.header.stamp, bot_tf))
            {
                for(PCLVelodyne &pc_pt : pc.points)
                {
                    double v[3] = {pc_pt.x, pc_pt.y, pc_pt.z};
                    trans->transformPoint(v, bot_tf);
                    pc_pt.x = v[0];
                    pc_pt.y = v[1];
                    pc_pt.z = v[2];
                }

            }
        }

        if(single_rev_idx>=0)
        {
			//printf("single_rev_idx: %d \n", single_rev_idx) ;
            for(int idx=0; idx<single_rev_idx; idx++)
                velodyne_onerev_pc+=velodyne_pointclouds[idx];

            outputPCLVelo(trans, velo_packets->coord_frame, velodyne_onerev_pc, onerev_pc_return);
            velodyne_onerev_pc.header.stamp = velodyne_pointclouds[single_rev_idx].header.stamp;
            for(size_t idx = single_rev_idx; idx < velodyne_pointclouds.size(); ++idx)
                velodyne_onerev_pc += velodyne_pointclouds[idx];
        }
        else
        {
            for(auto velo_pc : velodyne_pointclouds)
                velodyne_onerev_pc+=velo_pc;
        }

    }
}


























