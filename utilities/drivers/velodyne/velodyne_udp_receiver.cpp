/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne udp receiver
 */

#include "velodyne_driver.hpp"
#include "velodyne_udp_receiver.hpp"
#include <bot_core/bot_core.h>
#include <iostream>
#include <thread>

using namespace boost::asio::ip;

had::byteData_t VelodyneUDPReceiver::packIntoByteDataT(size_t length,
                                                          boost::array<uint8_t, 2048> &recv_buffer)
{
    had::byteData_t data_msg;
    data_msg.utime = bot_timestamp_now();
    data_msg.size = length;
    data_msg.data = std::vector<uint8_t>(recv_buffer.begin(), recv_buffer.begin()+length);
    return data_msg;
}

void VelodyneUDPReceiver::dataHandler(const boost::system::error_code& /*ec*/, size_t length)
{
    static had::byteDataArray_t single_rotate_msg;
    had::byteData_t single_msg = packIntoByteDataT(length, data_recv_buffer);
    single_rotate_msg.data.push_back(single_msg);


    VelodyneRotStat rotation_status = VelodyneInput::isNewSingleRot(single_msg.data, zero_rotation_offset);
    switch(rotation_status)
    {
    case VelodyneRotStat::NEW_ROTATION:
        single_rotate_msg.size = single_rotate_msg.data.size();
        publishVeloRaw(single_rotate_msg);
        single_rotate_msg.data.clear();
        break;
    case VelodyneRotStat::NORMAL_ROTATION:
        break;
    case VelodyneRotStat::REPEATED_ROTATION:
        single_rotate_msg.data.pop_back();
        std::cout<<"Warning: Repeated rotation received."<<std::endl;
        break;
    case VelodyneRotStat::WRONG_HEADER:
        std::cout<<"Warning: Wrong header received."<<std::endl;
        break;
    }
    startDataReceive();
}

void VelodyneUDPReceiver::positionhandler(const boost::system::error_code& /*ec*/, size_t length)
{
    had::byteData_t pos_msg = packIntoByteDataT(length, pos_recv_buffer);
    publishVeloPos(pos_msg);
    startPosReceive();
}

VelodyneUDPReceiver::VelodyneUDPReceiver(int data_port, int position_port,
                                         boost::asio::io_service &data_service,
                                         boost::asio::io_service &pos_service,
                                         int zero_rotation) : zero_rotation_offset(zero_rotation)
{
    data_socket = std::unique_ptr<udp::socket>(new udp::socket(data_service, udp::endpoint(udp::v4(), data_port)));
    position_socket = std::unique_ptr<udp::socket>(new udp::socket(pos_service, udp::endpoint(udp::v4(), position_port)));

    std::cout<<"Socket opened with udp port: "<<data_port<<" "<<position_port<<" zero rotation: "<<zero_rotation_offset<<std::endl;
    startDataReceive();
    startPosReceive();
}

VelodyneUDPReceiver::VelodyneUDPReceiver(int data_port,
                                         boost::asio::io_service &data_service,
                                         int zero_rotation) : zero_rotation_offset(zero_rotation)
{
    data_socket = std::unique_ptr<udp::socket>(new udp::socket(data_service, udp::endpoint(udp::v4(), data_port)));

    std::cout<<"Socket opened with udp port: "<<data_port<<" zero rotation: "<<zero_rotation_offset<<std::endl;
    startDataReceive();
}

void VelodyneUDPReceiver::startDataReceive()
{
    data_socket->async_receive(
                boost::asio::buffer(data_recv_buffer),
                boost::bind(&VelodyneUDPReceiver::dataHandler, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}

void VelodyneUDPReceiver::startPosReceive()
{
    position_socket->async_receive(
                boost::asio::buffer(pos_recv_buffer),
                boost::bind(&VelodyneUDPReceiver::positionhandler, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}

VelodyneUDPReceiver::~VelodyneUDPReceiver()
{
    if(data_socket.get()) data_socket->close();
    if(position_socket.get()) position_socket->close();
}
