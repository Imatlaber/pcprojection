/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne udp receiver
 */

#ifndef VELODYNE_UDP_RECEIVER_H
#define VELODYNE_UDP_RECEIVER_H

#include <lcmtypes/had/byteDataArray_t.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

class VelodyneUDPReceiver
{
private:
    std::unique_ptr<boost::asio::ip::udp::socket> data_socket, position_socket;

    boost::array<uint8_t, 2048> pos_recv_buffer, data_recv_buffer;

    had::byteData_t packIntoByteDataT(size_t length, boost::array<uint8_t, 2048> &recv_buffer);

    virtual void publishVeloRaw(had::byteDataArray_t velo_data) = 0;

    virtual void publishVeloPos(had::byteData_t velo_pos) = 0;

    void dataHandler(const boost::system::error_code &ec, size_t length);

    void positionhandler(const boost::system::error_code &ec, size_t length);

    int zero_rotation_offset;

public:
    //zero rotation is the offset given in 100*deg to determine
    VelodyneUDPReceiver(int data_port, int position_port,
                        boost::asio::io_service &data_service,
                        boost::asio::io_service &pos_service, int zero_rotation=0);

    VelodyneUDPReceiver(int data_port, boost::asio::io_service &data_service, int zero_rotation=0);

    void startDataReceive();

    void startPosReceive();

    ~VelodyneUDPReceiver();

};
#endif // VELODYNE_UDP_RECEIVER_H
