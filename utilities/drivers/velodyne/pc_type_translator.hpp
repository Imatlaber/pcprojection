/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne point type conversion
 */

#ifndef _LCMTYPES_TRANSLATOR_HPP_
#define _LCMTYPES_TRANSLATOR_HPP_

#include <bot_core/timestamp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <vector>
#include "lcmtypes/had/pointcloud_t.hpp"
#include "pcl_point_types.h"

pcl::PCLPointField getPCLPointfield(std::string name, int32_t offset, int8_t datatype, int32_t count);

/** \brief Convert a had::pointfield_t to pcl::PCLPointField
 *
 */
pcl::PCLPointField toPCLPointField(const had::pointfield_t& pf_t);

/** \brief Convert a pcl::PCLPointField to had::pointfield_t
 *
 */
had::pointfield_t fromPCLPointField(const pcl::PCLPointField& pf);

/** \brief Convert a pcl::PCLPointCloud2 to had::pointcloud_t
 *
 */
//NOTE: Not all Header information is used!!
void fromPCLPointCloud2(const pcl::PCLPointCloud2& pc, had::pointcloud_t& pc_t);

/** \brief Convert a pcl::PCLPointCloud2 to had::pointcloud_t
 *
 */
//NOTE: Not all Header information is used!!
void toPCLPointCloud2(const had::pointcloud_t& pc_t, pcl::PCLPointCloud2& pc);

template<class S>
std::vector<uint8_t> toByteVector(S a)
{
    char* b = reinterpret_cast<char*>(&a);
    std::vector<uint8_t> vect;
    for (unsigned int i = 0; i < sizeof(S); i++)
        vect.push_back(b[i] & 0xFF);
    return vect;
}

template<class D>
D bytesTo(std::string a)
{
    char* b = new char(sizeof(D));
    for (unsigned int i = 0; i < sizeof(D); i++)
        b[i] = (a[i] & 0xFF);
    D d = *(reinterpret_cast<D*>(b));
    delete b;
    return d;
}

/** \brief Convert a pcl::PointCloud<had::PCLVelodyne> to had::pointcloud_t
 *
 */
template <class T>
void fromPCLVelodyne(const pcl::PointCloud<T>& pcl, had::pointcloud_t& pc_t)
{
    pcl::PCLPointCloud2 pcl2;
	//for(auto & p : pcl.points)
	//	printf("x y z intensity distance ring medgree : %g %g %g %g %g %d %d \n", p.x, p.y, p.z, p.intensity, p.distance, p.ring, p.mdegree) ;
    pcl::toPCLPointCloud2(pcl, pcl2);
	//for(auto &d : pcl2.data)
	//	printf("%u \n", d) ;
    fromPCLPointCloud2(pcl2, pc_t);
}

/** \brief Convert a had::pointcloud_t to pcl::PointCloud<had::PCLVelodyne>
 *
 */
template <class T>
void toPCLVelodyne(const had::pointcloud_t& pc_t, pcl::PointCloud<T>& pcl)
{
    pcl::PCLPointCloud2 pc2;
    toPCLPointCloud2(pc_t, pc2);
    pcl::fromPCLPointCloud2(pc2, pcl);
    //Fix the changed type of velodyne::PCLVelodyne ring type
    bool found_ring_field = false;
    for(auto pc_field : pc_t.fields)
    {
        if(pc_field.name == "ring")
            found_ring_field = true;
    }
    if(found_ring_field)
    {
        for(auto &pt : pcl.points)
        {
            constexpr int negative_value = pow(2, 22);
            if(pt.ring >= negative_value)
                pt.ring -= negative_value;
        }
    }
}



#endif
