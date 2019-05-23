/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne point type conversion
 */

#include <pcl/conversions.h>
#include <zlib.h>
#include <snappy.h>
#include "pc_type_translator.hpp"

pcl::PCLPointField getPCLPointfield(std::string name, int32_t offset, int8_t datatype,
                                    int32_t count)
{
    pcl::PCLPointField field;
    field.name = name;
    field.offset = offset;
    field.count = count;
    field.datatype = datatype;
    return field;
}

/** \brief Convert a had::pointfield_t to pcl::PCLPointField
 *
 */
pcl::PCLPointField toPCLPointField(const had::pointfield_t& pf_t)
{
    pcl::PCLPointField pf;
    pf.name = pf_t.name;
    pf.offset = pf_t.offset;
    pf.datatype = pf_t.datatype;
    pf.count = pf_t.count;
    return pf;
}

/** \brief Convert a pcl::PCLPointField to had::pointfield_t
 *
 */
had::pointfield_t fromPCLPointField(const pcl::PCLPointField& pf)
{
    had::pointfield_t pf_t;
    pf_t.name = pf.name;
    pf_t.offset = pf.offset;
    pf_t.datatype = pf.datatype;
    pf_t.count = pf.count;
    return pf_t;
}

/** \brief Convert a pcl::PCLPointCloud2 to had::pointcloud_t
 *
 */
//NOTE: Not all Header information is used!!
void fromPCLPointCloud2(const pcl::PCLPointCloud2& pc, had::pointcloud_t& pc_t)
{
//     printf("trying to convert to point_cloud_t\n");
    pc_t.header.utime = pc.header.stamp;
    pc_t.header.sender = "had";
    pc_t.coord_frame = pc.header.frame_id;
    pc_t.height = pc.height;
    pc_t.width = pc.width;
    pc_t.is_dense = pc.is_dense;
    pc_t.fields.clear();
    for (auto& field : pc.fields)
        pc_t.fields.push_back(fromPCLPointField(field));
    pc_t.num_fields = pc.fields.size();
    pc_t.point_step = pc.point_step;
    pc_t.row_step = pc.row_step;

    //apply compression
    pcl::PCLPointCloud2 nonconst_pc = pc;
    char* raw_uncompressed_array = reinterpret_cast<char*>(nonconst_pc.data.data());
    std::string compressed_string;
    snappy::Compress(raw_uncompressed_array, nonconst_pc.data.size(),
                                    &compressed_string);
    pc_t.data.clear();
    pc_t.data.resize(compressed_string.length());
    std::copy(compressed_string.c_str(), compressed_string.c_str() + compressed_string.size(),
              pc_t.data.begin());
    pc_t.size = pc_t.data.size();
    //printf("\nTotal size before compression:%lu\n",pc.data.size());
	//for(auto &d : pc.data)
	//	printf("%u ", d) ;
    //printf("\nTotal size after compression:%lu\n",pc_t.data.size());
	//for(auto &d : pc_t.data)
	//	printf("%u ", d) ;
	//printf("\n") ;
}

/** \brief Convert a pcl::PCLPointCloud2 to had::pointcloud_t
 *
 */
//NOTE: Not all Header information is used!!
void toPCLPointCloud2(const had::pointcloud_t& pc_t, pcl::PCLPointCloud2& pc)
{
    pc.header.stamp = pc_t.header.utime;
    pc.header.frame_id = pc_t.coord_frame;
    pc.height = pc_t.height;
    pc.width = pc_t.width;
    pc.is_dense = pc_t.is_dense;
    pc.fields.clear();
    for (auto& field : pc_t.fields)
        pc.fields.push_back(toPCLPointField(field));
    pc.point_step = pc_t.point_step;
    pc.row_step = pc_t.row_step;

    //apply decompression
    had::pointcloud_t nonconst_pc_t = pc_t;
    char* raw_compressed_array = reinterpret_cast<char*>(nonconst_pc_t.data.data());
    std::string uncompressed_string;
    bool success = snappy::Uncompress(raw_compressed_array, nonconst_pc_t.data.size(),
                                      &uncompressed_string);
    if (!success)
        printf("Uncompress Failed!!!!\n");

    pc.data.clear();
    pc.data.resize(uncompressed_string.length());
    std::copy(uncompressed_string.c_str(),
              uncompressed_string.c_str() + uncompressed_string.size(), pc.data.begin());
    //printf("Total size before de-compression:%lu\n",pc_t.data.size());
    //printf("Total size after de-compression:%lu\n",pc.data.size());
}

