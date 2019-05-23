/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: velodyne point type
 */

#ifndef _PCL_POINT_TYPES_
#define _PCL_POINT_TYPES_

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/graph/graph_concepts.hpp>

struct PCLVelodyne
{
    PCLVelodyne(float x_, float y_, float z_, float intensity_ = 0.0)
        : x(x_),
          y(y_),
          z(z_),
          intensity(intensity_)
    {
        data[3] = 1.0f ;
    }

    PCLVelodyne()
    {
        data[3] = 1.0f ;
    }

    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float distance;
    float intensity;
    union
    {
        int ring;
        int id;
    } ;
	union
	{
		int utime1;                  // horizontal degree: 1000 mdegree = 1°
		int type;
	} ;
	union
	{
		int utime2;
		int label;
	} ;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

struct PCLVelodyneTRA
{
	PCLVelodyneTRA(float x_, float y_, float z_, float intensity_ = 0.0)
			: x(x_),
			  y(y_),
			  z(z_),
			  intensity(intensity_)
	{
		data[3] = 1.0f ;
		trajectory.setIdentity();
	}

	PCLVelodyneTRA()
	{
		data[3] = 1.0f ;
	}

	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float distance;
	float intensity;
	union
	{
		int ring;
		int id;
	} ;
	union
	{
		int utime1;                  // horizontal degree: 1000 mdegree = 1°
		int type;
	} ;
	union
	{
		int utime2;
		int label;
	} ;
	Eigen::Matrix<float, 4, 4> trajectory;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment


struct PCLVelodyneXYZNormal
{
    PCLVelodyneXYZNormal(float x_, float y_, float z_,float normal_x, float normal_y, float normal_z, float intensity_)
        : x(x_),
          y(y_),
          z(z_),
          normal_x(normal_x),
          normal_y(normal_y),
          normal_z(normal_z),
          intensity(intensity_)
    {}

    PCLVelodyneXYZNormal(){}

    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    PCL_ADD_UNION_NORMAL4D;
    PCL_ADD_EIGEN_MAPS_NORMAL4D;
    float distance;
    float intensity;
    int ring;
	union
	{
		int utime1;                  // horizontal degree: 1000 mdegree = 1°
		int type;
	} ;
	union
	{
		int utime2;
		int label;
	} ;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment


struct structure_point
{
    PCL_ADD_POINT4D;
    float x_in_sensor;
    float y_in_sensor;
    float z_in_sensor;
    float range2D;
    uint16_t ring;

    int serial_in_cloud;
    double ground_probability;
    double EstimatedGnd_height;
    double height2EsimatedGnd;
    int semantic_label;
    uint8_t is_valid;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

struct PointXYZRGBI
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	float intensity;
	inline PointXYZRGBI (float x_, float y_, float z_, float rgb_, float intensity_)
    {
      x = x_; y = y_; z = z_; data[3] = 1.0f;
      rgb = rgb_;
	  intensity = intensity_;
    }

    inline PointXYZRGBI ()
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = g = b = a = 0;
	  intensity = 0;
    }
    inline PointXYZRGBI (uint8_t _r, uint8_t _g, uint8_t _b)
    {
      x = y = z = 0.0f;
      data[3] = 1.0f;
      r = _r;
      g = _g;
      b = _b;
      a = 0;
	  intensity = 0;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PCLVelodyne,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, distance, distance)
                                   (float, intensity, intensity)
                                   (int, ring, ring)
								   (int, utime1, utime1)
								   (int, utime2, utime2)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (PCLVelodyneTRA,
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, distance, distance)
                                           (float, intensity, intensity)
                                           (int, ring, ring)
                                           (int, utime1, utime1)
                                           (int, utime2, utime2)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, intensity, intensity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (PCLVelodyneXYZNormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
				   (float, normal_x, normal_x)
				   (float, normal_y, normal_y)
				   (float, normal_z, normal_z)
                                   (float, distance, distance)
                                   (float, intensity, intensity)
                                   (int, ring, ring)
								   (int, utime1, utime1)
								   (int, utime2, utime2)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (structure_point,           // here we assume a XYZ + "test" (as fields)
                                   (float, 	x, 	x)
                                   (float, 	y, 	y)
                                   (float,	z, 	z)
                                   (float,      x_in_sensor, x_in_sensor)
                                   (float,      y_in_sensor, y_in_sensor)
                                   (float,      z_in_sensor, z_in_sensor)
                                   (uint16_t,   ring,   ring)
                                   (int,        serial_in_cloud, serial_in_cloud)
                                   (double,     ground_probability, ground_probability)
                                   (double,     EstimatedGnd_height, EstimatedGnd_height)
                                   (double,     height2EsimatedGnd, height2EsimatedGnd)
                                   (int,        semantic_label,     semantic_label)
                                   (uint8_t,       is_valid,     is_valid)

)

#endif // PCL_VELODYNE_H

