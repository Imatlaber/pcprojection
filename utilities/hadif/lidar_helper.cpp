/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: lidar helper
 */

#ifndef LIDAR_HELPER_CPP
#define LIDAR_HELPER_CPP

#include <Eigen/Sparse>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/impl/point_types.hpp>
#include "lidar_helper.hpp"
#include "math_helper.hpp"

template <class T>
Eigen::SparseMatrix<uint16_t> LidarHelper<T>::getSparseMat(const pcl::PointCloud<T> &pcl_pt,
                                                     double res, double canvas_size)
{
    typedef Eigen::Triplet<uint16_t> Triplet;
    std::vector<Triplet> tripletList;
    tripletList.reserve(pcl_pt.points.size());
    for(const auto& pt : pcl_pt.points)
    {
        // Remember to use getVerticalPoints when neccessary
        if(fabs(pt.x)<canvas_size && fabs(pt.y)<canvas_size)
        {
            int grid_x = (pt.x+canvas_size)/res;
            int grid_y = (pt.y+canvas_size)/res;
            tripletList.push_back(Triplet(grid_x,grid_y,1));
        }
    }
    Eigen::SparseMatrix<uint16_t>  mat(canvas_size*2./res, canvas_size*2./res);
    mat.setFromTriplets(tripletList.begin(), tripletList.end());
    return mat;
}

//get point in grid is a repeat in velodyne grid icp
template <class T>
pcl::PointCloud<pcl::PointXYZ> LidarHelper<T>::getPointInGrid(const pcl::PointCloud<T> &pcl_pt,
                                              double res, double canvas_size)
{
    Eigen::SparseMatrix<uint16_t> mat = getSparseMat(pcl_pt, res, canvas_size);
    pcl::PointCloud<pcl::PointXYZ> grid_pts;
    for (int k=0; k<mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<uint16_t>::InnerIterator it(mat,k); it; ++it)
      {
          pcl::PointXYZ grid_pt;
          grid_pt.z = 0.0; //it.value(); total points within this grid
          grid_pt.x = it.row()*res-100.0;   // row index
          grid_pt.y = it.col()*res-100.0;   // col index (here it is equal to k)
          grid_pts.push_back(grid_pt);
      }
    return grid_pts;
}

template <class T>
int LidarHelper<T>::countPointsInGrid(const pcl::PointCloud<T> &pcl_pt,
                      double res, double canvas_size)
{
    Eigen::SparseMatrix<uint16_t> mat = getSparseMat(pcl_pt, res, canvas_size);
    return mat.nonZeros();
}

template <class T>
pcl::PointCloud<T> LidarHelper<T>::getDownsamplePointCloud(typename pcl::PointCloud<T>::Ptr pc_ptr, double res)
{
    pcl::PointCloud<T> pc_downsampled;
    pcl::VoxelGrid<T> sor;
    sor.setInputCloud (pc_ptr);
    sor.setLeafSize (res, res, res);
    sor.filter (pc_downsampled);
    std::cout<<"Downsample from "<<pc_ptr->size()<<" to "<<pc_downsampled.size()<<std::endl;
    return pc_downsampled;
}

//Split pointclouds in a specified 2D square grid and perform voxel downsampling
template <class T>
LidarHelper<T>::LidarHelper(double res, double grid_size)
    :downsample_res_(res), grid_size_(grid_size){}

template <class T>
void LidarHelper<T>::insertPoints(pcl::PointCloud<T> &pc)
{
    //seperate into regions as specified by grid_size_
    for(T pt : pc.points)
    {
        int grid_x = pt.x/grid_size_;
        int grid_y = pt.y/grid_size_;
        int hash_xy = szudzik_map(grid_x, grid_y);
        if(smaller_pc_segments_.find(hash_xy)==smaller_pc_segments_.end())
        {
            typename pcl::PointCloud<T>::Ptr single_pc_seg(new pcl::PointCloud<T>);
            smaller_pc_segments_[hash_xy] = single_pc_seg;
        }
        smaller_pc_segments_[hash_xy]->push_back(pt);
    }
}

template <class T>
void LidarHelper<T>::getAllPoints(pcl::PointCloud<T> &all_pc)
{
    all_pc.clear();
    std::cout<<"Segments: "<<smaller_pc_segments_.size()<<std::endl;

    for(typename std::map<int, typename pcl::PointCloud<T>::Ptr>::iterator it=smaller_pc_segments_.begin();
        it!=smaller_pc_segments_.end(); it++)
    {
        typename pcl::PointCloud<T>::Ptr pcl_ptr = it->second;
        all_pc += getDownsamplePointCloud(pcl_ptr, downsample_res_);
    }
}

template <class T>
template <typename T1, typename T2>
cv::Mat LidarHelper<T>::getImg(const pcl::PointCloud<T1> &pc, T2 value, double res, double canvas, int CV_TYPE) 
{
    cv::Mat mat = cv::Mat::zeros(2*canvas/res, 2*canvas/res, CV_TYPE);
    for(T1 pt : pc.points)
    {
        int grid_x = (pt.x+canvas)/res;
        int grid_y = mat.rows-(pt.y+canvas)/res;
        if(grid_x>=0 && grid_x<mat.cols && grid_y>=0 && grid_y<mat.rows)
        {
            mat.at<T2>(grid_y, grid_x) = value;
        }
    }
    return mat;
}

//explicitly instantiate template classes
template class LidarHelper<PCLVelodyne>;
template cv::Mat LidarHelper<PCLVelodyne>::getImg(const pcl::PointCloud<pcl::PointXYZ> &pc, uint8_t value, double res, double canvas, int CV_TYPE) ;


#endif // LIDAR_HELPER_CPP

