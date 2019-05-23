/* Copyright (c) AutoNavi - All Rights Reserved 
 * Author: shuo <shuo@alibaba-inc.com>
 * Description: base class for transformation solver 
 */

#ifndef _TRANS_SOLVER_BASE_HPP_
#define _TRANS_SOLVER_BASE_HPP_

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

template<class T, class D>
class TransSolverBase
{
	public:
	TransSolverBase() {}
	
	void set_source_pc(typename pcl::PointCloud<T>::ConstPtr source_pc) 
	{
		source_pc_ = source_pc ;
	}

	void set_target_pc(typename pcl::PointCloud<T>::ConstPtr target_pc)
	{
		target_pc_ = target_pc ;
	}

	void set_initial_trans(const D& trans0)
	{
		trans0_ = trans0 ;
	}

	virtual void set_usr_data(void * data)  
	{
		data_ = data ;
	}

	virtual bool run() = 0 ;
	
	virtual const D& get_trans() const = 0 ;
	
	public:

	typename pcl::PointCloud<T>::ConstPtr source_pc_ ;
	typename pcl::PointCloud<T>::ConstPtr target_pc_ ;
	D trans0_ ;	
	void * data_ ;
	

} ;



#endif // _TRANS_SOLVER_6D_HPP_
