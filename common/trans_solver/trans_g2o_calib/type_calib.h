#ifndef _TYPE_CALIB_H_
#define _TYPE_CALIB_H_

#include <iostream>
#include <Eigen/Core>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/base_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/vertex_se3.h"

struct Cache_calib
{
	Cache_calib():
	calculated(false)
	{
	}

	bool calculated ;
	Eigen::Isometry3d move ;
} ;


class Vertex_calib : public g2o::BaseVertex<6, Eigen::Isometry3d>
{
	public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
    void setToOriginImpl() override ;

	bool read(std::istream& is) override  ;
		
	bool write(std::ostream& os) const override ;

    void oplusImpl(const double* update) override ;
	
	void clearQuadraticForm() override ;

	void clearCache() ;
} ;

class Meas_calib
{
	public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	Eigen::Vector3d p0 ;
	Eigen::Vector3d p1 ;
	Eigen::Matrix3d cov0 ;		
	Eigen::Matrix3d cov1 ;
	void * usr_data ;
} ;

class Edge_calib : public g2o::BaseUnaryEdge<3, Meas_calib, Vertex_calib>
{
	public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	void computeError() override ;
	
	bool read(std::istream& is) override  ;
		
	bool write(std::ostream& os) const override ;

	void clearCache() ;

} ;




#endif

