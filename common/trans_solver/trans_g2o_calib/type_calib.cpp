#include "type_calib.h"

namespace g2o
{
	G2O_REGISTER_TYPE(VERTEX_CALIB, Vertex_calib) ;
	G2O_REGISTER_TYPE(EDGE_CALIB, Edge_calib) ;
}

/** 
 ****************************** Vertex_calib *****************************
 */

void Vertex_calib::setToOriginImpl()
{
    _estimate = g2o::Isometry3D::Identity();
}

bool Vertex_calib::read(std::istream& is)
{
	return true ;
}

bool Vertex_calib::write(std::ostream& os) const
{
	return os.good() ;
}

void Vertex_calib::oplusImpl(const double* update)
{
  Eigen::Map<const g2o::Vector6d> v(update);
  g2o::Isometry3D increment = g2o::internal::fromVectorMQT(v);
  _estimate = _estimate * increment;
  clearCache() ;
}

void Vertex_calib::clearQuadraticForm() 
{
	//printf("enter calib clearQuadraticForm \n") ;
	_b.setZero();
	//clearCache() ;
}

void Vertex_calib::clearCache()
{
	for(auto& e : edges())
	{
		Edge_calib * ee = dynamic_cast<Edge_calib*>(e) ;
		ee->clearCache() ;
		break ;
	}
}

/** 
 ****************************** Edge_calib *****************************
 */

void Edge_calib::computeError() 
{
	const Vertex_calib * v = static_cast<const Vertex_calib*>(_vertices[0]) ;
	
	const g2o::Isometry3D& pose = v->estimate() ;
	
	static Eigen::Isometry3d all_pose ;
	static Eigen::Matrix3d proj_mat ;

	Cache_calib * cache =  (Cache_calib*)_measurement.usr_data ;
	if(!cache->calculated)
	{
		all_pose = pose.inverse() * cache->move * pose ;
		const auto& rot = all_pose.rotation() ;
		Eigen::Matrix3d cov = _measurement.cov0 + rot * _measurement.cov1 * rot.transpose() ;
		Eigen::Matrix3d icov = cov.inverse() ;
		proj_mat = icov.llt().matrixU() ;
		cache->calculated = true ;
	}
	
	_error = _measurement.p0 - all_pose * _measurement.p1 ;
	//std::cout << "before " << _error.transpose() << std::endl ;
	_error = proj_mat * _error ;
	//std::cout << "after " << _error.transpose() << std::endl ;
	//std::cout << "--------------- " << std::endl ;
	//_information.setIdentity() ;

}

bool Edge_calib::read(std::istream& is) 
{
	return true ;
}

bool Edge_calib::write(std::ostream& os) const 
{
	
	return os.good() ;
}

void Edge_calib::clearCache()
{
	Cache_calib * cache =  (Cache_calib*)_measurement.usr_data ;
	cache->calculated = false ;	
}

