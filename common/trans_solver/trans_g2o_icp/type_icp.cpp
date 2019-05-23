#include "type_icp.h"


namespace g2o
{
	G2O_REGISTER_TYPE(VERTEX_ICP, Vertex_icp) ;
	G2O_REGISTER_TYPE(EDGE_ICP, Edge_icp) ;
}

/** 
 ****************************** Vertex_icp *****************************
 */

void Vertex_icp::setToOriginImpl()
{
    _estimate = g2o::Isometry3D::Identity();
}

bool Vertex_icp::read(std::istream& is)
{
	return true ;
}

bool Vertex_icp::write(std::ostream& os) const
{
	return os.good() ;
}

void Vertex_icp::oplusImpl(const double* update)
{
  Eigen::Map<const g2o::Vector6d> v(update);
  g2o::Isometry3D increment = g2o::internal::fromVectorMQT(v);
  _estimate = _estimate * increment;
  //clearCache() ;
}

void Vertex_icp::clearQuadraticForm() 
{
	_b.setZero();
	//clearCache() ;	
}

void Vertex_icp::pop()
{
	//printf("enter pop \n") ;
	g2o::BaseVertex<6, g2o::Isometry3D>::pop() ;
	//clearCache() ;
}

void Vertex_icp::clearCache()
{
	//printf("trans icp: clear cache \n") ;
	for(auto& e : edges())
	{
		Edge_icp * ee = dynamic_cast<Edge_icp*>(e) ;
		ee->clearCache() ;
		break ;
	}

}

/** 
 ****************************** Edge_icp *****************************
 */

void Edge_icp::computeError() 
{
	const Vertex_icp * v = static_cast<const Vertex_icp*>(_vertices[0]) ;
	
	const g2o::Isometry3D& pose = v->estimate() ;
	
	_error =  pose * _measurement.p1 -_measurement.p0 ;

	const auto& rot = pose.rotation() ;
	Eigen::Matrix3d cov = _measurement.cov0 + rot * _measurement.cov1 * rot.transpose() ;
	Eigen::Matrix3d icov = cov.inverse() ;
	
	_measurement.proj_mat = icov.llt().matrixU() ;
	
	Eigen::Vector3d ypr = rot.eulerAngles(2, 1, 0) ;
	rotDerivative(ypr[2], ypr[1], ypr[0], _measurement.rot_phi, _measurement.rot_theta, _measurement.rot_psi) ;
	
	////std::cout << "before " << _error.transpose() << std::endl ;
	_error = _measurement.proj_mat * _error ;
	//std::cout << "after " << _error.transpose() << std::endl ;
	//std::cout << "--------------- " << std::endl ;
	//_information.setIdentity() ;

}

void Edge_icp::rotDerivative(double phi, double theta, double psi, Eigen::Matrix3d& dR_dPhi, Eigen::Matrix3d& dR_dTheta, Eigen::Matrix3d& dR_dPsi) 
{
	double cphi = cos(phi), sphi = sin(phi);
	double ctheta = cos(theta), stheta = sin(theta);
	double cpsi = cos(psi), spsi = sin(psi);
	    
	dR_dPhi(0,0) = 0.;
	dR_dPhi(1,0) = 0.;
	dR_dPhi(2,0) = 0.;
	
	dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
	dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
	dR_dPhi(2,1) = cphi*ctheta;
	
	dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
	dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
	dR_dPhi(2,2) = -ctheta*sphi;
	
	dR_dTheta(0,0) = -cpsi*stheta;
	dR_dTheta(1,0) = -spsi*stheta;
	dR_dTheta(2,0) = -ctheta;
	
	dR_dTheta(0,1) = cpsi*ctheta*sphi;
	dR_dTheta(1,1) = ctheta*sphi*spsi;
	dR_dTheta(2,1) = -sphi*stheta;
	
	dR_dTheta(0,2) = cphi*cpsi*ctheta;
	dR_dTheta(1,2) = cphi*ctheta*spsi;
	dR_dTheta(2,2) = -cphi*stheta;
	
	dR_dPsi(0,0) = -ctheta*spsi;
	dR_dPsi(1,0) = cpsi*ctheta;
	dR_dPsi(2,0) = 0.;
	
	dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
	dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
	dR_dPsi(2,1) = 0.;
	
	dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
	dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
	dR_dPsi(2,2) = 0.;

}

void Edge_icp::linearizeOplus()
{
	auto& J = _jacobianOplusXi ;		
	J.leftCols<3>() = Eigen::Matrix3d::Identity() ;
	J.col(3) = _measurement.rot_phi * _measurement.p1 ; 
	J.col(4) = _measurement.rot_theta * _measurement.p1 ;
	J.col(5) = _measurement.rot_psi * _measurement.p1 ;
	J = _measurement.proj_mat * J ;
	//printf("J : rows %zu cols %zu \n", J.rows(), J.cols()) ;
		

	//g2o::BaseUnaryEdge<3, Meas_icp, Vertex_icp>::linearizeOplus() ;	
}

bool Edge_icp::read(std::istream& is) 
{
	return true ;
}

bool Edge_icp::write(std::ostream& os) const 
{
	
	return os.good() ;
}

void Edge_icp::clearCache()
{
	Cache_icp * cache =  (Cache_icp*)_measurement.usr_data ;
	cache->calculated = false ;	
}

