#include <stdlib.h>
#include <bot_core/rotations.h>
#include <bot_core/small_linalg.h>
#include <bot_core/trans.h>
#include <string.h>

int main(int argc, char * argv[])
{
	if(argc<4)
		return 1 ;

	double rpy[3] = {0} ;
	rpy[0] = atof(argv[1]) ;
	rpy[1] = atof(argv[2]) ;
	rpy[2] = atof(argv[3]) ;

	double factor = M_PI/180.0 ;
	bot_vector_scale_3d(rpy, factor) ;

	printf("ypr: %f %f %f \n", rpy[0], rpy[1], rpy[2]) ;

	double quat[4] = {0} ;
	bot_roll_pitch_yaw_to_quat(rpy, quat) ;
	double rot[9] = {0} ;
	bot_quat_to_matrix(quat, rot) ;
	
	printf("rot \n") ;
	printf("%f %f %f \n", rot[0], rot[1], rot[2]) ;
	printf("%f %f %f \n", rot[3], rot[4], rot[5]) ;
	printf("%f %f %f \n", rot[6], rot[7], rot[8]) ;

	return 0 ;
}
