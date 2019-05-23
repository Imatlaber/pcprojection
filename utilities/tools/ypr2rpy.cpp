#include <stdlib.h>
#include <bot_core/rotations.h>
#include <bot_core/small_linalg.h>
#include <bot_core/trans.h>
#include <string.h>

int main(int argc, char * argv[])
{
	if(argc<4)
		return 1 ;

	double ypr[3] = {0} ;
	ypr[0] = atof(argv[1]) ;
	ypr[1] = atof(argv[2]) ;
	ypr[2] = atof(argv[3]) ;

	printf("ypr: %f %f %f \n", ypr[0], ypr[1], ypr[2]) ;

	double quat[4] = {0} ;
	double invert_rpy[3] = {-ypr[2], -ypr[1], -ypr[0]} ;
	bot_roll_pitch_yaw_to_quat(invert_rpy, quat) ;

	BotTrans trans ;
	memset(trans.trans_vec, 0, sizeof(trans.trans_vec)) ;
	memcpy(trans.rot_quat, quat, sizeof(quat)) ;
	
	bot_trans_invert(&trans) ;

	double rpy[3] = {0} ;
	bot_quat_to_roll_pitch_yaw(trans.rot_quat, rpy) ;
	
	printf("rpy: %f %f %f \n", rpy[0], rpy[1], rpy[2]) ;

	return 0 ;
}
