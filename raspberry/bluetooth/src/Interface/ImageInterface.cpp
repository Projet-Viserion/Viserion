#include <stdint.h>
#include <Interface/ImageInterface.h>


positionVoiture PosVoit;
int32_t speed_limit = 100;

void initCarPosition(){
	PosVoit.side = -1;
	PosVoit.dist = -1;
	PosVoit.percentageSide = -1.0;

}

positionVoiture* linkPositionVoiture(){
	return &PosVoit;
}

int32_t* linkCameraSpeedLimit()
{
	return &speed_limit;
}


