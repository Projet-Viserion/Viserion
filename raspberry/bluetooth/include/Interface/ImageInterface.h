#ifndef INTERFACES_VOIT_H
#define INTERFACES_VOIT_H

typedef struct positionVoiture{
	int side;
	int dist;
	float percentageSide;
}positionVoiture;

void initCarPosition();

positionVoiture* linkPositionVoiture();

int32_t* linkCameraSpeedLimit();

#endif
