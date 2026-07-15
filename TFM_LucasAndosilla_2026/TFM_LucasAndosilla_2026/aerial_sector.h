#ifndef AERIAL_SECTOR_H
#define AERIAL_SECTOR_H

typedef struct AerialSector
{
	float* pointsX; //X coordinates of the polygon corners
	float* pointsY; //Y coordinates of the polygon corners
	int nPoints;	//Number of corners of the polygon
} AerialSector;

//Inits the aerial sector
void initAerialSector(AerialSector* aerialSector,float* pointsX,float* pointsY,int nPoints);

#endif
