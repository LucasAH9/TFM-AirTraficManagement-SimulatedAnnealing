#include "aerial_sector.h"

void initAerialSector(AerialSector* aerialSector,float* pointsX,float* pointsY,int nPoints)
{
	aerialSector->nPoints = nPoints;
	aerialSector->pointsX = pointsX;
	aerialSector->pointsY = pointsY;
}
