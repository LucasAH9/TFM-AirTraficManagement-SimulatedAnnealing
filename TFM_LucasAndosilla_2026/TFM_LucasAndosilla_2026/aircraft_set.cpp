#include "aircraft_set.h"
#include <iostream>

void initAS(AircraftSet* aircraftSet, Aircraft* aircrafts, int nAircrafts)
{
	Aircraft a1, a2;
	int i, j;
	aircraftSet->aircrafts = aircrafts;
	aircraftSet->nAircrafts = nAircrafts;
	aircraftSet->G = (float**)malloc(nAircrafts * sizeof(float*));
	aircraftSet->L = (float**)malloc(nAircrafts * sizeof(float*));
	aircraftSet->pathSituations = (bool**)malloc(nAircrafts * sizeof(bool*));
	for (i = 0; i < nAircrafts; i++)
	{
		aircraftSet->G[i] = (float*)malloc(nAircrafts * sizeof(float));
		aircraftSet->L[i] = (float*)malloc(nAircrafts * sizeof(float));
		aircraftSet->pathSituations[i] = (bool*)malloc(nAircrafts * sizeof(bool));
		for (j = 0; j < nAircrafts; j++)
		{
			a1 = aircraftSet->aircrafts[i];
			a2 = aircraftSet->aircrafts[j];
			if (i < j)
			{
				aircraftSet->pathSituations[i][j] = isPathological(a1.x, a2.x, a1.safetyRad, a2.safetyRad);
				aircraftSet->L[i][j] = getL(a1.x, a1.y, a2.x, a2.y, a1.safetyRad, a2.safetyRad, aircraftSet->pathSituations[i][j]);
				aircraftSet->G[i][j] = getG(a1.x, a1.y, a2.x, a2.y, a1.safetyRad, a2.safetyRad, aircraftSet->pathSituations[i][j]);
			}
			else
			{
				aircraftSet->L[i][j] = 0;
				aircraftSet->G[i][j] = 0;
				aircraftSet->pathSituations[i][j] = true;
			}
		}
	}
}

void freeAS(AircraftSet* aircraftSet)
{
	int i;
	for (i = 0; i < aircraftSet->nAircrafts; i++)
	{
		free(aircraftSet->G[i]);
		free(aircraftSet->L[i]);
		free(aircraftSet->pathSituations[i]);
	}
	free(aircraftSet->G);
	free(aircraftSet->L);
	free(aircraftSet->pathSituations);
}

bool isPathological(float x1, float x2, float rad1, float rad2)
{
	if (fabs(x1 - x2) < rad1 + rad2) return true;
	else return false;
}

float getL(float x1, float y1, float x2, float y2, float rad1, float rad2, bool pathological)
{
	if (pathological) return tan(atan((y1 - y2) / (x1 - x2)) + asin((rad1 + rad2 / 2.) / (distance(x1, y1, x2, y2) / 2.)) + PI / 2.);
	else return tan(atan((y1 - y2) / (x1 - x2)) + asin((rad1 + rad2 / 2.) / (distance(x1, y1, x2, y2) / 2.)));
}

float getG(float x1, float y1, float x2, float y2, float rad1, float rad2, bool pathological)
{
	if (pathological) return tan(atan((y1 - y2) / (x1 - x2)) - asin((rad1 + rad2 / 2.) / (distance(x1, y1, x2, y2) / 2.)) + PI / 2.);
	else return tan(atan((y1 - y2) / (x1 - x2)) - asin((rad1 + rad2 / 2.) / (distance(x1, y1, x2, y2) / 2.)));
}

void getRisk(float* horRisk, float* verRisk, AircraftSet* aircraftSet, int index_1, int index_2, float change_1, float change_2, int man_1, int man_2)
{
	float velocity_1, velocity_2, angle_1, angle_2, altitude_1, altitude_2;
	int nMan_1, nMan_2;
	int path = 0;
	getManeuverStates(&velocity_1, &altitude_1, &angle_1, &nMan_1, &aircraftSet->aircrafts[index_1], change_1, man_1);
	getManeuverStates(&velocity_2, &altitude_2, &angle_2, &nMan_2, &aircraftSet->aircrafts[index_2], change_2, man_2);
	if (aircraftSet->pathSituations[index_1][index_2])
	{
		angle_1 += PI / 2.;
		angle_2 += PI / 2.;
		path = 1;
	}

	*horRisk = (velocity_1 * sin(angle_1) - velocity_2 * sin(angle_2)) / (velocity_1 * cos(angle_1) - velocity_2 * cos(angle_2));

	if (altitude_1 > altitude_2)
	{
		*verRisk = fmax(altitude_2 - altitude_1 + aircraftSet->aircrafts[index_2].safetyHeight, altitude_2 - altitude_1 + aircraftSet->aircrafts[index_1].safetyHeight);
	}
	else
	{
		*verRisk = fmax(altitude_1 - altitude_2 + aircraftSet->aircrafts[index_1].safetyHeight, altitude_1 - altitude_2 + aircraftSet->aircrafts[index_2].safetyHeight);
	}
}

void getManeuverStates(float* vel, float* alt, float* angle, int* nMan, Aircraft* aircraft, float change, int man)
{
	switch (man)
	{
	case NM:
	{
		*vel = aircraft->v;
		*alt = aircraft->z;
		*angle = aircraft->angle;
		//*nMan = aircraft->nManeuvers;
		break;
	}
	case VC:
	{
		if (change > 0) change *= (aircraft->vMax - aircraft->v);
		else change *= (aircraft->v - aircraft->vMin);
		*vel = aircraft->v + change;
		*alt = aircraft->z;
		*angle = aircraft->angle;
		//*nMan = aircraft->nManeuvers + 1;
		break;
	}
	case AC:
	{
		if (change > 0) change *= (aircraft->zMax - aircraft->z);
		else change *= (aircraft->z - aircraft->zMin);
		*vel = aircraft->v;
		*alt = aircraft->z + change;
		*angle = aircraft->angle;
		//nMan = aircraft->nManeuvers + 1;
		break;
	}
	case TC:
	{
		change *= aircraft->maxAngle;
		*vel = aircraft->v;
		*alt = aircraft->z;
		*angle = aircraft->angle + change;
		//*nMan = aircraft->nManeuvers + 1;
		break;
	}
	}
}


//ORIGINAL

void getEstimatedOutPoint(float* xOut, float* yOut, float* zOut, AerialSector* aerialSector, float angle, float x, float y, float z)
{
	//std::cout << "  received outpoints for calculation - out point: " << *xOut << ", " << *yOut << std::endl;
	// Inicialización preventiva de seguridad para X e Y
	*xOut = x;
	*yOut = y;
	*zOut = z;

	float minDistance = HUGE_VAL;
	float d, xAux, yAux, diff;
	int i = 0;
	while (i < aerialSector->nPoints)
	{
		if (i < aerialSector->nPoints - 1)
		{
			getIntersection(&xAux, &yAux, aerialSector->pointsX[i], aerialSector->pointsY[i],
				aerialSector->pointsX[i + 1], aerialSector->pointsY[i + 1], angle, x, y);
		}
		else
		{
			getIntersection(&xAux, &yAux, aerialSector->pointsX[i], aerialSector->pointsY[i],
				aerialSector->pointsX[0], aerialSector->pointsY[0], angle, x, y);
		}

		// Protección contra indeterminación en las aristas exactas del cubo
		if (fabs(xAux - x) < 0.001f && fabs(yAux - y) < 0.001f) {
			i++;
			continue;
		}

		float headingToIntersection = atan2(yAux - y, xAux - x);
		diff = fabs(fmod((angle - headingToIntersection + PI), 2. * PI) - PI);

		// Filtro de sentido de la marcha al frente
		if (diff < 0.15f)
		{
			float d = distance(xAux, yAux, x, y);
			if (d < minDistance)
			{
				minDistance = d;
				*xOut = xAux;
				*yOut = yAux;
			}
		}
		i++;
	}
}


/*
//VERSION CON PITCHANGLE
void getEstimatedOutPoint(float* xOut, float* yOut, float* zOut, AerialSector* aerialSector,
	float angle, float pitchAngle, float x, float y, float z, float zMin, float zMax)
{
	// Inicialización preventiva por defecto en el punto actual
	*xOut = x;
	*yOut = y;
	*zOut = z;

	float minDistance2D = HUGE_VAL;
	float xAux2D = x, yAux2D = y;
	bool foundWall = false;

	// 1. Encontrar la intersección en el plano horizontal (Paredes laterales)
	int i = 0;
	while (i < aerialSector->nPoints)
	{
		float xWallIntersection, yWallIntersection;
		if (i < aerialSector->nPoints - 1)
		{
			getIntersection(&xWallIntersection, &yWallIntersection, aerialSector->pointsX[i], aerialSector->pointsY[i], aerialSector->pointsX[i + 1], aerialSector->pointsY[i + 1], angle, x, y);
		}
		else
		{
			getIntersection(&xWallIntersection, &yWallIntersection, aerialSector->pointsX[i], aerialSector->pointsY[i], aerialSector->pointsX[0], aerialSector->pointsY[0], angle, x, y);
		}

		if (fabs(xWallIntersection - x) < 0.001f && fabs(yWallIntersection - y) < 0.001f) {
			i++;
			continue;
		}

		float headingToIntersection = atan2(yWallIntersection - y, xWallIntersection - x);
		float diff = fabs(fmod((angle - headingToIntersection + PI), 2. * PI) - PI);

		if (diff < 0.15f)
		{
			float d2D = distance(xWallIntersection, yWallIntersection, x, y);
			if (d2D < minDistance2D)
			{
				minDistance2D = d2D;
				xAux2D = xWallIntersection;
				yAux2D = yWallIntersection;
				foundWall = true;
			}
		}
		i++;
	}

	// 2. Integrar la componente vertical (Z) y límites de Techo/Suelo
	if (foundWall) {
		if (fabs(pitchAngle) < 0.0001f) {
			// Caso A: El avión vuela perfectamente plano (Z constante)
			*xOut = xAux2D;
			*yOut = yAux2D;
			*zOut = z;
		}
		else {
			// Calcular la altitud teórica a la que llegaría al cruzar la pared lateral
			float zProjected = z + minDistance2D * tan(pitchAngle);

			if (zProjected > zMax) {
				// Caso B: El avión choca contra el techo antes de llegar a la pared lateral
				float dCeiling = (zMax - z) / tan(pitchAngle); // Distancia horizontal hasta el techo
				*xOut = x + dCeiling * cos(angle);
				*yOut = y + dCeiling * sin(angle);
				*zOut = zMax;
			}
			else if (zProjected < zMin) {
				// Caso C: El avión choca contra el suelo antes de llegar a la pared lateral
				float dFloor = (zMin - z) / tan(pitchAngle); // Distancia horizontal hasta el suelo
				*xOut = x + dFloor * cos(angle);
				*yOut = y + dFloor * sin(angle);
				*zOut = zMin;
			}
			else {
				// Caso D: Sale del sector de forma limpia por la pared lateral en pleno ascenso/descenso
				*xOut = xAux2D;
				*yOut = yAux2D;
				*zOut = zProjected;
			}
		}
	}
}
*/
