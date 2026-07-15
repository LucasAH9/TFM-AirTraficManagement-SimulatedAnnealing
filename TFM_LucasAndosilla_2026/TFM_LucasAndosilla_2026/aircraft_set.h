#ifndef AIRCRAFT_SET_H
#define AIRCRAFT_SET_H

#include <math.h>
#include <cstdlib>
#include <cstdio>
#include "tools.h"
#include "aerial_sector.h"

#define NM 0
#define VC 1
#define AC 2
#define TC 3

typedef struct Aircraft
{
	int id;
	float v; 			//Current velocity
	float vIni; 		//Initial velocity
	float vMin; 		//Minimum velocity
	float vMax; 		//Maximum velocity
	float z; 			//Current altitude
	float zIni;			//Initial altitude
	float zMin; 		//Minimum altitude
	float zMax; 		//Maximum altitude
	float x;			//Current x
	float xIni; 		//Initial x (aerial sector border)
	float xFin;			//Final x (aerial sector border)
	float y;			//Current y
	float yIni;			//Initial y (aerial sector border)
	float yFin;			//Final y (aerial sector border)
	float zFin;
	float t;			//Current t (since aircraft enters aerial sector)
	float tFin;			//Estimated t to reach the border of aerial sector with the initial conf.
	float angle;		//Current angle
	float angleIni;		//Initial angle
	float maxAngle;		//Maximum turn angle
	//float pitchAngle;
	float safetyRad; 	//Safety rad
	float safetyHeight; //Safety height
	int idManeuvers;     //Number of maneuver since enters aerial sector
} Aircraft;

typedef struct AircraftSet
{
	Aircraft* aircrafts;	//Set of aircrafts
	float** L;				//Lij
	float** G;				//Gij
	bool** pathSituations; 	//Aircraft pairs with a pathological situation
	int nAircrafts;			//Number of aircrafts
} AircraftSet;

//Inits AircraftSet (sets aircrafts, nAircraftSet, L, G and PathSituations)
void initAS(AircraftSet* aircraftSet, Aircraft* aircrafts, int nAircrafts);

//Frees AircraftSet memory
void freeAS(AircraftSet* aircraftSet);

//Returns true if exists a pathological situation
bool isPathological(float x1, float x2, float rad1, float rad2);

//Calculates tan(L) between a pair of aircrafts 
float getL(float x1, float y1, float x2, float y2, float rad1, float rad2, bool pathological);

//Calculates tan(G) between a pair of aircrafts
float getG(float x1, float y1, float x2, float y2, float rad1, float rad2, bool pathological);

//Calculates the horizontal and vertical risk
void getRisk(float* horRisk, float* verRisk, AircraftSet* aircraftSet, int index_1, int index_2, float change_1, float change_2, int man_1, int man_2);

//Gets the maneuver states given the current maneuver
void getManeuverStates(float* vel, float* alt, float* angle, int* nMan, Aircraft* aircraft, float change, int man);

//Gets the out point of the aerial sector

void getEstimatedOutPoint(float* xOut, float* yOut, float* zOut, AerialSector* aerialSector, float angle, float x, float y, float z);
//void getEstimatedOutPoint(float* xOut, float* yOut, float* zOut, AerialSector* aerialSector, float angle, float pitchAngle, float x, float y, float z, float zMin, float zMax);

#endif
