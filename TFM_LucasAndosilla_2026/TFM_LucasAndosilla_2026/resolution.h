#ifndef RESOLUTION_H
#define RESOLUTION_H

#include <cstdio>
#include <cstdlib>

#define NOBJS 6

typedef struct Parameters
{
	float weightVar_1;			//Variance weight objective 1
	float weightVar_2;			//Variance weight objective 2
	float weightVar_4;			//Variance weight objective 4
	float weightVar_5;			//Variance weight objective 5

	float VCost;				//Velocity change cost
	float ACost;				//Altitude change cost
	float TCost;				//Turn change cost

	float envSizeMagnitude;		//Magnitude environment size
	int envSizeManeuver;		//Maneuver environment size

	float T;					//Current temperature
	float Tf; 					//Final temperature
	int L; 						//Number of iterations until temperature change
	float alpha; 				//Rate of exponential decrease of temperature

	int maxIts; 				//Maximum iterations
	int convIts;				//Maximum number of iterations without accepting solutions

	int nObjs; 					//Number of objectives
	float** ranges;				//Objective ranges [nObjs][2]

	float horVerCost;			//Importance of Vertical risk with respect Horizontal risk		
} Parameters;

typedef struct Solution
{
	float* magnitudes;	//Changes magnitudes
	int* maneuvers;		//Changes maneuvers
	float* fitness;		//Fitness of every objective
} Solution;

//Inits a solution
void initSolution(Solution* solution, int nAircrafts, int nObjs);

//Frees a solution
void freeSolution(Solution* solution);

//Copies a solution into another
void copySolution(Solution* solutionTarget, Solution* solutionSource, int nAircrafts, int nObjs);

//Updates objective value ranges
void updateRanges(float** ranges, float* fitness, int nObjs);

//Prints a solution
void printSolution(Solution* solution, int nAircrafts, int nObjs);

#endif
