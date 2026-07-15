#include <cstdlib>
#include <cstdio>


#ifndef STATISTICS_H
#define STATISTICS_H

typedef struct Statistic
{
	FILE* statsFile;
	float* minFit;
	bool foundSolution;
	int nIts;
} Statistic;

void initStat(Statistic* stat, int nObjs);

void closeStat(Statistic* stat);

//void storeStat(Statistic* stat,Solution* solution,float probability,int nSolutions,int nObjs);

#endif
