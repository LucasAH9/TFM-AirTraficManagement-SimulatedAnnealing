#include "statistics.h"

void initStat(Statistic* stat, int nObjs)
{
	//stat->statsFile = fopen("statistics.csv", "w+");
	errno_t err = fopen_s(&stat->statsFile, "statistics.csv", "w+");

	if (err != 0) {
		// Manejar error si no se pudo abrir el archivo
		printf("Error al abrir el archivo.\n");
	}
	stat->minFit = (float*)malloc(nObjs * sizeof(float));
	stat->nIts = 0;
	stat->foundSolution = false;
	int i;
	for (i = 0; i < nObjs; i++)
	{
		stat->minFit[i] = RAND_MAX;
	}
}

void closeStat(Statistic* stat)
{
	free(stat->minFit);
	fclose(stat->statsFile);
}

/*
void storeStat(Statistic* stat,Solution* solution,float probability,int nSolutions,int nObjs)
{
	int i;
	for(i=0;i<nObjs;i++)
	{
		if((solution->fitness[i] < stat->minFit[i]) && (solution->fitness[1] < 0))
		{
			stat->minFit[i] = solution->fitness[i];
			stat->foundSolution = true;
		}
		if(stat->foundSolution == true)
		{
			fprintf(stat->statsFile,"%f ",stat->minFit[i]);
		}
	}
	if(stat->foundSolution == true)
	{
		fprintf(stat->statsFile,"%d ",nSolutions);
		fprintf(stat->statsFile,"%f\n",probability);
		stat->nIts++;
	}
}
*/