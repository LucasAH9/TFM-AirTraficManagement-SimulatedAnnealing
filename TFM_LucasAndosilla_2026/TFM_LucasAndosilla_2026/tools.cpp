#include "tools.h"

float euclideanDistance(float* vec_1, float* vec_2, int n)
{
	float acum = 0;
	int i = 0;
	for (i = 0; i < n; i++)
	{
		acum += (vec_1[i] - vec_2[i]) * (vec_1[i] - vec_2[i]);
	}
	return sqrt(acum);
}

float distance(float x1, float y1, float x2, float y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float distance_3(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

void getIntersection(float* xRes, float* yRes, float x0, float y0, float x1, float y1, float angle, float x2, float y2)
{
	float y3 = y2 + 10 * sin(angle);
	float x3 = x2 + 10 * cos(angle);
	float A1 = y1 - y0;
	float B1 = x0 - x1;
	float C1 = A1 * x0 + B1 * y0;
	float A2 = y3 - y2;
	float B2 = x2 - x3;
	float C2 = A2 * x2 + B2 * y2;
	float det = A1 * B2 - A2 * B1;
	if (det == 0)
	{
		*xRes = HUGE_VAL;
		*yRes = HUGE_VAL;
	}
	else
	{
		*xRes = (B2 * C1 - B1 * C2) / det;
		*yRes = (A1 * C2 - A2 * C1) / det;
	}
}

float angle(float x2, float x1, float d)
{
	return acos((x2 - x1) / d);
}

int sumatorial(int n)
{
	int acum = 0;
	int i;
	for (i = 0; i < n; i++)
	{
		acum += i;
	}
	return acum;
}

void copyTwoArrays(float* target_1, float* source_1, float* target_2, float* source_2, int n_1, int n_2)
{
	copyArray(target_1, source_1, n_1);
	copyArray(target_2, source_2, n_2);
}

float copyArray(float* target, float* source, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		target[i] = source[i];
	}
	return 0;
}

float minArray(int* index, float* array, int n)
{
	float min = 10000000000;
	float element;
	int i;
	for (i = 0; i < n; i++)
	{
		element = array[i];
		if (element < min)
		{
			*index = i;
			min = element;
		}
	}
	return min;
}

float meanArray(float* array, int n)
{
	float acum = 0;
	int i;
	for (i = 0; i < n; i++)
	{
		acum += array[i];
	}
	return acum / (float)n;
}

float minArrayDiffIndex(int* index, float* array, int n, int diffIndex)
{
	float min = 10000000000;
	float element;
	int i;
	for (i = 0; i < n; i++)
	{
		if (i != diffIndex)
		{
			element = array[i];
			if (element < min)
			{
				*index = i;
				min = element;
			}
		}
	}
	return min;
}

bool equalArray(float* array_1, float* array_2, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		if (array_1[i] != array_2[i])
		{
			return false;
		}
	}
	return true;
}

void mallocMat(float*** mat, int m, int n)
{
	*mat = (float**)malloc(m * sizeof(float*));
	int i;
	for (i = 0; i < m; i++)
	{
		(*mat)[i] = (float*)malloc(n * sizeof(float));
	}
}

void freeMat(float** mat, int m)
{
	int i;
	for (i = 0; i < m; i++)
	{
		free(mat[i]);
	}
	free(mat);
}

void printMat(float** mat, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			printf("%f ", mat[i][j]);
		}
		printf("\n");
	}
}

void printFloatVec(float* vec, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		printf("%f ", vec[i]);
	}
	printf("\n");
}

void printIntVec(int* vec, int n)
{
	int i;
	for (i = 0; i < n; i++)
	{
		printf("%d ", vec[i]);
	}
	printf("\n");
}

int loadMat(float*** mat, FILE* fd, int nCols)
{
	int nRows;
	fscanf_s(fd, "%d\n", &nRows);
	mallocMat(mat, nRows, nCols);
	int i;
	for (i = 0; i < nRows; i++)
	{
		switch (nCols)
		{
		case 2:
		{
			fscanf_s(fd, "%f %f\n", &(*mat)[i][0], &(*mat)[i][1]);
			break;
		}
		case 3:
		{
			fscanf_s(fd, "%f %f %f\n", &(*mat)[i][0], &(*mat)[i][1], &(*mat)[i][2]);
			break;
		}
		case 4:
		{
			fscanf_s(fd, "%f %f %f %f\n", &(*mat)[i][0], &(*mat)[i][1], &(*mat)[i][2], &(*mat)[i][3]);
			break;
		}
		case 5:
		{
			fscanf_s(fd, "%f %f %f %f %f\n", &(*mat)[i][0], &(*mat)[i][1], &(*mat)[i][2], &(*mat)[i][3], &(*mat)[i][4]);
			break;
		}
		case 6:
		{
			fscanf_s(fd, "%f %f %f %f %f %f\n", &(*mat)[i][0], &(*mat)[i][1], &(*mat)[i][2], &(*mat)[i][3], &(*mat)[i][4], &(*mat)[i][5]);
			break;
		}
		}
	}
	return nRows;
}
