#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>
#include <cstdio>
#include <cstdlib>


#define PI 3.1415926535

float euclideanDistance(float* vec_1, float* vec_2, int n);

//Distance between two points
float distance(float x1, float y1, float x2, float y2);

float distance_3(float x1, float y1, float z1, float x2, float y2, float z2);

void getIntersection(float* xRes, float* yRes, float x0, float y0, float x1, float y1, float angle, float x2, float y2);

//Arcosin
float angle(float x2, float x1, float d);

int sumatorial(int n);

//Copies source_1 and source_2 in target_1 and target_2 with size n_1 and n_2
void copyTwoArrays(float* target_1, float* source_1, float* target_2, float* source_2, int n_1, int n_2);

//Copies source in target with size n
float copyArray(float* target, float* source, int n);

float minArray(int* index, float* array, int n);

float meanArray(float* array, int n);

bool equalArray(float* array_1, float* array_2, int n);

void mallocMat(float*** mat, int m, int n);

void freeMat(float** mat, int n);

void printMat(float** mat, int m, int n);

void printFloatVec(float* vec, int n);

void printIntVec(int* vec, int n);

int loadMat(float*** mat, FILE* fd, int nCols);

#endif
