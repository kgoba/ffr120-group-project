/*
 * Random.cpp
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis goba
 */

#include "Random.h"

#include <cstdlib>
#include <ctime>
#include <cmath>

Random g_random;

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

Random::Random() {
	srand48(time(0));
}

double Random::uniform() {
	return drand48();
}

double Random::uniform(double max) {
	return max * drand48();
}

double Random::uniform(double min, double max) {
	return (max - min) * drand48() + min;
}

double Random::gauss()
{
	static double U, V;
	static int step = 0;
	double Z;

	if(step == 0) {
		U = (lrand48() + 1.) / (((long long)1<<31) + 1.);
		V = lrand48() / (((long long)1<<31) + 1.);
		Z = sqrt(-2 * log(U)) * sin(2 * M_PI * V);
	} else
		Z = sqrt(-2 * log(U)) * cos(2 * M_PI * V);

	step = 1 - step;

	return Z;
}

double Random::gauss(double sigma)
{
	return gauss()*sigma;
}

