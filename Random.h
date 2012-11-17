/*
 * Random.h
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis goba
 */

#ifndef RANDOM_H_
#define RANDOM_H_


// A helper class for random number generation

// Currently uses drand48(), which is provided on *NIX/MacOS systems
// but not on Windows

class Random {
public:
	Random();

    // uniformly distributed random numbers
	double uniform();                       // range [0,1)
	double uniform(double max);             // range [0, max)
	double uniform(double min, double max); // range [min, max)

    // Gaussian distributed random numbers
	double gauss();                         // mu=0, sigma=1
	double gauss(double sigma);             // mu=0
	double gauss(double mu, double sigma);
};

extern Random g_random;

#endif /* RANDOM_H_ */
