/*
 * Boids.h
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis Goba
 */

#ifndef BOIDS_H_
#define BOIDS_H_

#include <vector>

using std::vector;

// Structure containing the data of a single Boid.
//   Provides two convenience methods for randomization
//   and moving (updating the position).
struct Boid
{
	double x, y;            // position
	double vx, vy;          // velocity

	void randomize();
	void move(double dt);   // dt in seconds
};

// Class that represents a collection of Boids.
// This is also the simulation currently.
class Boids {
public:
    // initialise with N random Boids
	Boids(int N);

    // do a timestep (update Boids)
	void step(double dt);               // dt in seconds

	// draw visualisation (for OpenGL)
	void draw();

private:
    // our collection of Boids (agents)
	vector<Boid>	_boids;
};

#endif /* BOIDS_H_ */
