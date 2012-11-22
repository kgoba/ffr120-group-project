/*
 * Boids.h
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis Goba
 */

#ifndef BOIDS_H_
#define BOIDS_H_

#include <vector>

#include "UI.h"

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

	void steer(double dt, const vector<Boid *> & neighbours, double wallDistance);
};

// Class that represents a collection of Boids.
// This is also the simulation currently.
class Boids : public DrawableObject {
public:
    // initialise with N random Boids
	Boids(int N);
	~Boids();

    // do a timestep (update Boids)
	void step(double dt);               // dt in seconds

	// draw visualisation (for OpenGL)
    virtual void draw();

private:
    // our collection of Boids (agents)
	vector<Boid *>	_boids;

	void computeNeighbours();
};

#endif /* BOIDS_H_ */
