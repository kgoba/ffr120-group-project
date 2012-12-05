/*
 * Boids.h
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis Goba
 */

#ifndef BOIDS_H_
#define BOIDS_H_

#include <vector>
#include <iostream>

#include "UI.h"

using std::vector;
using std::ostream;

struct BoidBase
{
    // current state
	double x, y;            // position
	double vx, vy;          // velocity

    virtual int type() = 0;

	void randomizeState();

	virtual void steer(double dt, const vector<BoidBase *> & neighbours, double wallDistance) = 0;

	void move(double dt);   // dt in seconds
};

// Structure containing the data of a single Boid.
//   Provides two convenience methods for randomization
//   and moving (updating the position).
struct Boid : public BoidBase
{
    // behaviour parameters
	double k_noise;		// velocity noise term
	double k_sep;	    // separation force coefficient
	double k_avoid;		// wall-avoidance coefficient
	double k_keep;		// velocity control coefficient
	double k_align;		// velocity alignment coefficient
	double k_flee;
	double r2_norm;		// normal separation distance squared
	double v_norm;		// normal velocity

	void randomizeParameters();

	void steer(double dt, const vector<BoidBase *> & neighbours, double wallDistance);

	int type() { return 1; }
};

struct PredatorBoid : public BoidBase
{
    // behaviour parameters
	double k_avoid;		// wall-avoidance coefficient
	double k_keep;		// velocity control coefficient
	double v_norm;		// normal/maximum velocity
	double k_chase;

	void randomizeParameters();

	void steer(double dt, const vector<BoidBase *> & neighbours, double wallDistance);

	int type() { return 2; }
};


// Class that represents a collection of Boids.
// This is also the simulation currently.
class BoidSimulation : public DrawableObject {
public:
    // initialise with N random Boids
	BoidSimulation(int N);
	~BoidSimulation();

    // do a timestep (update Boids)
	void step(double dt);               // dt in seconds

	// draw visualisation (for OpenGL)
    virtual void draw();

    void dumpCoordinates(ostream &os);

private:
    // our collection of Boids (agents)
	vector<BoidBase *>	_boids;

	double _boxSize;

	void computeNeighbours();
};

#endif /* BOIDS_H_ */
