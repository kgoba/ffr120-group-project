/*
 * Vicsek.h
 *
 *  Created on: 2011/12/05
 *      Author(s): Karlis Goba
 */

#ifndef VICSEK_H_
#define VICSEK_H_

#include <vector>
#include <iostream>

#include "UI.h"

class VicsekSimulation  {
public:
	VicsekSimulation(int N, double boxSize, double noise, double r = 1, double v = 1);
	~VicsekSimulation();

    // do a simulation step
	void step();

    double getOrderParameter();

    void dumpCoordinates(std::ostream &os);

private:
    struct Particle {
        double x, y;
        double heading, newHeading;
    };

    // our collection of agents
	std::vector<Particle *>	_agents;

	double _boxSize;
	double _rNeighbour;
	double _velocity;
	double _noiseLevel;

    double getDistance2(const Particle *p1, const Particle *p2);
};

#endif /* VICSEK_H_ */
