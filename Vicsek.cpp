/*
 * Vicsek.cpp
 *
 *  Created on: 2011/12/05
 *      Author(s): Karlis goba
 */

#include "Vicsek.h"
#include "Random.h"

#include <GL/gl.h>

#include <cmath>
#include <vector>
#include <algorithm>

using std::vector;
using std::ostream;
using std::pair;
using std::sort;

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

VicsekSimulation::VicsekSimulation(int N, double boxSize, double noise, double r, double v) {
	_boxSize = boxSize;
	_rNeighbour = r;
	_velocity = v;
	_noiseLevel = noise;
	_agents.resize(N);
	for (size_t i = 0; i < _agents.size(); i++) {
	    Particle *agent = new Particle();
	    agent->x = g_random.uniform(boxSize);
	    agent->y = g_random.uniform(boxSize);
	    agent->heading = g_random.uniform(2*M_PI);
		_agents[i] = agent;
	}
}

VicsekSimulation::~VicsekSimulation()
{
	for (unsigned i = 0; i < _agents.size(); i++) {
		delete _agents[i];
	}
}

double VicsekSimulation::getDistance2(const VicsekSimulation::Particle *p1, const VicsekSimulation::Particle *p2) {
    double dx = fabs(p1->x - p2->x);
    if (_boxSize - dx < dx) dx = _boxSize - dx;

    double dy = fabs(p1->y - p2->y);
    if (_boxSize - dy < dy) dy = _boxSize - dy;

    return dx*dx + dy*dy;
}


void VicsekSimulation::step() {
    // iterate through each boid and calculate
    // the information available to it
    // i.e. neighbours and obstacles it sees
    // pass it to each boid so it can take steering decision
    size_t N = _agents.size();
    double r2 = _rNeighbour * _rNeighbour;
	for (size_t i = 0; i < N; i++) {
		Particle *current = _agents[i];

		// inefficient search for neighbours:
		// find the squared distances between the current and every other Boid
		vector<pair<double, Particle*> > dist(N);
		for (size_t j = 0; j < N; j++) {
		    double d2 = getDistance2(current, _agents[j]);
		    dist[j] = pair<double, Particle*>(d2, _agents[j]);
		}
		// sort by distances (0-th element is the current boid itself)
		sort(dist.begin(), dist.end());

        // compute close neighbours
        vector<Particle *> neighbours;
        for (size_t k = 1; k < N; k++) {
            if (dist[k].first > r2) break;
            neighbours.push_back(dist[k].second);
        }
        if (neighbours.size() == 0) continue;

        // compute average heading of neighbours
        double avgHeading = 0;
        for (size_t k = 0; k < neighbours.size(); k++) {
            Particle *neighbour = neighbours[k];
            avgHeading += neighbour->heading;
        }
        avgHeading /= neighbours.size();

        // add noise term
        current->newHeading = avgHeading + g_random.uniform(_noiseLevel);
        while (current->newHeading < 0) current->newHeading += 2*M_PI;
        while (current->newHeading > 2*M_PI) current->newHeading -= 2*M_PI;
	}

    // update boid positions
	for (size_t i = 0; i < N; i++) {
		Particle *current = _agents[i];
		current->heading = current->newHeading;
		double vx = _velocity * cos(current->heading);
		double vy = _velocity * sin(current->heading);
		current->x += vx;
		current->y += vy;
		if (current->x > _boxSize) current->x -= _boxSize;
		if (current->x < 0) current->x += _boxSize;
		if (current->y > _boxSize) current->y -= _boxSize;
		if (current->y < 0) current->y += _boxSize;
	}
}

void VicsekSimulation::dumpCoordinates(ostream &os)
{
	for (unsigned i = 0; i < _agents.size(); i++) {
	    os << i << '\t' << _agents[i]->x << '\t' << _agents[i]->y << std::endl;
	}
}

double VicsekSimulation::getOrderParameter()
{
    size_t N = _agents.size();
    double avgVx = 0, avgVy = 0;
	for (size_t i = 0; i < N; i++) {
		Particle *current = _agents[i];
		avgVx += cos(current->heading);
		avgVy += sin(current->heading);
	}
	avgVx /= N;
	avgVy /= N;
	return sqrt(avgVx * avgVx + avgVy * avgVy);
}
