/*
 * Boids.cpp
 *
 *  Created on: 2011/11/10
 *      Author(s): Karlis goba
 */

#include "Boids.h"
#include "Random.h"

#include <GL/gl.h>

#include <cmath>
#include <vector>
#include <algorithm>

using std::vector;
using std::pair;
using std::sort;

///////////////////////////////////////////////////////////////////////////////
// Boid structure

void Boid::randomize()
{
	x = g_random.uniform(-10, 10);
	y = g_random.uniform(-10, 10);
	vx = g_random.gauss();
	vy = g_random.gauss();
}

void Boid::move(double dt)
{
	x += vx * dt;
	y += vy * dt;
}

void Boid::steer(double dt, const vector<Boid *> & neighbours, double wallDistance)
{
	double k_noise = 0.1;		// velocity noise term
	double k_sep = 0.2;		// separation force coefficient
	double k_avoid = 10;		// wall-avoidance coefficient
	double k_keep = 0.05;		// velocity control coefficient
	double k_align = 0.05;		// velocity alignment coefficient

	double r2_norm = 1;			// normal separation distance squared
	double v_norm = 3;			// normal velocity

    // steering force (change in velocity)
	double dvx = g_random.gauss(k_noise);
	double dvy = g_random.gauss(k_noise);

    // separation term (and cohesion term somewhat)
    for (size_t k = 0; k < neighbours.size(); k++) {
        Boid *nearest = neighbours[k];
        double rx = nearest->x - this->x;
        double ry = nearest->y - this->y;
        double r2 = 1E-6 + rx*rx + ry*ry;			// distance squared
        dvx += -k_sep * rx * (1 / r2 - 1 / r2_norm);
        dvy += -k_sep * ry * (1 / r2 - 1 / r2_norm);
    }

    // velocity alignment term
    for (size_t k = 0; k < neighbours.size(); k++) {
        Boid *nearest = neighbours[k];
        dvx += k_align * (nearest->vx - this->vx);
        dvy += k_align * (nearest->vy - this->vy);
    }

    // wall-avoidance term
    dvx += k_avoid * (this->x - wallDistance) / pow(abs(this->x - wallDistance), 3);
    dvx += k_avoid * (this->x + wallDistance) / pow(abs(this->x + wallDistance), 3);
    dvy += k_avoid * (this->y - wallDistance) / pow(abs(this->y - wallDistance), 3);
    dvy += k_avoid * (this->y + wallDistance) / pow(abs(this->y + wallDistance), 3);

    // keep-up-speed term
    double v = sqrt(this->vx*this->vx + this->vy*this->vy);
    dvx += k_keep * (v_norm - v) * this->vx / v;
    dvy += k_keep * (v_norm - v) * this->vy / v;

    // update velocity
    this->vx += dt * dvx;
    this->vy += dt * dvy;
}

///////////////////////////////////////////////////////////////////////////////
// Boids class

Boids::Boids(int N) {
	_boids.resize(N);
	for (size_t i = 0; i < _boids.size(); i++) {
		_boids[i] = new Boid();
		_boids[i]->randomize();
	}
}

Boids::~Boids()
{
	for (unsigned i = 0; i < _boids.size(); i++) {
		delete _boids[i];
	}
}

void Boids::step(double dt) {
    computeNeighbours();
    //updatePreyPopulation();

    // iterate through each boid and calculate
    // the information available to it
    // i.e. neighbours and obstacles it sees
    // pass it to each boid so it can take steering decision
	for (size_t i = 0; i < _boids.size(); i++) {
		Boid *current = _boids[i];

		// inefficient search for neighbours:
		// find the squared distances between the current and every other Boid
		vector<pair<double, Boid*> > dist(_boids.size());
		for (size_t j = 0; j < _boids.size(); j++) {
			double dx = _boids[j]->x - current->x;
			double dy = _boids[j]->y - current->y;
			dist[j] = pair<double, Boid*>(dx*dx + dy*dy, _boids[j]);
		}
		// sort by distances (0-th element is the current boid itself)
		sort(dist.begin(), dist.end());

        vector<Boid *> neighbours;
        for (size_t k = 1; k < 5; k++) {
            if (dist[k].first > 9) break;
            neighbours.push_back(dist[k].second);
        }

        current->steer(dt, neighbours, 15);
	}

    // update boid positions
	for (size_t i = 0; i < _boids.size(); i++) {
		_boids[i]->move(dt);
	}
}

void Boids::computeNeighbours() {
}


// draws the Boids as OpenGL objects
void Boids::draw() {
    static float colorTail[4]       = { 0.3, 0.5, 0.1, 0.0 };
    static float colorBoid[4]       = { 0.9, 0.3, 0.1, 0.0 };
    static double c120 = -1.0 / 2;
    static double s120 = sqrt(3) / 2;

    glColor4fv(colorTail);
    glBegin(GL_TRIANGLES);
    // for every Boid
	for (unsigned i = 0; i < _boids.size(); i++) {
	    // draw it as triangle pointing to its velocity vector
		double x1 = _boids[i]->vx * 0.25;
		double y1 = _boids[i]->vy * 0.25;
		// calculate the other 2 corners of the triangle by rotation
		double x2 = 0.5 * (x1 * c120 - y1 * s120);
		double y2 = 0.5 * (x1 * s120 + y1 * c120);
		double x3 = 0.5 * (x1 * c120 + y1 * s120);
		double y3 = 0.5 * (-x1 * s120 + y1 * c120);
		// send coordinates to OpenGL
		glVertex3d(_boids[i]->x + x1, 0, _boids[i]->y + y1);
		glVertex3d(_boids[i]->x + x2, 0, _boids[i]->y + y2);
		glVertex3d(_boids[i]->x + x3, 0, _boids[i]->y + y3);
	}
	glEnd();
}

