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

void BoidBase::move(double dt)
{
	x += vx * dt;
	y += vy * dt;
}

void BoidBase::randomizeState()
{
	x = g_random.uniform(-10, 10);
	y = g_random.uniform(-10, 10);
	vx = g_random.gauss(0.2);
	vy = g_random.gauss(0.2);
}

void Boid::randomizeParameters()
{
	k_noise = g_random.gauss(0.1, 0.02);		// velocity noise term
	k_sep = g_random.gauss(0.2, 0.04);		// separation force coefficient
	k_avoid = g_random.gauss(10, 2);		// wall-avoidance coefficient
	k_keep = g_random.gauss(0.02, 0.005);		// velocity control coefficient
	k_align = g_random.gauss(0.05, 0.01);		// velocity alignment coefficient

	r2_norm = g_random.gauss(1, 0.25);		// normal separation distance squared
	v_norm = g_random.gauss(4, 1);			// normal velocity
}

void Boid::steer(double dt, const vector<BoidBase *> & neighbours, double wallDistance)
{
    // steering force (change in velocity)
	double dvx = g_random.gauss(k_noise);
	double dvy = g_random.gauss(k_noise);

    // separation term (and cohesion term somewhat)
    for (size_t k = 0; k < neighbours.size(); k++) {
        BoidBase *nearest = neighbours[k];
        if (nearest->type() != 1) continue;
        double rx = nearest->x - this->x;
        double ry = nearest->y - this->y;
        double r2 = 1E-6 + rx*rx + ry*ry;			// distance squared
        dvx += -k_sep * rx * (1 / r2 - 1 / r2_norm);
        dvy += -k_sep * ry * (1 / r2 - 1 / r2_norm);
    }

    // velocity alignment term
    for (size_t k = 0; k < neighbours.size(); k++) {
        BoidBase *nearest = neighbours[k];
        if (nearest->type() != 1) continue;
        dvx += k_align * (nearest->vx - this->vx);
        dvy += k_align * (nearest->vy - this->vy);
    }

    // wall-avoidance term
    dvx += -k_avoid / pow(abs(this->x - wallDistance), 2);
    dvx += k_avoid / pow(abs(this->x + wallDistance), 2);
    dvy += -k_avoid / pow(abs(this->y - wallDistance), 2);
    dvy += k_avoid / pow(abs(this->y + wallDistance), 2);

    // keep-up-speed term
    double v = sqrt(this->vx*this->vx + this->vy*this->vy);
    dvx += k_keep * (v_norm - v) * this->vx / v;
    dvy += k_keep * (v_norm - v) * this->vy / v;

    // update velocity
    this->vx += dt * dvx;
    this->vy += dt * dvy;
}

void PredatorBoid::randomizeParameters()
{
	k_avoid = g_random.gauss(10, 2);		// wall-avoidance coefficient
	k_keep = g_random.gauss(0.05, 0.01);		// wall-avoidance coefficient
	k_chase = g_random.gauss(1, 0.2);		// wall-avoidance coefficient
	v_norm = g_random.gauss(8, 1);			// normal velocity
}

void PredatorBoid::steer(double dt, const vector<BoidBase *> & neighbours, double wallDistance)
{
    // steering force (change in velocity)
	double dvx = 0;
	double dvy = 0;

    // prey attraction term
    for (size_t k = 0; k < neighbours.size(); k++) {
        BoidBase *nearest = neighbours[k];
        if (nearest->type() == 1) {
            double rx = nearest->x - this->x;
            double ry = nearest->y - this->y;
            double r = sqrt(1E-6 + rx*rx + ry*ry);
            dvx += k_chase * rx / r;
            dvy += k_chase * ry / r;
        }
    }

    // wall-avoidance term
    dvx += -k_avoid / pow(abs(this->x - wallDistance), 2);
    dvx += k_avoid / pow(abs(this->x + wallDistance), 2);
    dvy += -k_avoid / pow(abs(this->y - wallDistance), 2);
    dvy += k_avoid / pow(abs(this->y + wallDistance), 2);

    // keep-up-speed term
    double v = sqrt(this->vx*this->vx + this->vy*this->vy);
    if (v > v_norm) {
        dvx += k_keep * (v_norm - v) * this->vx / v;
        dvy += k_keep * (v_norm - v) * this->vy / v;
    }
    dvx -= 0.1 * this->vx / v;
    dvy -= 0.1 * this->vy / v;

    // update velocity
    this->vx += dt * dvx;
    this->vy += dt * dvy;
}

///////////////////////////////////////////////////////////////////////////////
// Boids class

BoidSimulation::BoidSimulation(int N) {
	_boids.resize(N);
	for (size_t i = 0; i < _boids.size(); i++) {
	    Boid *boid = new Boid();
	    boid->randomizeParameters();
	    boid->randomizeState();
		_boids[i] = boid;
	}
	for (size_t i = 0; i < 3; i++) {
	    PredatorBoid *boid = new PredatorBoid();
	    boid->randomizeParameters();
	    boid->randomizeState();
		_boids.push_back(boid);
	}

	_boxSize = 30;
}

BoidSimulation::~BoidSimulation()
{
	for (unsigned i = 0; i < _boids.size(); i++) {
		delete _boids[i];
	}
}

void BoidSimulation::step(double dt) {
    computeNeighbours();

    // iterate through each boid and calculate
    // the information available to it
    // i.e. neighbours and obstacles it sees
    // pass it to each boid so it can take steering decision
	for (size_t i = 0; i < _boids.size(); i++) {
		BoidBase *current = _boids[i];

		// inefficient search for neighbours:
		// find the squared distances between the current and every other Boid
		vector<pair<double, BoidBase*> > dist(_boids.size());
		for (size_t j = 0; j < _boids.size(); j++) {
			double dx = _boids[j]->x - current->x;
			double dy = _boids[j]->y - current->y;
			dist[j] = pair<double, BoidBase*>(dx*dx + dy*dy, _boids[j]);
		}
		// sort by distances (0-th element is the current boid itself)
		sort(dist.begin(), dist.end());

        vector<BoidBase *> neighbours;
        for (size_t k = 1; k < 5; k++) {
            if (dist[k].first > 4*4) break;
            neighbours.push_back(dist[k].second);
        }

        current->steer(dt, neighbours, _boxSize / 2);
	}

    // update boid positions
	for (size_t i = 0; i < _boids.size(); i++) {
		_boids[i]->move(dt);
	}
}

void BoidSimulation::computeNeighbours() {
}


// draws the Boids as OpenGL objects
void BoidSimulation::draw() {
    static float colorTail[4]       = { 0.3, 0.5, 0.1, 0.0 };
    static float colorBoid[4]       = { 0.8, 0.7, 0.1, 0.0 };
    static float colorBoid2[4]       = { 0.9, 0.4, 0.1, 0.0 };
    static float colorBox[4]       = { 0.4, 0.4, 0.8, 0.0 };
    static double c120 = -1.0 / 2;
    static double s120 = sqrt(3) / 2;

    glColor4fv(colorBox);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-_boxSize / 2, 0, -_boxSize / 2);
    glVertex3f(-_boxSize / 2, 0, _boxSize / 2);
    glVertex3f(_boxSize / 2, 0, _boxSize / 2);
    glVertex3f(_boxSize / 2, 0, -_boxSize / 2);
    glEnd();

    glBegin(GL_TRIANGLES);
    // for every Boid
	for (unsigned i = 0; i < _boids.size(); i++) {
	    if (_boids[i]->type() == 1) glColor4fv(colorBoid);
	    else glColor4fv(colorBoid2);
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

