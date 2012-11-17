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
	x = g_random.uniform(-1, 1);
	y = g_random.uniform(-1, 1);
	vx = g_random.gauss();
	vy = g_random.gauss();
}

void Boid::move(double dt)
{
	x += vx * dt;
	y += vy * dt;
}


///////////////////////////////////////////////////////////////////////////////
// Boids class

Boids::Boids(int N) {
	_boids.resize(N);
	for (unsigned i = 0; i < _boids.size(); i++) {
		_boids[i].randomize();
	}
}

void Boids::step(double dt) {
	// for each boid:
	//		find neighbours within radius
	//		calculate different steering terms
	//		update velocity/position

    // different parameters - should move them elsewhere
	double k_noise = 0.01;		// velocity noise term
	double k_sep = 0.01;		// separation force coefficient
	double k_avoid = 0.5;		// wall-avoidance coefficient
	double k_keep = 0.01;		// velocity control coefficient
	double k_align = 0.001;		// velocity alignment coefficient

	double r2_norm = 1;			// normal separation distance squared
	double v_norm = 4;			// normal velocity

	for (unsigned i = 0; i < _boids.size(); i++) {
		Boid &current = _boids[i];

		// inefficient search for neighbours:
		// find the distances between the current and every other Boid
		vector<pair<double, int> > dist(_boids.size());
		for (unsigned j = 0; j < _boids.size(); j++) {
			double dx = _boids[j].x - current.x;
			double dy = _boids[j].y - current.y;
			dist[j] = pair<double, int>(dx*dx + dy*dy, j);
		}
		// sort by distances (0-th element is the current boid itself)
		sort(dist.begin(), dist.end());

		// steering force (change in velocity)
		double dvx = g_random.gauss(k_noise);
		double dvy = g_random.gauss(k_noise);

		// separation term (and cohesion term somewhat)
		for (unsigned k = 1; k <= 3; k++) {
			Boid &nearest = _boids[dist[k].second];
			double rx = nearest.x - current.x;
			double ry = nearest.y - current.y;
			double r2 = 1E-6 + dist[k].first;			// distance squared
			dvx += -k_sep * rx * (1 / r2 - 1 / r2_norm);
			dvy += -k_sep * ry * (1 / r2 - 1 / r2_norm);
		}

		// alignment term
		for (unsigned k = 1; k <= 3; k++) {
			Boid &nearest = _boids[dist[k].second];
			dvx += k_align * (nearest.vx - current.vx);
			dvy += k_align * (nearest.vy - current.vy);
		}

		// wall-avoidance term
		dvx += k_avoid * (current.x - 10) / pow(abs(current.x - 10), 3);
		dvx += k_avoid * (current.x + 10) / pow(abs(current.x + 10), 3);
		dvy += k_avoid * (current.y - 10) / pow(abs(current.y - 10), 3);
		dvy += k_avoid * (current.y + 10) / pow(abs(current.y + 10), 3);

		// keep-up-speed term
		double v = sqrt(current.vx*current.vx + current.vy*current.vy);
		dvx += k_keep * (v_norm - v) * current.vx / v;
		dvy += k_keep * (v_norm - v) * current.vy / v;

		// update velocity
		current.vx += dvx;
		current.vy += dvy;

        // slow-down term
//		current.vx *= 0.99;
//		current.vy *= 0.99;
	}

	for (unsigned i = 0; i < _boids.size(); i++) {
		_boids[i].move(dt);
	}
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
		double x1 = _boids[i].vx * 0.25;
		double y1 = _boids[i].vy * 0.25;
		// calculate the other 2 corners of the triangle by rotation
		double x2 = 0.5 * (x1 * c120 - y1 * s120);
		double y2 = 0.5 * (x1 * s120 + y1 * c120);
		double x3 = 0.5 * (x1 * c120 + y1 * s120);
		double y3 = 0.5 * (-x1 * s120 + y1 * c120);
		// send coordinates to OpenGL
		glVertex3d(_boids[i].x + x1, 0, _boids[i].y + y1);
		glVertex3d(_boids[i].x + x2, 0, _boids[i].y + y2);
		glVertex3d(_boids[i].x + x3, 0, _boids[i].y + y3);
	}
	glEnd();
}

