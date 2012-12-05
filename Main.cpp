#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <GL/glut.h>
#include <GL/gl.h>

#include "Boids.h"
#include "Vicsek.h"
#include "UI.h"

// Contains the main() function and a global simulation instance,
// as well as timer control for updating the simulation status.


// Global instance of our simulation class
BoidSimulation g_sim(50);

const int g_frameInterval = 40;


// Periodic timer callback
void animateSceneCallback(int value)
{
    glutTimerFunc(g_frameInterval, animateSceneCallback, 0);

    double dt = (double)g_frameInterval / 1000;
    g_sim.step(dt);

    // Force redraw
    glutPostRedisplay();
}

int uiMain(int argc, char** argv)
{
    glutInit(&argc, argv);
    uiInit();
    uiAddObject(&g_sim);
    // Start animation timer
    glutTimerFunc (g_frameInterval, animateSceneCallback, 0);
    // Turn the flow of control over to GLUT
    glutMainLoop();
    return 0;
}

int simMain(int argc, char** argv)
{
    double noise = 0.1;
    int maxtime = 100;
    if (argc >= 2) {
        sscanf(argv[1], "%lf", &noise);
    }
    if (argc >= 3) {
        sscanf(argv[2], "%d", &maxtime);
    }

    VicsekSimulation sim(100, 10, noise);
    int t = 0;
    while (t < maxtime) {
        sim.step();
        t++;
        sim.dumpCoordinates(std::cout);
        //double m = sim.getOrderParameter();
        //std::cout << t << '\t' << m << std::endl;
    }
}

int main(int argc, char** argv)
{
    //uiMain(argc, argv);
    simMain(argc, argv);
}
