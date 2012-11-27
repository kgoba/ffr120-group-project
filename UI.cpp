#include<vector>

using std::vector;

#include "UI.h"

// This OpenGL code was copied over from some sample code and modified.

// Contains OpenGL UI routines like mouse control, redrawing of the screen.

// This needs to be reorganised better in the future.

/*
void displayCallback(void);
void reshapeCallback(GLint width, GLint height);
void mouseClickCallback(int button, int state, int x, int y);
void mouseMoveCallback(int x, int y);
void menuSelectCallback(int idCommand);
void keyboardCallback(unsigned char key, int x, int y);

int buildPopupMenu (void);
*/

vector<DrawableObject *> g_objects;

#if !defined(GLUT_WHEEL_UP)
#  define GLUT_WHEEL_UP   3
#  define GLUT_WHEEL_DOWN 4
#endif

// Menu identifiers - copied from sample code
enum {
    MENU_LIGHTING = 1,
    MENU_POLYMODE,
    MENU_TEXTURING,
    MENU_EXIT
};

// OpenGL UI bookkeeping variables
static int g_width = 600;                          // Initial window width
static int g_height = 400;                         // Initial window height

static bool g_lightingEnabled = true;
static bool g_fillPolygons = true;
static bool g_button1Down = false;

static int g_yClick = 0;
static int g_xClick = 0;

static GLfloat g_azimuthAngle = 0.0;
static GLfloat g_elevationAngle = 90.0;
static GLfloat g_viewDistance = 25;

static GLfloat g_nearPlane = 1;
static GLfloat g_farPlane = 1000;
static GLfloat g_FOV = 50.;

static char *g_windowTitle = "FFR120: Project";

static float g_lightPos[4] = { -60, 20, 70, 1 };  // Position of light

void drawAxes(void)
{
    float colorAxes[4]       = { 0.6, 0.6, 0.6, 0.0 };
    glColor4fv(colorAxes);
    float w = 10;
    float scale = 0.01;
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, w, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, w);
    glEnd();
    glPushMatrix();
    glTranslatef(w, 0, 0);
    glScalef(scale, scale, 1);
    glutStrokeCharacter(GLUT_STROKE_ROMAN, 'x');
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0, w, 0);
    glScalef(scale, scale, 1);
    glutStrokeCharacter(GLUT_STROKE_ROMAN, 'y');
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0, 0, w);
    glScalef(scale, scale, 1);
    glutStrokeCharacter(GLUT_STROKE_ROMAN, 'z');
    glPopMatrix();
}

void renderObjects(void)
{
    float colorAmbient[4]    = { 0.5, 0.5, 0.5, 1.0 };
    float colorNone[4]       = { 0.0, 0.0, 0.0, 0.0 };
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT, colorAmbient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, colorNone);
    //drawAxes();
    glPopMatrix();
}

void displayCallback(void)
{
//    glClearColor(1.0, 1.0, 1.0, 0.0);
    // Clear frame buffer and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Set up viewing transformation, looking down -Z axis
    glLoadIdentity();
    glTranslatef(0, 0, -g_viewDistance);
    glRotatef(g_elevationAngle, 1, 0, 0);
    glRotatef(g_azimuthAngle, 0, 1, 0);

    // Set up the stationary light
    glLightfv(GL_LIGHT0, GL_POSITION, g_lightPos);
    // Render the scene
    renderObjects();
    for (unsigned i = 0; i < g_objects.size(); i++)
        g_objects[i]->draw();
    // Make sure changes appear onscreen
    glutSwapBuffers();
}

void reshapeCallback(GLint width, GLint height)
{
    g_width = width;
    g_height = height;
    glViewport(0, 0, g_width, g_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(g_FOV, (float)g_width / g_height, g_nearPlane, g_farPlane);
    glMatrixMode(GL_MODELVIEW);
}

void mouseClickCallback(int button, int state, int x, int y)
{
    // Respond to mouse button presses.
    // If button1 pressed, mark this state so we know in motion function.
    switch (button)
    {
    case GLUT_LEFT_BUTTON:
    {
        g_button1Down = (state == GLUT_DOWN);
        g_yClick = y;
        g_xClick = x;
        break;
    }
    case GLUT_WHEEL_UP:
    {
        g_viewDistance *= 1.1;
        break;
    }
    case GLUT_WHEEL_DOWN:
    {
        g_viewDistance /= 1.1;
        break;
    }
    case GLUT_MIDDLE_BUTTON:
        break;
    }
}

void mouseMoveCallback(int x, int y)
{
    // If button1 pressed, zoom in/out if mouse is moved up/down.
    if (g_button1Down)
    {
//        g_fViewDistance = (y - g_yClick) / 3.0;
//        if (g_fViewDistance < VIEWING_DISTANCE_MIN)
//            g_fViewDistance = VIEWING_DISTANCE_MIN;
        g_elevationAngle += (y - g_yClick) * 0.5;
        g_azimuthAngle += (x - g_xClick) * 0.5;
        g_yClick = y;
        g_xClick = x;
        //glutPostRedisplay();
    }
}


void menuSelectCallback(int idCommand)
{
    switch (idCommand)
    {
        case MENU_LIGHTING:
            g_lightingEnabled = !g_lightingEnabled;
            if (g_lightingEnabled)
                glEnable(GL_LIGHTING);
            else
                glDisable(GL_LIGHTING);
            break;
        case MENU_POLYMODE:
            g_fillPolygons = !g_fillPolygons;
            glPolygonMode (GL_FRONT_AND_BACK, g_fillPolygons ? GL_FILL : GL_LINE);
            break;
        case MENU_EXIT:
            exit (0);
            break;
    }
    // Almost any menu selection requires a redraw
    glutPostRedisplay();
}

void keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:             // ESCAPE key
            exit (0);
            break;
        case 'l':
            menuSelectCallback(MENU_LIGHTING);
            break;
        case 'p':
            menuSelectCallback(MENU_POLYMODE);
            break;
        case 't':
            menuSelectCallback(MENU_TEXTURING);
            break;
    }
}

int buildPopupMenu (void)
{
    int menu;
    menu = glutCreateMenu (menuSelectCallback);
    glutAddMenuEntry ("Toggle lighting\tl", MENU_LIGHTING);
    glutAddMenuEntry ("Toggle polygon fill\tp", MENU_POLYMODE);
    glutAddMenuEntry ("Toggle texturing\tt", MENU_TEXTURING);
    glutAddMenuEntry ("Exit demo\tEsc", MENU_EXIT);
    return menu;
}

void initOpenGL(void)
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);
//    glEnable(GL_LINE_SMOOTH);
//    glEnable(GL_POLYGON_SMOOTH);
//    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
//    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
}

void uiInit(void)
{
    // GLUT Window Initialization:
    glutInitWindowSize(g_width, g_height);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow(g_windowTitle);
    // Initialize OpenGL graphics state
    initOpenGL();
    // Register callbacks:
    glutDisplayFunc (displayCallback);
    glutReshapeFunc (reshapeCallback);
    glutKeyboardFunc (keyboardCallback);
    glutMouseFunc (mouseClickCallback);
    glutMotionFunc (mouseMoveCallback);
    // Create our popup menu
    buildPopupMenu ();
    glutAttachMenu (GLUT_RIGHT_BUTTON);
}

void uiAddObject(DrawableObject *object)
{
    g_objects.push_back(object);
}
