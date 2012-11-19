#include <GL/glut.h>
#include <GL/gl.h>

#ifndef _UI_H_
#define _UI_H_

// pure abstract class for our simulation object(s)
class DrawableObject
{
public:
    virtual void draw() = 0;
};

void uiInit(void);
void uiAddObject(DrawableObject *object);

#endif
