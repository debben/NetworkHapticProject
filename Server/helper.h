#ifndef HelperHD_H_
#define HelperHD_H_

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#include <HD/hd.h>
#include <HDU/hduVector.h>


void initGlut(int argc, char* argv[]);

void initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF, int&, int&);
void drawString(const char* );
/* Draws the cartesian axes. */
void drawAxes(double axisLength);
void drawHapticsString(char*,const hduVector3Dd,const float, const GLfloat*);
/* Draws a sphere to represent an electric charge. */
void drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius);

/* Draws the force vector. */
void drawForceVector(GLUquadricObj* pQuadObj,
                     const hduVector3Dd &position,
                     const hduVector3Dd &forceVector,
                     double arrowThickness);
void setupGraphicsState();

#endif /* HelperHD_H_ */

/******************************************************************************/
