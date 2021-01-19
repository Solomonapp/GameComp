#include "ode/ode.h"
#include "ofApp.h"
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"

#ifndef PALLETOBJECT_H
#define PALLETOBJECT_H

/* This class wraps an ODE physics entity and its graphics elements
 * together into a single game object.
 */
class PalletObject
{
public:
    PalletObject(float x, float y, float z, dWorldID w, dSpaceID s);

    /* ODE objects */
    dBodyID m_body;
    dMass   m_mass;
    dGeomID m_geom;

    /* The 3D model */
    ofxAssimpModelLoader m_model;

    /* Attributes of this object */
    float x, y, z;
    ofQuaternion m_rotation;

    void setPosition(float x, float y, float z);
    void draw();

    bool debug_draw = false;

    /* The length, width, height of the pallet */
    const float c_len=1,c_wid=1,c_hei=0.1;
};

#endif // PALLETOBJECT_H
