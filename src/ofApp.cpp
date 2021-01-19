#include "ofApp.h"
#include "time.h"
#include <chrono>
#include <iostream>
#include <ctime>
#include <cmath>
#include <sstream>
#include <string> // std::string, std::to_string

#define LENGTH 0.5	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.16 // wheel radius
#define STARTZ 0.2	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass

static const dVector3 yunit = {0, 1, 0}, zunit = {0, 0, 1};

//--------------------------------------------------------------
void ofApp::setup()
{
    timerend = false;

    starttime = ofGetElapsedTimeMillis();

    /* Ensure texture sizes are normalized */
    /* Note that textures must be square! */
    ofDisableArbTex();

    int i;
    dMass m;
    speed = 0, steer = 0;

    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -0.5);
    ground = dCreatePlane(space, 0, 0, 1, 0);

    // chassis body
    body[0] = dBodyCreate(world);
    dBodySetPosition(body[0], 0, 0, STARTZ);
    dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
    dMassAdjust(&m, CMASS);
    dBodySetMass(body[0], &m);
    box[0] = dCreateBox(0, LENGTH, WIDTH, HEIGHT);
    dGeomSetBody(box[0], body[0]);

    // wheel bodies
    for (i = 1; i <= 4; i++)
    {
        body[i] = dBodyCreate(world);
        dQuaternion q;
        dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(body[i], q);
        dMassSetSphere(&m, 1, RADIUS);
        dMassAdjust(&m, WMASS);
        dBodySetMass(body[i], &m);
        sphere[i - 1] = dCreateSphere(0, RADIUS);
        dGeomSetBody(sphere[i - 1], body[i]);
    }

    dBodySetPosition(body[1], 0.5 * LENGTH, WIDTH * 0.5, STARTZ); //f
    dBodySetPosition(body[2], -0.5 * LENGTH, WIDTH * 0.5, STARTZ);
    dBodySetPosition(body[3], -0.5 * LENGTH, -WIDTH * 0.5, STARTZ);
    dBodySetPosition(body[4], 0.5 * LENGTH, -WIDTH * 0.5, STARTZ); //f

    // front and back wheel hinges
    for (i = 0; i < 4; i++)
    {
        joint[i] = dJointCreateHinge2(world, 0);
        dJointAttach(joint[i], body[0], body[i + 1]);
        const dReal *a = dBodyGetPosition(body[i + 1]);
        dJointSetHinge2Anchor(joint[i], a[0], a[1], a[2]);
        dJointSetHinge2Axes(joint[i], zunit, yunit);
    }
    // set joint suspension
    for (i = 0; i < 4; i++)
    {
        dJointSetHinge2Param(joint[i], dParamSuspensionERP, 0.4);
        dJointSetHinge2Param(joint[i], dParamSuspensionCFM, 0.8);
    }

    // lock back wheels along the steering axis
    for (i = 1; i < 4; i++)
    {
        // set stops to make sure wheels always stay in alignment
        dJointSetHinge2Param(joint[i], dParamLoStop, 0);
        dJointSetHinge2Param(joint[i], dParamHiStop, 0);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate(space);
    dSpaceSetCleanup(car_space, 0);
    dSpaceAdd(car_space, box[0]);
    dSpaceAdd(car_space, sphere[0]);
    dSpaceAdd(car_space, sphere[1]);
    dSpaceAdd(car_space, sphere[2]);
    dSpaceAdd(car_space, sphere[3]); //added

    // environment

    // draws the flags randomly inside the box
    for (int i = 0; i <= nflag; i++)
    {
        flags[i] = dCreateBox(space, 0.2, 0.2, 1);
        dGeomSetPosition(flags[i], ofRandom(-9.5, 9.5), ofRandom(-9.5, 9.5), 0 + 0.05); //xyzd
    }

    wall1 = dCreateBox(space, 0.1, 20, 5);
    dGeomSetPosition(wall1, 10, 0, 2.5); //xyz

    wall2 = dCreateBox(space, 0.1, 20, 5);
    dGeomSetPosition(wall2, -10, 0, 2.5); //xyz

    wall3 = dCreateBox(space, 20, 0.1, 5);
    dGeomSetPosition(wall3, 0, -10, 2.5); //xyz

    wall4 = dCreateBox(space, 20, 0.1, 5);
    dGeomSetPosition(wall4, 0, 10, 2.5); //xyz

    float camPos[3]; // position
    float camHPR[3]; // Heading, Pitch and Roll
    const dReal *buggyPos = dBodyGetPosition(body[0]);
    const dReal *buggyRot = dBodyGetRotation(body[0]);
    camPos[0] = buggyPos[0] - buggyRot[0] * 4.0;
    camPos[1] = buggyPos[1] - buggyRot[4] * 4.0;
    camPos[2] = buggyPos[2] + 1.0;
    camHPR[0] = atan2(buggyRot[4], buggyRot[0]) * 180.0 / M_PI;
    camHPR[1] = 0;
    camHPR[2] = 0;

    // Set up the OpenFrameworks camera
    ofVec3f upVector;
    upVector.set(0, 0, 1);
    cam.setAutoDistance(false);
    cam.setNearClip(0.01);
    cam.setPosition(camPos[0], camPos[1], camPos[2]);
    cam.lookAt({0, 0, 0}, upVector);
    cam.setUpAxis(upVector);
    cam.disableMouseInput();
    //dsSetViewpoint(camPos,camHPR);

    dAllocateODEDataForThread(dAllocateMaskAll);

    /* Graphics ground plane */
    m_ground.set(20, 20);				 // 8x8 plane
    m_ground.mapTexCoords(0, 0, 10, 10); // Texture tiles every 2 units
    m_ground.setResolution(128, 128);	 // How many triangles to divide the plane into.

    /* The texture is saved in the bin/data directory.
     * It was found by searching "tiling snow texture" ...
     */
    if (!ofLoadImage(m_groundTex, "snow.jpg"))
    {
        std::cerr << "Failed to load ground texture." << std::endl;
    }
    m_groundTex.setTextureWrap(GL_REPEAT, GL_REPEAT);

    /* The light */
    m_light1.setPosition(8, 8, 5);
    m_light1.lookAt(glm::vec3(0, 0, 0));
    m_light1.enable();

    /* Load the models */
    m_lowpolytree.loadModel("lowpolytree.dae", true);
    m_lowpolytree.setRotation(0, 90.0, 1, 0, 0);
    float scale;

    scale = 1.0 / m_lowpolytree.getNormalizedScale();
    m_lowpolytree.setScale(scale * 50, scale * 50, scale * 50);

    //loads the sound
    mySound.load("car.wav");
    mySound2.load("flag.mp3");
}

//--------------------------------------------------------------
void ofApp::update()
{

    ofSetVerticalSync(true);

    if (flagscollected == 30)
    {
        mySound2.play();
    }

    float camPos[3]; // position
    const dReal *buggyPos = dBodyGetPosition(body[0]);
    const dReal *buggyRot = dBodyGetRotation(body[0]);
    camPos[0] = buggyPos[0] - buggyRot[0] * 4.0;
    camPos[1] = buggyPos[1] - buggyRot[4] * 4.0;
    camPos[2] = buggyPos[2] + 1.0; // draws camera closer
    const ofVec3f lookpos = ofVec3f(buggyPos[0], buggyPos[1], buggyPos[2]);
    ofVec3f upVector = ofVec3f(0, 0, 1); //xyz

    if (!(gameOver))
    {
        //lookat.set(camPos[0], camPos[1], camPos[2]);
        cam.setPosition(camPos[0], camPos[1], camPos[2]);
        cam.lookAt(lookpos, upVector);

        // motor turn front wheels
        dJointSetHinge2Param(joint[0], dParamVel2, -speed);
        dJointSetHinge2Param(joint[0], dParamFMax2, 0.1);

        dJointSetHinge2Param(joint[3], dParamVel2, -speed);
        dJointSetHinge2Param(joint[3], dParamFMax2, 0.1);

        // steering
        dReal v = steer - dJointGetHinge2Angle1(joint[0]);
        if (v > 0.1)
            v = 0.1;
        if (v < -0.1)
            v = -0.1;
        v *= 10.0;
        dJointSetHinge2Param(joint[0], dParamVel, v);
        dJointSetHinge2Param(joint[3], dParamVel, v);
        dJointSetHinge2Param(joint[0], dParamFMax, 0.2);
        dJointSetHinge2Param(joint[3], dParamFMax, 0.2);
        dJointSetHinge2Param(joint[0], dParamLoStop, -0.75);
        dJointSetHinge2Param(joint[3], dParamLoStop, -0.75);
        dJointSetHinge2Param(joint[0], dParamHiStop, 0.75);
        dJointSetHinge2Param(joint[3], dParamHiStop, 0.75);
        dJointSetHinge2Param(joint[0], dParamFudgeFactor, 0.1);
        dJointSetHinge2Param(joint[3], dParamFudgeFactor, 0.1);
    }
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, 0.05);

    // remove all contact joints
    dJointGroupEmpty(contactgroup);

    for (int i = 0; i < nflag; i++)
    {
        if (flagb[i])
        {
            dGeomDisable(flags[i]);
            // dGeomSetPosition(-30,30);
        }
    }
}
//--------------------------------------------------------------
void ofApp::draw()
{

    // draw the scene
    ofBackground(0);

    cam.begin();

    ofEnableDepthTest();

    ofPushMatrix();

    ofSetColor(ofColor::lightGrey);
    //ofDrawGrid(0.2f,100, false, false,false,true);
    m_groundTex.bind();
    m_ground.draw();
    m_groundTex.unbind();

    ofDrawAxis(10);

    // chassis
    ofSetColor(ofColor::yellow); //FRONT
    const dReal sides[3] = {LENGTH, WIDTH, HEIGHT};
    const dReal *pos_ode = dBodyGetPosition(body[0]);
    const dReal *rot_ode = dBodyGetQuaternion(body[0]);
    drawBox(pos_ode, rot_ode, sides);

    // wheels
    ofSetColor(ofColor::black);
    for (int i = 1; i <= 4; i++)
    {
        drawCyl(dBodyGetPosition(body[i]), dBodyGetQuaternion(body[i]), 0.02f, RADIUS);
    }

    // walls
    ofSetColor(ofColor::red);
    dVector3 ss;
    dQuaternion r;
    dGeomBoxGetLengths(wall1, ss);
    drawBox(dGeomGetPosition(wall1), r, ss);

    dGeomBoxGetLengths(wall2, ss);
    drawBox(dGeomGetPosition(wall2), r, ss);

    dGeomBoxGetLengths(wall3, ss);
    drawBox(dGeomGetPosition(wall3), r, ss);

    dGeomBoxGetLengths(wall4, ss);
    drawBox(dGeomGetPosition(wall4), r, ss);

    //flags

    ofSetColor(ofColor::blueViolet);
    dReal flagspace[3];

    for (int i = 0; i <= nflag; i++)
    {
        if (!flagb[i])
        {
            dGeomBoxGetLengths(flags[1], flagspace);
            drawBox(dGeomGetPosition(flags[i]), r, flagspace);
        }
    }

    /* Draw some trees */
    for (int i = -8; i <= 8; i += 8)
        for (int j = -8; j <= 8; j += 8)
        {
            if (i == j && i == 0)
                continue;
            m_lowpolytree.setPosition(i, j, 0); // position of trees
            m_lowpolytree.drawFaces();
        }

    ofDisableDepthTest();
    cam.end();

    //timer
    float timer = ofGetElapsedTimeMillis() - starttime;
    if (timer >= 90000)
    { //std::cout<<"timer " << timer <<endl;
        timerend = true;
    }

    //this pushs the 2d matrix and pops it at the end to print instructions on the screen
    ofPushMatrix();
    ofSetColor(ofColor::white);
    ofDrawBitmapString("Collect 30 flags to WIN! || Hit the wall to LOSE ", 50, 20);
    ofDrawBitmapString("Timer:" + std::to_string(timer), 50, 30);
    ofDrawBitmapString(" INSTRUCTIONS ", 50, 45);
    ofDrawBitmapString("______________", 50, 50);
    ofDrawBitmapString("A = FORWARD ", 50, 65);
    ofDrawBitmapString("Z = BACKWARD ", 50, 85);
    ofDrawBitmapString(", = TURN LEFT ", 50, 105);
    ofDrawBitmapString(". = TURN RIGHT ", 50, 120);
    ofDrawBitmapString("Q = EXIT ", 50, 135);
    ofDrawBitmapString("P = Play Engine Sound ", 50, 150);
    ofDrawBitmapString("S = Stop Engine Sound ", 50, 165);
    ofDrawBitmapString("SPACE = BREAK & RESET STEERING ", 50, 180);
    ofDrawBitmapString("Flags Collected:" + std::to_string(flagscollected), 500, 300);

    //this draws a black background when you lose then it exits the game
    if (gameOver)
    {
        ofBackground(0);
        ofDrawBitmapString("You lose!", 50, 50);
        exit();
    }
    //this draws a black background when you win then it exits the game
    if (flagscollected == 30)
    {
        ofBackground(0);
        ofDrawBitmapString("You Won!", 50, 50);
        exit();
    }
    ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::exit()
{
    dGeomDestroy(box[0]);
    dGeomDestroy(sphere[0]);
    dGeomDestroy(sphere[1]);
    dGeomDestroy(sphere[2]);
    dGeomDestroy(sphere[3]);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
}
//--------------------------------------------------------------
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    {
        dSpaceCollide2(o1, o2, data, &nearCallback);
    }
    myApp->collide(o1, o2);
}

void ofApp::drawCyl(const dReal *pos_ode, const dQuaternion rot_ode, dReal len, dReal rad)
{
    ofCylinderPrimitive c;

    // ODE's and glm's quaternions are stored in a different order to openFrameworks:
    // ofQuaternion::ofQuaternion(float x, float y, float z, float w)
    // dQuaternion	dReal[4]	[ w, x, y, z ], where w is the real part and (x, y, z) form the vector part.
    // glm::quat(w,x,y,z)
    ofQuaternion rot_of(rot_ode[1], rot_ode[2], rot_ode[3], rot_ode[0]);

    // The OF Cylinder lies along the y axis (its length is along Y); ODE's stands tall in Z.
    // Let's fix that by creating a simple "rotate around X axis by 90 degrees" quaternion:
    ofQuaternion fix_cy;
    fix_cy.makeRotate(90, 1, 0, 0);

    // And create a final orientation by combining that first rotation with the actual orientation of
    // the cylinder we got from ODE (in of quaternion ordering):
    ofQuaternion rot_final = fix_cy * rot_of;

    // ofCylinder dimensions: its length is 80 and is radius 60
    // std::cout << c.getHeight() << ", " <<c.getRadius() << std::endl;
    // scale it to be unit length and radius * the actual size:
    c.setScale(glm::vec3(rad / 60.0, len / 60.0, rad / 60.0));

    // Use our calculated quaternion to orient the cylinder properly:
    c.setGlobalOrientation(rot_final);

    // Now set the cylinder's position according to ODE physics:
    c.setGlobalPosition(glm::vec3(pos_ode[0], pos_ode[1], pos_ode[2]));

    // Draw it:
    c.draw();
}

void ofApp::drawBox(const dReal *pos_ode, const dQuaternion rot_ode, const dReal *sides_ode)
{
    ofBoxPrimitive b;
    // ofBox dimensions: 100 * 100 * 100
    // std::cout << b.getSize() << std::endl;

    // scale it to be unit w, h, d * the actual size:
    b.setScale(glm::vec3(0.01 * sides_ode[0], 0.01 * sides_ode[1], 0.01 * sides_ode[2]));

    // Simply set the orientation based on ODE's quaternion. Since we are using glm::quat
    // this time, the ordering is the same as ODE:
    b.setGlobalOrientation(glm::quat(rot_ode[0], rot_ode[1], rot_ode[2], rot_ode[3]));

    // Now set the box's position according to ODE physics:
    b.setGlobalPosition(glm::vec3(pos_ode[0], pos_ode[1], pos_ode[2]));

    // Draw it:
    b.draw();
}

void ofApp::collide(dGeomID o1, dGeomID o2)
{
    //if i delete eACH  time will affect performance but moving it out
    int i, n;

    for (int j = 0; j < nflag; j++)
    {
        if (flagb[j])
        {
            continue;
        }
        if ((o1 == flags[j] && o2 == box[0]) || (o2 == flags[j] && o1 == box[0]))
        {
            flagb[j] = true;
            //std::cout<<"testing touched" << std:: endl;
            flagscollected++;
        }
    }

    bool touchingWall_1 = (o1 == box[0] || o1 == sphere[0] || o1 == sphere[1] || o1 == sphere[2] || o1 == sphere[3]) && ((o2 == wall1) || (o2 == wall2) || (o2 == wall3) || (o2 == wall4));
    bool touchingWall_2 = (o2 == box[0] || o2 == sphere[0] || o2 == sphere[1] || o2 == sphere[2] || o2 == sphere[3]) && ((o1 == wall1) || (o1 == wall2) || (o1 == wall3) || (o1 == wall4));
    if (touchingWall_1 || touchingWall_2)
    {
        //std::cout << "Game Over" << std::endl;
        gameOver = true;
    }
    // only collide things with the ground and wall
    int g1 = (o1 == ground || o1 == wall1 || o1 == wall2 || o1 == wall3 || o1 == wall4);
    int g2 = (o2 == ground || o2 == wall1 || o2 == wall2 || o2 == wall3 || o2 == wall4);
    if (!(g1 ^ g2))
        return;

    const int N = 10;
    dContact contact[N];
    n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
    if (n > 0)
    {
        for (i = 0; i < n; i++)
        {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                                      dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            contact[i].surface.soft_erp = 0.5;
            contact[i].surface.soft_cfm = 0.3;
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach(c,
                         dGeomGetBody(contact[i].geom.g1),
                         dGeomGetBody(contact[i].geom.g2));
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{

    switch (key)
    {
    case 'a':
    case 'A':
        speed += 0.2;
        break;
    case 'z':
    case 'Z':
        speed -= 0.2;
        break;
    case ',':
        steer -= 0.1;
        break;
    case '.':
        steer += 0.1;
        break;
    case ' ':
        speed = 0;
        steer = 0;
        break;
    case 'q':
        ofExit();
        break;
    }
    //set steering limits
    if (steer <= -0.1)
    {
        steer = -0.1;
    } //L
    if (steer >= 0.1)
    {
        steer = 0.1;
    } //R

    //set speed limits
    if (speed <= -10)
    {
        speed = -10;
    } //F
    if (speed >= 20)
    {
        speed = 20;
    } //B

    // keys p and s are used to control engine sound
    if (key == 'p')
    {
        mySound.setLoop(true);
        mySound.play();
    }
    if (key == 's')
    {
        mySound.stop();
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}
//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {}
//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {}
//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {}
//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {}
//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {}
//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {}
//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {}
//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {}
//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {}
