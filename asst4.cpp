////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <list>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "arcball.h"
#include "matrix4.h"
#include "quat.h"
#include "rigtform.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
//lab5
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
//lab6
#include "sgutils.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>      // std::istringstream
//lab7
#include "geometry.h"
#include "uniforms.h"
#include "texture.h"
#include "material.h"
//lab8
#include "mesh.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event

//additional global
static bool g_worldview = true;// when g_world view is true, world-sky frame, else sky-sky frame
//static int target_obj = 2;//o = 0 is first cube, 1 is second cube, 2 is sky deprecate in asst4
static int vision_obj = 2;//e = 0 is robot1 node, 1 is robot2 node, 2 is sky_node 
static double g_arcballSDcreenRadius = 0.25*min(g_windowWidth, g_windowHeight);
static double g_arcballScale = 0;
static bool g_arcball_act = true; // when arcball is active in view
static bool g_pressP = false;
//lab6
static list<vector<RigTForm>*> keyFrames;
static vector<shared_ptr<SgRbtNode>> dumpNodes;
static int g_frames_index = -1;
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback
static bool g_animation = false;
static const char *filename = "keyFrames_suho.txt";
//lab 7
static shared_ptr<Material> g_redDiffuseMat, g_blueDiffuseMat, g_bumpFloorMat, g_arcballMat, g_pickingMat, g_lightMat;
//lab 9
static const char *meshFile = "cube.mesh";
static shared_ptr<Material> g_meshMat;

shared_ptr<Material> g_overridingMaterial;

// --------- Geometry
//typedef SgGeometryShapeNode<Geometry> MyShapeNode;
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground, cube and sphere geometry
static shared_ptr<Geometry>  g_cube, g_ground, g_sphere;
// --------- Scene
static RigTForm *g_sphereRbt=new RigTForm(Cvec3(0, 0, 0));
//in asst4 snitpets
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_eyeNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking
//lab 7
static shared_ptr<SgRbtNode> g_light1Node, g_light2Node;
//lab 9
static shared_ptr<SgRbtNode> g_meshNode;
static shared_ptr<Geometry> g_mesh;
///////////////// END OF G L O B A L S //////////////////////////////////////////////////

//lab 7
static void initGround() {
	int ibLen, vbLen;
	getPlaneVbIbLen(vbLen, ibLen);

	// Temporary storage for cube Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
	g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
	int ibLen, vbLen;
	getCubeVbIbLen(vbLen, ibLen);

	// Temporary storage for cube Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeCube(1, vtx.begin(), idx.begin());
	g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(20, 10, vbLen, ibLen);

	// Temporary storage for sphere Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);
	makeSphere(1, 20, 10, vtx.begin(), idx.begin());
	g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void makeScaledSphere(double scale) {
	int ibLen, vbLen;
	getSphereVbIbLen(20, 10, vbLen, ibLen);

	// Temporary storage for sphere Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);
	makeSphere(scale, 20, 10, vtx.begin(), idx.begin());

	g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

//lab8 Load Mesh File using load
static void initMesh() {
	Mesh mesh;
	mesh.load(meshFile);
	int numFace = mesh.getNumFaces();
	vector<VertexPN> vtx;
	for (int i = 0; i < numFace; i++) {
		Mesh::Face tmpFace = mesh.getFace(i);
		Cvec3 normal = tmpFace.getNormal();
		const int numV = tmpFace.getNumVertices();
		for (int j = 0; j < numV; j=j+2) {
			//Seperate face by triangle
			for (int k = 0; k < 3; k++) {
				//make triangle
				Mesh::Vertex tmpVertex = tmpFace.getVertex((j + k)%numV);
				Cvec3 position = tmpVertex.getPosition();
				VertexPN tmpPN = VertexPN(position[0], position[1], position[2], normal[0], normal[1], normal[2]);
				vtx.push_back(tmpPN);
			}
		}
	}
	g_mesh.reset(new SimpleGeometryPN(&vtx[0], vtx.size()));
}


// takes a projection matrix and send to the the shaders after lab7
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
	uniforms.put("uProjMatrix", projMatrix);
}


// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
	if (g_windowWidth >= g_windowHeight)
		g_frustFovY = g_frustMinFov;
	else {
		const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
		g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
	}
}

static Matrix4 makeProjectionMatrix() {
	return Matrix4::makeProjection(
		g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
		g_frustNear, g_frustFar);
}

static void setArc() {
	const RigTForm eyeRbt = getPathAccumRbt(g_world, g_eyeNode);
	const RigTForm invEyeRbt = inv(eyeRbt);
	(*g_sphereRbt) = getPathAccumRbt(g_world, g_currentPickedRbtNode);
	double z = (inv(invEyeRbt) * (*g_sphereRbt)).getTranslation()[2];
	//get z cordination
	if (z <= -CS175_EPS) {
		g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight);
	}
}

static void drawStuff(bool picking) {
	// Declare an empty uniforms
	Uniforms uniforms;

	// build & send proj. matrix to vshader
	const Matrix4 projmat = makeProjectionMatrix();
	sendProjectionMatrix(uniforms, projmat);

	const RigTForm eyeRbt = getPathAccumRbt(g_world, g_eyeNode);
	const RigTForm invEyeRbt = inv(eyeRbt);
	Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
	Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();
	const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // light1 position in eye coordinates
	const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // light2 position in eye coordinates
	uniforms.put("uLight", eyeLight1);
	uniforms.put("uLight2", eyeLight2);

	Matrix4 MVM;
	Matrix4 NMVM;
	if (!picking) {
		// draw ground and robot
		// ===========
		// initialize the drawer with our uniforms, as opposed to curSS
		Drawer drawer(invEyeRbt, uniforms);

		// draw as before
		g_world->accept(drawer);
		
		// draw arcball as part of asst3
		// when obj is sky, sphere is exist when vision is sky and world-sky mode
		g_arcball_act = (vision_obj == 2 && (g_worldview||g_currentPickedRbtNode!=g_groundNode)) || (vision_obj != 2 && g_currentPickedRbtNode!=g_groundNode);
		if (g_arcball_act)//re draw ths arcball
		{
			//switch to wire frame mode
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			if (g_arcballScale == 0) {
				double z = (invEyeRbt * (*g_sphereRbt)).getTranslation()[2];//get z cordination
				if (z <= -CS175_EPS) {
					g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight);
				}
			}
			makeScaledSphere(g_arcballScale*g_arcballSDcreenRadius);
			MVM = rigTFormToMatrix(invEyeRbt * (*g_sphereRbt));
			NMVM = normalMatrix(MVM);
			sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));
			//draw sphrere
			g_arcballMat->draw(*g_sphere, uniforms);
			//switch back to solid mode
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}
	else {
		//Picking State
		// intialize the picker with our uniforms, as opposed to curSS
		Picker picker(invEyeRbt, uniforms);
		// set overiding material to our picking material
		g_overridingMaterial = g_pickingMat;

		g_world->accept(picker);
		// unset the overriding material
		g_overridingMaterial.reset();

		glFlush();
		g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
		g_pressP = false;//Initialize the state of picking
		if (g_currentPickedRbtNode == shared_ptr<SgRbtNode>()) {
			g_currentPickedRbtNode = g_groundNode;   // set to NULL
			cout << "No Part Picked" << endl;
		}
		cout << "Picking mode is " << (g_pressP ? "on" : "off") << endl;

		setArc();
		glutPostRedisplay();
	}
}

//lab 7 pick
static void pick() {
	// We need to set the clear color to black, for pick rendering.
	// so let's save the clear color
	GLdouble clearColor[4];
	glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

	glClearColor(0, 0, 0, 0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// No more glUseProgram
	drawStuff(true); // no more curSS

	// Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
	// to see result of the pick rendering pass
	// glutSwapBuffers();

	//Now set back the clear color
	glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

	checkGlErrors();
}

//lab 7 display
static void display() {
	// No more glUseProgram

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawStuff(false); // no more curSS

	glutSwapBuffers();

	checkGlErrors();
}

//For lab6
//Find iterator in keyFrames which current index index start 0
static list<vector<RigTForm>*>::iterator findItofFrames(int index) {
	list<vector<RigTForm>*>::iterator frame_it = keyFrames.begin();
	int count = 0;
	while (count < index) {
		count++;
		if (frame_it != keyFrames.end())
			frame_it++;
		else
			printf("index error in findItofFrames which input index: %d, keyFrames size is %d\n", index, keyFrames.size());
	}
	return frame_it;
}

//insert KeyFrame return iterator of new keyFrame in keyFrames
static list<vector<RigTForm>*>::iterator insertFrame() {
	vector<RigTForm> *frame = new vector<RigTForm>();
	vector<shared_ptr<SgRbtNode>>::iterator it;
	//Get Rbt current state from dump
	for (it = dumpNodes.begin(); it != dumpNodes.end(); it++) {
		RigTForm tmpRigT = (*(*it)).getRbt();
		frame->push_back(tmpRigT);
	}
	list<vector<RigTForm>*>::iterator frame_it = findItofFrames(g_frames_index);
	printf("Copying scene graph to current frame[%d]\n", g_frames_index);
	if (frame_it == keyFrames.begin()) {
		//First
		keyFrames.push_front(frame);
		return keyFrames.begin();
	}//else
	return keyFrames.insert(frame_it, frame); //return iterator of new keyFrame in keyFrames
}

static void resetRbtFromFrame() {
	list<vector<RigTForm>*>::iterator frame_it = findItofFrames(g_frames_index);
	vector<RigTForm> * current_frame;

	current_frame = *frame_it;
	vector<RigTForm>::iterator rigt_it;
	vector<shared_ptr<SgRbtNode>>::iterator dump_it;
	for (rigt_it = current_frame->begin(), dump_it = dumpNodes.begin();
		rigt_it != current_frame->end() && dump_it != dumpNodes.end();
		rigt_it++, dump_it++) {
		(*(*dump_it)).setRbt(*rigt_it);
	}
}

/*
If the current key frame is defined, delete the current key frame and do the following:
- If the list of frames is empty after the deletion, set the current key frame to undefined.
- Otherwise
	- If the deleted frame is not the first frame, set the current frame to the frame immediately before the deleted frame
	- Else set the current frame to the frame immediately after the deleted frame
	- Copy RBT data from the new current frame to the scene graph.
*/
static void delete_core() {
	printf("deleting current frame[%d]\n", g_frames_index);
	list<vector<RigTForm>*>::iterator frame_it = findItofFrames(g_frames_index);
	delete *frame_it;
	keyFrames.erase(frame_it);
	if (g_frames_index>0)
		g_frames_index--;
	printf("Now at frame [%d]\n", g_frames_index);

	if (keyFrames.empty()) {
		printf("No frames Defined\n");
		g_frames_index--;
	}
	else {
		resetRbtFromFrame();
	}
}

static Cvec3 lerp(Cvec3 t1_, Cvec3 t2_, double alpha_) {
	return t1_*(1 - alpha_) + t2_*alpha_;
}

//Lab 8 CRS construction, change lerp
static Cvec3 crs(Cvec3 t0_, Cvec3 t1_, Cvec3 t2_, Cvec3 t3_, double alpha_) {
	//t0_, t1_, t2_, t3_ is mapping c_i-1, c_i, c_i+1, c_i+2
	Cvec3 d = (t2_ - t0_) * 1 / 6 + t1_;
	Cvec3 e = (t3_ - t1_) * -1 / 6 + t2_;
	return t1_*pow((1 - alpha_), 3.0) + d * 3 * alpha_ * pow((1 - alpha_), 2.0) + e * 3 * pow((alpha_), 2.0) * (1 - alpha_) + t2_*pow((alpha_), 3.0);
}
static Quat quat_power(Quat q_, double alpha) {
	int cn = 1;
	double sin_value = sqrt(abs(1 - q_[0] * q_[0]));//No abs it can give sqrt(- value)
	Cvec3 axis = Cvec3(0, 0, 0);
	if (sin_value > CS175_EPS2) {//prevent divide zero
		axis = Cvec3(q_[1] / sin_value, q_[2] / sin_value, q_[3] / sin_value);
	}

	if (q_[0] < 0)
		cn = -1;
	q_ = q_*cn;//conditionaly negate
	double pi = atan2(sin_value, q_[0]);
	pi = alpha*pi;
	sin_value = sin(pi);
	return Quat(cos(pi), axis*sin_value);
}

static Quat slerp(Quat r1_, Quat r2_, double alpha_) {
	const double n = norm2(r1_);
	Quat q_ = r1_;
	if (n > CS175_EPS2) {
		q_ = r2_*inv(r1_);
	}
	Quat rotation_q;
	rotation_q = quat_power(q_, alpha_);
	return rotation_q*r1_;
}

//Lab 8 CRS construction, Quaternion Splining
static Quat quat_splining(Quat r0_, Quat r1_, Quat r2_, Quat r3_, double alpha_) {
	//r0_, r1_, r2_, r3_ is mapping c_i-1, c_i, c_i+1, c_i+2
	Quat d = quat_power(r2_*inv(r0_), 1 / 6) * r1_;
	Quat e = quat_power(r3_*inv(r1_), -1 / 6) * r2_;
	Quat p01 = slerp(r1_,d,alpha_);
	Quat p12 = slerp(d, e, alpha_);
	Quat p23 = slerp(e, r2_, alpha_);
	Quat p012 = slerp(p01, p12, alpha_);
	Quat p123 = slerp(p12, p23, alpha_);
	return slerp(p012, p123, alpha_);
}

// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if we are at the end of the animation
// sequence, or false otherwise.
bool interpolateAndDisplay(float t) { 
	int t_index = floor(t);
	if (t >= (float)(keyFrames.size() - 3))//Finish the animation
		return true;
	if ((float)t_index == t) {
		printf("\n%d frame start\n", t_index + 1);
	}
	double alpha = (double)t - (double)t_index;
	vector<RigTForm> *frame0, *frame1, *frame2, *frame3;
	vector<RigTForm>::iterator  fit0, fit1, fit2, fit3; // It mapping c_i-1,c_i, c_i+1,c_i+2
	list<vector<RigTForm>*>::iterator frame_it;
	vector<shared_ptr<SgRbtNode>>::iterator dump_it = dumpNodes.begin();
	
	frame_it = findItofFrames(t_index);
	frame0 = *frame_it;
	frame_it = findItofFrames(t_index + 1);
	frame1 = *frame_it;
	frame_it = findItofFrames(t_index + 2);
	frame2 = *frame_it;
	frame_it = findItofFrames(t_index+3);
	frame3 = *frame_it;
	
	fit0 = frame0->begin(); 
	fit1 = frame1->begin(); 
	fit2 = frame2->begin(); 
	fit3 = frame3->begin();

	for (; fit0 != frame0->end() &&fit1 != frame1->end() && fit2 != frame2->end() && fit3 != frame3->end();
		fit0++, fit1++, fit2++, fit3++, dump_it++) {
		//Cvec3 newT = lerp((*fit1).getTranslation(), (*fit2).getTranslation(), alpha);
		Cvec3 newT = crs((*fit0).getTranslation(), (*fit1).getTranslation(), (*fit2).getTranslation(), (*fit3).getTranslation(), alpha);
		//Quat newQ = slerp((*fit1).getRotation(), (*fit2).getRotation(), alpha);
		Quat newQ = quat_splining((*fit0).getRotation(), (*fit1).getRotation(), (*fit2).getRotation(), (*fit3).getRotation(), alpha);
		RigTForm newRigT = RigTForm(newT,newQ);
		(*(*dump_it)).setRbt(newRigT);
	}

	setArc();
	glutPostRedisplay();
	return false;
}

// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
	float t = (float)ms / (float)g_msBetweenKeyFrames;
	bool endReached = interpolateAndDisplay(t);
	endReached = endReached || !g_animation;//When g_animation true stop it
	if (!endReached) {
		glutTimerFunc(1000 / g_animateFramesPerSecond,
			animateTimerCallback,
			ms + 1000 / g_animateFramesPerSecond);
	}
	else {
		//Finished
		g_frames_index = keyFrames.size() - 2;
		printf("Finished playing animation\n");
		printf("Now at frame [%d]\n", g_frames_index);
		resetRbtFromFrame();
		g_animation = false;
		glutPostRedisplay();
	}
}

/*current key frame to the first frame. Copy this frame to the scene graph.*/
static void read_core() {
	//Erase the frames without current frame
	printf("Read Start\n");
	list<vector<RigTForm>*>::iterator frame_it;
	for (frame_it = keyFrames.begin(); frame_it != keyFrames.end();) {
		list<vector<RigTForm>*>::iterator erase_it = frame_it;
		frame_it++;
		delete *erase_it;
		keyFrames.erase(erase_it);
		g_frames_index--;
	}
	//Erase Done Now Read data
	ifstream f(filename, ios::binary);
	int numFrames, numRbtsPerFrames;
	int cFrame=0, crbt = 0;
	string line;
	f >> numFrames >> numRbtsPerFrames;
	if (f.is_open())
	{
		vector<RigTForm> *newVector;
		while (getline(f, line))
		{
			if (cFrame == numFrames) break;
			std::istringstream iss(line);
			if (crbt == 0)//New Frame
			{
				newVector = new vector<RigTForm>();
			}
			RigTForm newRbt;
			Cvec3 t_;
			Quat q_;
			if (!(iss >> t_[0] >> t_[1] >> t_[2] >> q_[0] >> q_[1] >> q_[2] >> q_[3])) {continue;}
			crbt++;
			newRbt = RigTForm(t_, q_);
			newVector->push_back(newRbt);
			if (crbt == numRbtsPerFrames)//Read all rbt in frame
			{
				crbt = 0;
				keyFrames.push_back(newVector);
				vector<RigTForm>::iterator rbt_it;
				cFrame++;

			}
		}
		f.close();
		g_frames_index = 0;
		resetRbtFromFrame();
		setArc();
		glutPostRedisplay();
		printf("read file Done\n");
	}
	else {
		printf("Read Fail\n");
	}
	
}

/*output key frames to output file. Make sure file format is consistent with input format*/
static void write_core() {
	/*output key frames to output file. Make sure file format is consistent with input format*/
	ofstream outfile(filename, ofstream::binary);
	list<vector<RigTForm>*>::iterator frame_it;
	vector<RigTForm> *frame = *(keyFrames.begin());
	outfile << keyFrames.size() << ' ' << frame->size() << '\n';
	int count = 0;
	for (frame_it = keyFrames.begin(); frame_it != keyFrames.end(); frame_it++, count++) {
		vector<RigTForm>::iterator rbt_it;
		frame = *frame_it;
		//outfile << "#" << count << '\n';
		for (rbt_it = frame->begin(); rbt_it != frame->end(); ++rbt_it) {
			Cvec3 t_ = (*rbt_it).getTranslation();
			Quat r_ = (*rbt_it).getRotation();
			outfile << t_[0] << ' ' << t_[1] << ' ' << t_[2] << ' ';
			outfile << r_[0] << ' ' << r_[1] << ' ' << r_[2] << ' ' << r_[3] << '\n';
		}
	}
	outfile.close();
}
// lab6 done

static void reshape(const int w, const int h) {
	g_windowWidth = w;
	g_windowHeight = h;
	glViewport(0, 0, w, h);
	g_arcballSDcreenRadius = 0.25*min(g_windowWidth, g_windowHeight);
	cerr << "Size of window is now " << w << "x" << h << endl;
	updateFrustFovY();
	glutPostRedisplay();
}

// handling input about motion, mouse button key press
//Using in motion, input is dx,dy
static void _egomotion(const double dx, const double dy, RigTForm m_) {
	//Perform ego motion
	RigTForm m = m_;
	//Change When left or right
	if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down
		m = RigTForm(Cvec3(-dx, -dy, 0) *g_arcballScale);
	}
	else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
		m = RigTForm(Cvec3(0, 0, dy) * g_arcballScale);
	}
	else if (g_mouseLClickButton && !g_mouseRClickButton) {
		m = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
	}
	RigTForm targetRbt;
	RigTForm eyeRbt = getPathAccumRbt(g_world, g_eyeNode);
	shared_ptr<SgRbtNode> targetNode;

	targetRbt = doMtoOwrtA(inv(m), eyeRbt, eyeRbt);
	(*g_sphereRbt) = targetRbt;
	if (g_currentPickedRbtNode == g_groundNode) {
		targetNode = (vision_obj == 0) ? g_robot1Node : g_robot2Node;
		(*targetNode).setRbt(targetRbt);
		g_eyeNode = targetNode;
	}
	else {
		(*g_currentPickedRbtNode).setRbt(targetRbt);
		g_eyeNode = g_currentPickedRbtNode;
	}
}

static void motion(const int x, const int y) {
	const double dx = x - g_mouseClickX;
	const double dy = g_windowHeight - y - 1 - g_mouseClickY;
	Cvec2 sphere_o;
	Cvec3 v1, v2;
	double v1_x = 0.0, v1_y = 0.0, v1_z = 0.0;
	double v2_x = 0.0, v2_y = 0.0, v2_z = 0.0;
	//pixel the center of Sphere
	RigTForm m;
	Quat q;
	RigTForm eyeRbt = getPathAccumRbt(g_world, g_eyeNode);
	if (!g_arcball_act) g_arcballScale = 0.01;
	//Get motion M
	//left button Click, rotation
	if (g_mouseLClickButton && !g_mouseRClickButton) {
		if (g_arcball_act) {
			double z = (inv(eyeRbt) * (*g_sphereRbt)).getTranslation()[2];//Do near Test;
			//If z is big(near by screen) We can't rotation if not picked body(if picked body, we can ego motion, We do under! in _egomotion)
			if (z <= -CS175_EPS ) {
				sphere_o = getScreenSpaceCoord((inv(eyeRbt) * (*g_sphereRbt)).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
				v1_x = g_mouseClickX - sphere_o(0);
				v1_y = g_mouseClickY - sphere_o(1);
				if ((pow(v1_x, 2) + pow(v1_y, 2)) > pow(g_arcballSDcreenRadius, 2))
				{
					v1_z = 0;
				}
				else {
					v1_z = sqrt(pow(g_arcballSDcreenRadius, 2) - pow(v1_x, 2) - pow(v1_y, 2));
				}
				v1 = Cvec3(v1_x, v1_y, v1_z);
				v1.normalize();

				//get V2
				v2_x = x - sphere_o(0);
				v2_y = g_windowHeight - y - 1 - sphere_o(1);
				if ((pow(v2_x, 2) + pow(v2_y, 2)) > pow(g_arcballSDcreenRadius, 2))
				{
					v2_z = 0;
				}
				else {
					v2_z = sqrt(pow(g_arcballSDcreenRadius, 2) - pow(v2_x, 2) - pow(v2_y, 2));
				}
				v2 = Cvec3(v2_x, v2_y, v2_z);
				v2.normalize();
				q = Quat(dot(v1, v2), cross(v1, v2));
			}
			else{//exception No rotation
				q = Quat();
			}
		}
		else {
			q = Quat::makeXRotation(-dy) * Quat::makeYRotation(dx);
		}
		m = RigTForm(q);
	}
	else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down
		m = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale);
	}
	else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
		m = RigTForm(Cvec3(0, 0, -dy) * g_arcballScale);
	}

	//Get M done Move the Object or sky using m
	RigTForm affine;
	RigTForm parentRbt;
	if (g_mouseClickDown) {
		if (vision_obj==0 && g_currentPickedRbtNode == g_robot1Node) {
			//perform egomotion for robot1
			_egomotion(dx, dy,m);
		}
		else if (vision_obj == 1 && g_currentPickedRbtNode == g_robot2Node) {
			//perform egomotion for robot2
			_egomotion(dx, dy,m);
		}
		else if (g_currentPickedRbtNode == g_groundNode) {
			//When pick is ground
			if (vision_obj == 0 || vision_obj == 1) {
				_egomotion(dx, dy,m);
			}
			else {
				//when vision is sky
				if (g_worldview) {
					//move the eye
					affine = makeMixedFrame(RigTForm(), eyeRbt);
				}
				else {
					//sky-sky means ego motion can's using egmotion func because m is different
					if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  
						// middle or (left and right) button down?
						m = RigTForm(Cvec3(0, 0, dy) * g_arcballScale);
					}
					else if (g_mouseRClickButton && !g_mouseLClickButton) {
						// right button down?
						m = RigTForm(Cvec3(-dx, -dy, 0) * g_arcballScale);
					}
					affine = eyeRbt;
				}
				(*g_skyNode).setRbt(doMtoOwrtA(inv(m), eyeRbt, affine));
				g_eyeNode = g_skyNode;
			}
		}
		else {
			//When try to object
			//a^t = w^t*A = s^t*A_s, s^t = w^t*S (A is O_T*E_R ( O is our target object));
			//A_s = S^-1*A (S is parent obj of obj)
			RigTForm targetRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);//
			affine = makeMixedFrame(targetRbt, eyeRbt);//A 
			RigTForm affine_s;
			parentRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode, 1);//S
			affine_s = inv(parentRbt)*affine;
			(*g_currentPickedRbtNode).setRbt(doMtoOwrtA(m, (*g_currentPickedRbtNode).getRbt(), affine_s));//do L = doMtoOwrtA(M,L,As)
			//cout << "get sphere Rbt with changed" << endl;
			(*g_sphereRbt) = getPathAccumRbt(g_world, g_currentPickedRbtNode);
		}
		glutPostRedisplay(); // we always redraw if we changed the scene
	}

	g_mouseClickX = x;
	g_mouseClickY = g_windowHeight - y - 1;
}

//When mouse motion is haapen, this funciton is call
static void mouse(const int button, const int state, const int x, const int y) {
	int prev = g_mouseClickDown;
	int prev_L = g_mouseLClickButton;
	int prev_R = g_mouseRClickButton;
	int prev_M = g_mouseMClickButton;

	g_mouseClickX = x;
	g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

	g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
	g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
	g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

	g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
	g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
	g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);
	g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
	//sometimes, mouse function is not call when press both button and button up simultancely TODO
	//Draw arcball after leave hands when scaling (click right and left)
	if (((prev_L == 1 && prev_R == 1) && !(g_mouseLClickButton && g_mouseRClickButton)) || (prev_M && !g_mouseMClickButton))
	{
		if (g_arcball_act) {
			//cout << "get eyeRbt when scale the arcball Rbt does not change" << endl;
			RigTForm eyeRbt = getPathAccumRbt(g_world, g_eyeNode);
			double z = (inv(eyeRbt) * (*g_sphereRbt)).getTranslation()[2];//get z cordination
			if (z <= -CS175_EPS) {
				g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight);
			}
			glutPostRedisplay();
		}
	}

	//When picking mode, click left button picking something
	if (g_pressP && g_mouseLClickButton)
	{
		pick();
	}
}

//When press the key this function is call, key is what I press the button
static void keyboard(const unsigned char key, const int x, const int y) {
	list<vector<RigTForm>*>::iterator frame_it;
	
	switch (key) {
	case 27:
		exit(0);                                  // ESC
	case 'h':
		cout << " ============== H E L P ==============\n\n"
			<< "h\t\thelp menu\n"
			<< "s\t\tsave screenshot\n"
			<< "f\t\tToggle flat shading on/off.\n"
			<< "o\t\tCycle object to edit\n"
			<< "v\t\tCycle view\n"
			<< "drag left mouse to rotate\n" << endl;
		break;
	case 's':
		glFlush();
		writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
		break;

	case 'v':
		double z;
		switch (vision_obj)
		{
		case 0:
			vision_obj = 1;
			g_eyeNode = g_robot2Node;
			cout << "Active Eye is Robot2\n";
			break;
		case 1:
			vision_obj = 2;
			g_eyeNode = g_skyNode;
			cout << "Active Eye is Sky\n";
			break;
		case 2://when sky view to robot 1
			vision_obj = 0;
			g_eyeNode = g_robot1Node;
			cout << "Active Eye is Robot1\n";
			break;
		default:
			vision_obj = 2;
			break;
		}
		z = (inv(getPathAccumRbt(g_world,g_eyeNode)) * (*g_sphereRbt)).getTranslation()[2];//get z cordination
		if (z <= -CS175_EPS) {
			g_arcballScale = getScreenToEyeScale(z, g_frustFovY, g_windowHeight);
		}
		break;
	case 'm':
		g_worldview = !g_worldview;
		cout << "Editing skt eye w.r.t " << (g_worldview ? "world-sky frame" : "sky-sky frame") << endl;
		break;
	case 'p':
		g_pressP = !g_pressP;
		cout << "Picking mode is " << (g_pressP ? "on" : "off") << endl;
		break;

	case ' ':
		if(g_frames_index == -1)
			cout << "No key frame definded"<<endl;
		else {
			resetRbtFromFrame();
			printf("Loading current Key frame [%d] to scene graph\n", g_frames_index);
		}
		break;
	case 'u':
		if (dumpNodes.empty()) {
			dumpSgRbtNodes(g_world, dumpNodes);//Make pointer nodes
			printf("dump\n");
		}
		if (keyFrames.empty()) {
			g_frames_index = 0;
			printf("Create New Frame[0]\n");
		}
		frame_it = insertFrame();
		frame_it++;
		if (frame_it != keyFrames.end())//When make first frame, We can't erase anything
		{
			delete *frame_it;
			keyFrames.erase(frame_it);
		}
		break;
	case'<':
		if (g_frames_index > 0) {
			g_frames_index--;
			resetRbtFromFrame();
			printf("Stepped backward to frame [%d]\n",g_frames_index);
		}
		break;
	case'>':
		if (g_frames_index < keyFrames.size()-1) {
			g_frames_index++;
			resetRbtFromFrame();
			printf("Stepped forward to frame [%d]\n", g_frames_index);
		}
		break;
	case'd':
		if (keyFrames.empty()) {
			printf("Frame is now Empty\n");
			break;
		}
		delete_core();
		break;
	case'n':
		if (keyFrames.empty()) {
			dumpSgRbtNodes(g_world,dumpNodes);//Make pointer nodes
			printf("dump done\n");
		}
		g_frames_index++;
		insertFrame();
		break;
	case 'y':
		//Play Stop the animation
		if (keyFrames.size() < 4) {
			printf("Cannot play animation with less than 4 keyframes.\n");
			break;
		}
		printf("Playing animation...\n");
		g_animation = !g_animation;//when g_animation true, playing animation, if change to false, we stop it
		if(g_animation)
			glutTimerFunc(0, animateTimerCallback, 0);//start callback function immedietly, start time is zero
		break;
	case '+':
		/*Make the animation go faster, this is accomplished by having one fewer interpolated frame
			between each pair of keyframes*/
		if (g_msBetweenKeyFrames > 100) {
			g_msBetweenKeyFrames -= 100; 
			g_animateFramesPerSecond -= 1;
		}
		printf("%d ms between keyframes.\n", g_msBetweenKeyFrames);
		break;
	case '-':
		/*Make the animation go slower, this is accomplished by having one more interpolated frame
			between each pair of keyframes..*/
		if (g_msBetweenKeyFrames < 10000) {
			g_msBetweenKeyFrames += 100; 
			g_animateFramesPerSecond += 1;
		}
		printf("%d ms between keyframes.\n", g_msBetweenKeyFrames);
		break;
	case 'i':
		if (dumpNodes.empty()) {
			dumpSgRbtNodes(g_world, dumpNodes);//Make pointer nodes
			printf("dump\n");
		}
		read_core();
		break;
	case 'w':
		if (keyFrames.size() < 4) {
			printf("Cannot output animation with less than 4 keyframes.\n");
			break;
		}
		write_core();
		printf("output current key frames to output file\n");
	}
	
	glutPostRedisplay();
}
//Handling Done 
static void initGlutState(int argc, char * argv[]) {
	glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
	glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
	glutCreateWindow("Assignment 5");                       // title the window

	glutDisplayFunc(display);                               // display rendering callback
	glutReshapeFunc(reshape);                               // window reshape callback
	glutMotionFunc(motion);                                 // mouse movement callback
	glutMouseFunc(mouse);                                   // mouse click callback
	glutKeyboardFunc(keyboard);
}

static void initGLState() {
	glClearColor(128. / 255., 200. / 255., 255. / 255., 0.);
	glClearDepth(0.);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_GREATER);
	glReadBuffer(GL_BACK);
	if (!g_Gl2Compatible)
		glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
	// Create some prototype materials
	Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
	Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");
	Material specular("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader");

	// copy diffuse prototype and set red color
	g_redDiffuseMat.reset(new Material(diffuse));
	g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

	// copy diffuse prototype and set blue color
	g_blueDiffuseMat.reset(new Material(diffuse));
	g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

	// normal mapping material
	g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
	g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
	g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

	// copy solid prototype, and set to wireframed rendering
	g_arcballMat.reset(new Material(solid));
	g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
	g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// copy solid prototype, and set to color white
	g_lightMat.reset(new Material(solid));
	g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

	// copy specular prototype and set green color
	g_meshMat.reset(new Material(specular));
	g_meshMat->getUniforms().put("uColor", Cvec3f(0, 1, 0));

	// pick shader
	g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
	initGround();
	initCubes();
	initSphere();
	initMesh();
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material){
	const double ARM_LEN = 0.7,
		ARM_THICK = 0.25,
		TORSO_LEN = 1.5,
		TORSO_THICK = 0.25,
		TORSO_WIDTH = 1,
		LEG_LEN = 1,
		LEG_THICK = 0.25,
		HEAD_R = 0.75;
	const int NUM_JOINTS = 10,
		NUM_SHAPES = 10;

	struct JointDesc {
		int parent;
		float x, y, z;
	};

	JointDesc jointDesc[NUM_JOINTS] = {
		{ -1 }, // torso
		{ 0,  TORSO_WIDTH / 2, TORSO_LEN / 2, 0 }, // upper right arm
		{ 0,  -1* TORSO_WIDTH / 2, TORSO_LEN / 2, 0 }, // upper left arm
		{ 1,  ARM_LEN, 0, 0 }, // lower right arm
		{ 2,  -1*ARM_LEN, 0, 0 }, // lower left arm
		{ 0,  TORSO_WIDTH / 2,-1* TORSO_LEN / 2, 0 }, // upper right Leg
		{ 0,  -1*TORSO_WIDTH / 2,-1 * TORSO_LEN / 2, 0 }, // upper left Leg
		{ 5,  0,-1 * LEG_LEN, 0 }, // lower right Leg
		{ 6,  0,-1 * LEG_LEN, 0 }, // lower left Leg
		{ 0,  0,TORSO_LEN/2 + 0.25, 0 } // head

	};

	struct ShapeDesc {
		int parentJointId;
		float x, y, z, sx, sy, sz;
		shared_ptr<Geometry> geometry;
	};

	ShapeDesc shapeDesc[NUM_SHAPES] = {
		{ 0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube }, // torso
		{ 1, ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere }, // upper right arm
		{ 2, -1*ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere }, // upper left arm
		{ 3, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube }, // lower right arm
		{ 4, -1*ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube }, // lower left arm
		{ 5, 0, -1*LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphere }, // upper right leg
		{ 6, 0, -1 * LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphere }, // upper left leg
		{ 7, 0, -1 * LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube }, // lower right leg
		{ 8, 0, -1 * LEG_LEN / 2, 0, LEG_THICK , LEG_LEN , LEG_THICK, g_cube }, // lower right leg
		{ 9, 0, HEAD_R/2, 0, HEAD_R/2 , HEAD_R/2 , HEAD_R/2, g_sphere }, // head
	};

	shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

	for (int i = 0; i < NUM_JOINTS; ++i) {
		if (jointDesc[i].parent == -1)
			jointNodes[i] = base;
		else {
			jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
			jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
		}
	}
	// The new MyShapeNode takes in a material as opposed to color
	for (int i = 0; i < NUM_SHAPES; ++i) {
		shared_ptr<SgGeometryShapeNode> shape(
			new MyShapeNode(shapeDesc[i].geometry,
				material, // USE MATERIAL as opposed to color
				Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
				Cvec3(0, 0, 0),
				Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
		jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
	}
}

static void constructLight(shared_ptr<SgTransformNode> base) {
	shared_ptr<SgTransformNode> jointNode = base;
	struct ShapeDesc {
		float x, y, z, sx, sy, sz;
		shared_ptr<Geometry> geometry;
	};

	const double LIGHT_R = 1;
	ShapeDesc shapeDesc = { 0, 0, 0, LIGHT_R / 2 , LIGHT_R / 2 , LIGHT_R / 2, g_sphere }; // light
	shared_ptr<SgGeometryShapeNode> shape(
		new MyShapeNode(shapeDesc.geometry,
			g_lightMat, // USE MATERIAL as opposed to color
			Cvec3(shapeDesc.x, shapeDesc.y, shapeDesc.z),
			Cvec3(0, 0, 0),
			Cvec3(shapeDesc.sx, shapeDesc.sy, shapeDesc.sz)));
	jointNode->addChild(shape);
}

static void constructMesh() {
	shared_ptr<SgTransformNode> jointNode = g_meshNode;
	struct ShapeDesc {
		float x, y, z, sx, sy, sz;
		shared_ptr<Geometry> geometry;
	};

	const double edgeLen = 1;
	ShapeDesc shapeDesc = { 0, 0, 0,edgeLen , edgeLen , edgeLen, g_mesh }; // Mesh
	shared_ptr<SgGeometryShapeNode> shape(
		new MyShapeNode(shapeDesc.geometry,
			g_meshMat, // USE MATERIAL as opposed to color
			Cvec3(shapeDesc.x, shapeDesc.y, shapeDesc.z),
			Cvec3(0, 0, 0),
			Cvec3(shapeDesc.sx, shapeDesc.sy, shapeDesc.sz)));
	jointNode->addChild(shape);
}

static void initScene() {
	g_world.reset(new SgRootNode());//make g_world

	g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));
	g_eyeNode = g_skyNode;
	
	g_groundNode.reset(new SgRbtNode());
	g_currentPickedRbtNode = g_groundNode;
	g_groundNode->addChild(shared_ptr<MyShapeNode>(
		new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

	g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
	g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

	constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
	constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

	g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(4.0, 2.0, 2.0))));
	g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(-3.0, 4.0, -3.0))));
	
	constructLight(g_light1Node);
	constructLight(g_light2Node);

	g_meshNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 0, 0))));
	constructMesh();

	g_world->addChild(g_skyNode);
	g_world->addChild(g_groundNode);
	g_world->addChild(g_robot1Node);
	g_world->addChild(g_robot2Node);
	g_world->addChild(g_light1Node);
	g_world->addChild(g_light2Node);
	g_world->addChild(g_meshNode);
}



int main(int argc, char * argv[]) {
	try {
		initGlutState(argc, argv);

		glewInit(); // load the OpenGL extensions

		cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
		if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
		else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

		initGLState();
		initMaterials();
		initGeometry();
		initScene();

		glutMainLoop();
		return 0;
	}
	catch (const runtime_error& e) {
		cout << "Exception caught: " << e.what() << endl;
		return -1;
	}
}
