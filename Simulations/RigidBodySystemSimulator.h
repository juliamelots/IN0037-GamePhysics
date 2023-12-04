#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "util/matrixbase.h"
#include "collisionDetect.h"
#include <ctime>

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class Rigidbody {
public:
	float m_mass;
	Vec3 m_position;
	Quat m_rotation;
	Vec3 m_velocity;
	Vec3 m_angularVelocity;
	Vec3 m_angularMomentum;
	GamePhysics::Mat4d m_initialIntertiaTensor;

	Vec3 m_size;

public:
	Rigidbody();

	Rigidbody(Vec3 position, Vec3 size, int mass) {
		m_position = position;
		m_mass = static_cast<float>(mass);
		m_size = size;
		double massToDozen = m_mass / 12.0f;
		m_initialIntertiaTensor = GamePhysics::Mat4( (size.y * size.y + size.z * size.z) ,0, 0, 0,
			0, (size.x * size.x + size.z * size.z), 0, 0,
			0, 0, (size.x * size.x + size.y * size.y),0,
			0, 0, 0, 1) * massToDozen;
		m_initialIntertiaTensor = m_initialIntertiaTensor.inverse();

		//TODO: calculate m_initialIntertiaTensorFrom size
	}


	Vec3 getVelocityOfPosition(Vec3 point, bool positionInObjectSpace);
	Vec3 getVelocityOfPosition(Vec3 point);
	Vec3 getPositionAfterRotation(Vec3 initialPos);
	Vec3 getWorldPositionOfPoint(Vec3 point);
	Mat4 getIntertiaTensor();
	Mat4 getlocalToWorldMat();
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void calculateTorque(float timeStep);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void removeRigidbodies();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	std::vector<Rigidbody> m_rigidbodies;
	Vec3 m_externalForcePosition;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif