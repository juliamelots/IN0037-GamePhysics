#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#define TESTCASEUSEDTORUNTEST 2


class RigidBody {
public:
	float m_mass;
	Vec3 m_position;
	Quat m_rotation;
	Vec3 m_velocity;
	Vec3 m_angularVelocity;
	Vec3 m_angularMomentum;
	GamePhysics::Mat4d m_initialInverseIntertiaTensor;
	GamePhysics::Mat4d m_scaleMatrix;
	GamePhysics::Mat4d m_translationMatrix;

public:
	RigidBody();

	RigidBody(Vec3 position, Vec3 size, int mass) {
		m_position = position;
		m_mass = static_cast<float>(mass);
		m_scaleMatrix.initScaling(size.x, size.y, size.z);
		m_translationMatrix.initTranslation(m_position.x, m_position.y, m_position.z);
		GamePhysics::Mat4d initialIntertiaTensor = GamePhysics::Mat4(
			(size.y * size.y + size.z * size.z), 0, 0, 0,
			0, (size.x * size.x + size.z * size.z), 0, 0,
			0, 0, (size.x * size.x + size.y * size.y), 0,
			0, 0, 0, 1) * m_mass / 12.0f;
		m_initialInverseIntertiaTensor = initialIntertiaTensor.inverse();
	}

	Vec3 getVelocityOfPosition(Vec3 point);
	Vec3 getWorldPositionOfPoint(Vec3 point);
	Mat4 getInverseIntertiaTensor();
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

	// ExtraFunctions
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);

	void removeRigidBodies();

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;
	Vec3 m_externalForcePosition;
	vector<RigidBody> m_rigidBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif