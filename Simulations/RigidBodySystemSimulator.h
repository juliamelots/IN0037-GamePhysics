#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#include <vector>
#define TESTCASEUSEDTORUNTEST 2
#define FLOOR -1
#define RADIUS 0.05f

class RigidBody {
public:
	RigidBody() {
		RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), 1);
	};

	RigidBody(int inf) // Ground constructor
	{
		m_oldPosition = m_position = Vec3(0.0, -51.0, 0.0);
		m_mass = static_cast<float>(inf);
		m_isCollision = false;
		m_rotation = Quat(0.0, 0.0, 0.0);
		m_internalForce = Vec3();
		m_linearVelocity = Vec3();
		m_angularVelocity = Vec3();
		m_angularMomentum = Vec3();
		m_torqueExternalForce = Vec3();
		Mat4 initialIntertiaTensor = Mat4(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1) * inf;
		m_initialInverseIntertiaTensor = initialIntertiaTensor.inverse();
		m_scaleMatrix.initScaling(inf, 100, inf);
		m_translationMatrix.initTranslation(m_position.x, m_position.y, m_position.z);
	};

	RigidBody(Vec3 position, Vec3 size, int mass)
	{
		m_mass = static_cast<float>(mass);
		m_isCollision = false;
		m_oldPosition = m_position = position;
		m_rotation = Quat(0.0, 0.0, 0.0);
		m_internalForce = Vec3();
		m_linearVelocity = Vec3();
		m_angularVelocity = Vec3();
		m_angularMomentum = Vec3();
		m_torqueExternalForce = Vec3();
		Mat4 initialIntertiaTensor = Mat4(
			(size.y * size.y + size.z * size.z), 0, 0, 0,
			0, (size.x * size.x + size.z * size.z), 0, 0,
			0, 0, (size.x * size.x + size.y * size.y), 0,
			0, 0, 0, 1) * m_mass / 12.0f;
		m_initialInverseIntertiaTensor = initialIntertiaTensor.inverse();
		m_scaleMatrix.initScaling(size.x, size.y, size.z);
		m_translationMatrix.initTranslation(m_position.x, m_position.y, m_position.z);
	}

	Vec3 getVelocityOfPosition(Vec3 point);
	Vec3 getWorldPositionOfPoint(Vec3 point);
	Mat4 getInverseIntertiaTensor();
	Mat4 getlocalToWorldMat();

	float m_mass;
	bool m_isCollision;
	Vec3 m_position;
	Vec3 m_oldPosition;
	Quat m_rotation;
	Vec3 m_internalForce;
	Vec3 m_linearVelocity;
	Vec3 m_angularVelocity;
	Vec3 m_angularMomentum;
	Vec3 m_torqueExternalForce;
	Mat4 m_initialInverseIntertiaTensor;
	Mat4 m_scaleMatrix;
	Mat4 m_translationMatrix;
};

class Spring {
public:
	// Constructors
	Spring(int bodyIdx1, int bodyIdx2, float initialLength, float stiffness)
		: bodyIdx1(bodyIdx1), bodyIdx2(bodyIdx2), m_fInitialLength(initialLength), m_fStiffness(stiffness) {}

	// Data Attributes
	int bodyIdx1;
	int bodyIdx2;
	float m_fStiffness;
	float m_fInitialLength;
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

	// Mass spring related functions
	void addSpring(int indexPoint1, int indexPoint2, float initialLength, float stiffness);
	int getNumberOfSprings();
	void removeSprings();
	void addRandomSprings(int number);
	void setDampingFactor(float damping);
	float getCurrentLength(Spring spring);
	void applyInternalForce(Spring spring);

	// Rigid Body related functions
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	int getNumberOfRigidBodies();
	void removeRigidBodies();
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);

private:
	// Attributes
	Vec3 m_externalForce;
	Vec3 m_externalForcePosition;
	vector<RigidBody> m_rigidBodies;
	vector<Spring> m_vSprings;
	float m_fCoefRestitution;
	float m_fGravity;
	float m_fDamping;
	RigidBody m_ground = RigidBody(1000000000000);
	int m_iWebSize;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif