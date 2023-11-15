#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class MassPoint {
public:
	// Constructors
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed)
	{
		m_position = m_OldPosition = position;
		m_velocity = m_OldVelocity = velocity;
		m_internalForce = m_OldInternalForce = Vec3();
		m_bIsFixed = isFixed;
	};

	// Data Attributes
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_internalForce;
	Vec3 m_OldPosition;
	Vec3 m_OldVelocity;
	Vec3 m_OldInternalForce;
	bool m_bIsFixed;

};

class Spring {
public:
	// Constructors
	Spring(MassPoint* point1, MassPoint* point2, float initialLength)
	{
		m_point1 = point1;
		m_point2 = point2;
		m_fInitialLength = initialLength;
	};

	// Data Attributes
	MassPoint* m_point1;
	MassPoint* m_point2;
	float m_fInitialLength;

	// Specific Functions
	float getCurrentLength() { return norm(m_point1->m_position - m_point2->m_position); };
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	void addSpring(int indexPoint1, int indexPoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void checkCollision();

	// Additional Functions
	void applyInternalForce(Spring* spring);
	Vec3 calculateNewPosition(Vec3 position, Vec3 velocity, float timeStep);
	Vec3 calculateNewVelocity(Vec3 velocity, Vec3 internalForce, float timeStep);

	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	const float m_fRadius = 0.05f;
	vector<MassPoint*> m_vMassPoints;
	vector<Spring*> m_vSprings;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif