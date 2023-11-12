#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "MassSpringComponents.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

#define STD_FIXED 0
#define STD_SPHERE_SIZE 0.05

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
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);

	// Simulation Functions
	void applyExternalForce(Vec3 force);
	void applyInternalForce(Spring* spring);
	Vec3 applyInternalForce(Vec3 position, Vec3 velocity, vector<int> attachedSprings);
	Vec3 integratePosition(Vec3 position, Vec3 velocity, float timeStep);
	Vec3 integrateVelocity(Vec3 velocity, Vec3 internalForce, float timeStep);

	// Set Up Functions
	void setUp2PointSystem();
	
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

	// Control Attributes
	vector<MassPoint*> m_massPoints;
	vector<Spring*> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif