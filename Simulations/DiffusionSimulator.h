#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int rows, int columns);
	void setupB(std::vector<Real>& b);
	void setupA(SparseMatrix<Real>& A, double factor);

	float at(int i, int j) const
	{ return values.at(i * m + j); };

	void set(int i, int j, float new_value)
	{ values.at(i * m + j) = new_value; };

private:
	int m
	int n;
	std::vector<float> values;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid & T; //save results of every time step
	int m_iM;
	int m_iN;
	float m_fAlpha;
	float m_fDeltaSpace;
};

#endif