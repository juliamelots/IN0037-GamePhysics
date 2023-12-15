#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

class Grid {
public:
	Grid();
	Grid(int rows, int columns);

	void setupB(std::vector<Real>& b);
	void setupA(SparseMatrix<Real>& A, double factor);
	void fillT(std::vector<Real> x);

	float at(int i, int j) const
	{ return values.at(i * m + j); };

	void set(int i, int j, float new_value)
	{ values.at(i * m + j) = new_value; };

private:
	int m;
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
	Grid & diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

private:
	// Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	int m_iM;
	int m_iN;
	float m_fAlpha;
	float m_fDeltaSpace;
	Grid & T; //save results of every time step
};

#endif