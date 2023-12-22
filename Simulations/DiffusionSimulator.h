#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"

class DiffusionSimulator :public Simulator {
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific functions
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

	// Grid functions
	void setupT();
	void setupB(std::vector<Real>& b);
	void setupA(SparseMatrix<Real>& A,const double& factor);
	void fillT(const std::vector<Real>& x);

	bool isBoundary(int i, int j, int k = 0) const
	{
		return (i == 0 || i == m_iX - 1 || j == 0 || j == m_iY - 1)
				|| (m_b3D && (k == 0 || k == m_iZ - 1));
	};

	//--------------------------------------------------------------------------------------
	// Calculates universal index to access vector of size X*Y*Z
	// when given three dimension-specific indexes
	//--------------------------------------------------------------------------------------
	float idx(int i, int j, int k = 0) const
	{
		if (m_b3D) return i * m_iY * m_iZ + j * m_iZ + k;
		return i * m_iY + j;
		
	};

	//--------------------------------------------------------------------------------------
	// Calculates tuple of three dimension-specific indexes
	// to access vector of size X*Y*Z when given universal index
	//--------------------------------------------------------------------------------------
	std::tuple<int, int, int> idx(int l) const
	{
		if (m_b3D) return std::make_tuple(l / (m_iX * m_iY), l % (m_iX * m_iY) / m_iY, l % (m_iX * m_iY) % m_iY);
		return std::make_tuple(l / m_iY, l % m_iY, 0);
	}

	float getNormalValue(int i, int j, int k = 0, float min_val = 0, float max_val = 1) const
	{
		float t{ T.at(idx(i, j, k)) };
		float f;
		if (t > 0)
			f =  (t - 0) / (m_fMaxValue);
		else
			f = (-t) / (-m_fMinValue);
		return f * (max_val - min_val) + min_val;
	};

private:
	// UI attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Simulator attributes
	bool m_b3D;
	float m_fAlpha;
	float m_fDeltaSpace;

	// Grid attributes
	int m_iNewX; int m_iX;
	int m_iNewY; int m_iY;
	int m_iNewZ; int m_iZ;
	float m_fMaxValue;
	float m_fMinValue;
	std::vector<float> T;
};

#endif