#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include<ctime>
using namespace std;

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	// TO-DO initialize with minimum values
	m_fAlpha = 0.01;
	m_fDeltaSpace = 0.01;
	m_iX = 16;
	m_iY = 16;
	m_iZ = 1;
}

const char* DiffusionSimulator::getTestCasesStr()
{
	return "2D Explicit_solver,2D Implicit_solver,3D Explicit_solver,3D Implicit_solver";
}

void DiffusionSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	// TO-DO what should be the minimum values?
	// dimension values used in real time, can't directly alter them
	TwAddVarRW(DUC->g_pTweakBar, "X", TW_TYPE_INT32, &m_iNewX, "min=16");
	TwAddVarRW(DUC->g_pTweakBar, "Y", TW_TYPE_INT32, &m_iNewY, "min=16");
	if (m_iTestCase > 1) // 3D implementation
		TwAddVarRW(DUC->g_pTweakBar, "Z", TW_TYPE_INT32, &m_iNewZ, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coeficient", TW_TYPE_FLOAT, &m_fAlpha, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "SpaceStep", TW_TYPE_FLOAT, &m_fDeltaSpace, "min=0.001 step=0.001");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// dimensions altered only if simulation was reset or case was changed
	if (m_iNewX != m_iX || m_iNewY != m_iY || m_iNewZ != m_iZ)
	{
		m_iX = m_iNewX;
		m_iY = m_iNewY;
		m_iZ = m_iNewZ;
	}
	// 3D checks used in real time, altered only if case was changed
	m_b3D = (m_iTestCase > 1);
	setupT();
	switch (m_iTestCase)
	{
	case 0:
		cout << "2D Explicit solver!\n";
		break;
	case 1:
		cout << "2D Implicit solver!\n";
		break;
	case 2:
		cout << "3D Explicit solver!\n";
		break;
	case 3:
		cout << "3D Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep)
{
}

//--------------------------------------------------------------------------------------
// Set up T as vector of size X*Y*Z (emulates XxYxZ matrix)
// with zero in boundary cells and random numbers in others
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupT()
{
	cout << "setT start" << endl;
	T.clear();
	std::mt19937 eng;
	std::uniform_real_distribution<float> randVal(-50.0, 50.0);
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			for (int k = 0; k < m_iZ; k++)
			{
				if (isBoundary(i, j, k))
					T.push_back(0.0);
				else
				{
					float new_value = randVal(eng);
					if (new_value > m_fMaxValue) m_fMaxValue = new_value;
					if (new_value < m_fMinValue) m_fMinValue = new_value;
					T.push_back(new_value);
				}
			}
	cout << "setT end" << endl;
}

//--------------------------------------------------------------------------------------
// Fill vector T with T^(n+1) values from solved vector x of size X*Y*Z
//--------------------------------------------------------------------------------------
void DiffusionSimulator::fillT(std::vector<Real> x)
{
	cout << "fillT start" << endl;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			for (int k = 0; k < m_iZ; k++)
			{
				// temperature in boundary cells stays zero
				if (isBoundary(i, j, k))
					T.at(idx(i, j, k)) = 0.0;
				else
				{
					float new_value = x.at(idx(i, j, k));
					if (new_value > m_fMaxValue) m_fMaxValue = new_value;
					if (new_value < m_fMinValue) m_fMinValue = new_value;
					T.at(idx(i, j, k)) = new_value;
				}
			}
	cout << "fillT end" << endl;
}

//--------------------------------------------------------------------------------------
// Set up vector of size X*Y*Z (emulates XxYxZ matrix) with T^n values
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupB(std::vector<Real>& b)
{
	cout << "setB start" << endl;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			for (int k = 0; k < m_iZ; k++)
				b.push_back(T.at(idx(i, j, k)));
	cout << "setB end" << endl;
}

//--------------------------------------------------------------------------------------
// Set up sparse matrix of size (X*Y*Z)x(X*Y*Z)
// (emulates (XxYxZ)x(XxYxZ) matrix) with linear system's coefficients
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupA(SparseMatrix<Real>& A, double factor)
{
	int center;
	if (m_b3D)
	{
		// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
		cout << "setA 3D start" << endl;
		for (int i = 0; i < m_iX; i++)
			for (int j = 0; j < m_iY; j++)
				for (int k = 0; k < m_iZ; k++)
				{
					center = idx(i, j, k);
					if (isBoundary(i, j, k))
						A.set_element(center, center, 1.0);
					else
					{
						// A.set_element(linear access of element in T^n,
						//  linear access of element in T^(n+1));
						// TO-DO check if A elements are correct
						A.set_element(center, center, 1 + 6 * factor); // TO-DO multiply by 6?
						A.set_element(center, idx(i + 1, j, k), -factor);
						A.set_element(center, idx(i - 1, j, k), -factor);
						A.set_element(center, idx(i, j + 1, k), -factor);
						A.set_element(center, idx(i, j - 1, k), -factor);
						A.set_element(center, idx(i, j, k + 1), -factor);
						A.set_element(center, idx(i, j, k - 1), -factor);
					}
				}
		cout << "setA 3D end" << endl;
	}
	else
	{
		// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
		cout << "setA 2D start" << endl;
		for (int i = 0; i < m_iX; i++)
			for (int j = 0; j < m_iY; j++)
			{
				center = idx(i, j);
				if (isBoundary(i, j))
					A.set_element(center, center, 1.0);
				else
				{
					// A.set_element(linear access of element in T^n,
					//  linear access of element in T^(n+1));
					A.set_element(center, center, 1 + 4 * factor);
					A.set_element(center, idx(i + 1, j), -factor);
					A.set_element(center, idx(i - 1, j), -factor);
					A.set_element(center, idx(i, j + 1), -factor);
					A.set_element(center, idx(i, j - 1), -factor);
				}
			}
		cout << "setA 2D end" << endl;
	}
}

//--------------------------------------------------------------------------------------
// Solve AT=b
//--------------------------------------------------------------------------------------
void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep)
{
}

//--------------------------------------------------------------------------------------
// Update current T^n state to T^(n+1) for each frame
//--------------------------------------------------------------------------------------
void DiffusionSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}