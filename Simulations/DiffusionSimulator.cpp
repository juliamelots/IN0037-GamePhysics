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
	if (m_iNewX != m_iX || m_iNewY != m_iY || m_iNewZ != m_iZ)
	{
		m_iX = m_iNewX;
		m_iY = m_iNewY;
		m_iZ = m_iNewZ;
	}
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
// Set up T as vector of size M*N (emulates MxN matrix)
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupT()
{
	T.clear();
	cout << "setT start" << endl;
	std::mt19937 eng;
	std::uniform_real_distribution<float> randVal(-50.0, 50.0);
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
		{
			if (isBoundary(i, j))
				T.push_back(0.0);
			else
			{
				float new_value = randVal(eng);
				cout << new_value << endl;
				if (new_value > m_fMaxValue) m_fMaxValue = new_value;
				if (new_value < m_fMinValue) m_fMinValue = new_value;
				cout << m_fMaxValue << m_fMinValue << endl;
				T.push_back(new_value);
			}
		}
	cout << "setT end" << endl;
}

//--------------------------------------------------------------------------------------
// Fill matrix T with T^(n+1) values from solved vector x of size M*N
//--------------------------------------------------------------------------------------
void DiffusionSimulator::fillT(std::vector<Real> x)
{
	// make sure that the temperature in boundary cells stays zero
	cout << "fillT start" << endl;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
		{
			float new_value = x.at(idx(i, j));
			if (new_value > m_fMaxValue) m_fMaxValue = new_value;
			if (new_value < m_fMinValue) m_fMinValue = new_value;
			T.at(idx(i, j)) = new_value;
		}
	cout << "fillT end" << endl;
}

//--------------------------------------------------------------------------------------
// Set up vector of size M*N (emulates MxN matrix) with current T values
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupB(std::vector<Real>& b)
{
	cout << "setB start" << endl;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			b.push_back(T.at(idx(i, j)));
	cout << "setB end" << endl;
}

//--------------------------------------------------------------------------------------
// Set up sparse matrix of size (M*N)x(M*N)
// (emulates (MxN)x(MxN) matrix) with linear system's coefficients
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupA(SparseMatrix<Real>& A, double factor)
{
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	cout << "setA start" << endl;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
		{
			if (isBoundary(i, j))
				A.set_element(idx(i, j), idx(i, j), 1 + 4 * factor);
			else
			{
				// A.set_element(linear access of element in T^n,
				//  linear access of element in T^(n+1));
				A.set_element(idx(i, j), idx(i, j), 1 + 4 * factor);
				A.set_element(idx(i, j), idx(i + 1, j), -factor);
				A.set_element(idx(i, j), idx(i - 1, j), -factor);
				A.set_element(idx(i, j), idx(i, j + 1), -factor);
				A.set_element(idx(i, j), idx(i, j - 1), -factor);
			}
		}
	cout << "setA end" << endl;
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