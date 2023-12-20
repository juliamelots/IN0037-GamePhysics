#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include<ctime>
using namespace std;

DiffusionSimulator::DiffusionSimulator()
{
	cout << "init start" << endl;
	m_iTestCase = 0;
	// TO-DO initialize with minimum values
	m_fAlpha = 0.01;
	m_fDeltaSpace = 0.01;
	m_iX = 16;
	m_iY = 16;
	m_iZ = 1;
	cout << "init end" << endl;
}

const char * DiffusionSimulator::getTestCasesStr()
{
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	cout << "init UI start" << endl;
	this->DUC = DUC;
	// TO-DO what should be the minimum values?
	// TO-DO fix error in change of dimensions
	TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_INT32, &m_iX, "min=16");
	TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_INT32, &m_iY, "min=16");
	if (m_iTestCase > 1) // 3D implementations
		TwAddVarRW(DUC->g_pTweakBar, "P", TW_TYPE_INT32, &m_iZ, "min=16");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coeficient", TW_TYPE_FLOAT, &m_fAlpha, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "SpaceStep", TW_TYPE_FLOAT, &m_fDeltaSpace, "min=0.001 step=0.001");
	cout << "init UI end" << endl;
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	setupT();
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep)
{
	cout << "init exp start" << endl;
	std::vector<Real> newT;
	float factor = m_fAlpha * timeStep / (m_fDeltaSpace * m_fDeltaSpace);
	// make sure that the temperature in boundary cells stays zero
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
		{
			if (isBoundary(i, j))
				newT.push_back(0.0);
			else
				newT.push_back
				(
					(1 + 4 * factor) * T.at(idx(i, j))
					+ factor * (T.at(idx(i+1, j)) + T.at(idx(i-1 ,j))
						+ T.at(idx(i, j+1)) + T.at(idx(i, j-1)))
				);
		}
	fillT(newT);
	cout << "init exp end" << endl;
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
				A.set_element(idx(i, j), idx(i, j), 1 + 4*factor);
				A.set_element(idx(i, j), idx(i+1, j), -factor);
				A.set_element(idx(i, j), idx(i-1, j), -factor);
				A.set_element(idx(i, j), idx(i, j+1), -factor);
				A.set_element(idx(i, j), idx(i, j-1), -factor);
			}
		}
	cout << "setA end" << endl;
}

//--------------------------------------------------------------------------------------
// Solve AT=b
//--------------------------------------------------------------------------------------
void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep)
{
	const int N = m_iX * m_iY;
	float factor = m_fAlpha * timeStep / (m_fDeltaSpace * m_fDeltaSpace);
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N); // TO-DO add expected zeros per row
	std::vector<Real> *b = new std::vector<Real> (N);

	setupA(*A, factor);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);
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
	Vec3 color;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
		{
			// red for negative values and white for positive ones
			if (isBoundary(i, j))
				color = Vec3(0, 0, 0);
			else
				color = Vec3(getNormalValue(i, j), T.at(idx(i, j)) > 0, T.at(idx(i, j)) > 0);
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
			DUC->drawSphere(Vec3(i, j, 0), 0.5 * Vec3(1, 1, 1));
		}
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
