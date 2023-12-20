#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

//--------------------------------------------------------------------------------------
// Initialize MxN grid with border/boundary cells
//--------------------------------------------------------------------------------------
Grid::Grid(int rows, int columns)
{
	m = rows;
	n = columns;
	std::vector<float> values(m * n, 0.0);
}

DiffusionSimulator::DiffusionSimulator()
: T(Grid(16, 16))
{
	m_iTestCase = 0;
	// TO-DO initialize with minimum values
	m_iM = m_iN = 16;
	m_fAlpha = 0.0;
	m_fDeltaSpace = 0.0;
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
	this->DUC = DUC;
	// TO-DO what should be the minimum values?
	TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_INT32, &m_iM, "min=16 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_INT32, &m_iN, "min=16 step=1");
	// if (m_iTestCase > 1) // 3D implementations
	// 	TwAddVarRW(DUC->g_pTweakBar, "P", TW_TYPE_INT32, &m_iP, "min=16 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coeficient", TW_TYPE_FLOAT, &m_fAlpha, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "SpaceStep", TW_TYPE_FLOAT, &m_fDeltaSpace, "min=0.001 step=0.001");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	T = Grid(m_iM, m_iN);
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

Grid & DiffusionSimulator::diffuseTemperatureExplicit(float timeStep)
{
	Grid & newT = Grid(m_iM, m_iN);
	float factor = m_fAlpha * timeStep / (m_fDeltaSpace * m_fDeltaSpace);
	// make sure that the temperature in boundary cells stays zero
	// -> limit for loops to non-boundary cells
	for (int i = 1; i < m_iM-1; i++)
		for (int j = 1; j < m_iN-1; j++)
			newT.set(i, j,
				(1 + 4 * factor) * T.value(i, j) +
				factor * (T.value(i+1, j) + T.value(i-1, j) + T.value(i, j+1) + T.value(i, j-1)));
	return newT;
}

//--------------------------------------------------------------------------------------
// Fill matrix T with T^(n+1) values from solved vector x of size M*N
//--------------------------------------------------------------------------------------
void Grid::fillT(std::vector<Real> x)
{
	// make sure that the temperature in boundary cells stays zero
	// -> limit for loops to non-boundary cells
	for (int i = 1; i < m-1; i++)
		for (int j = 1; j < n-1; j++)
			set(i, j, x.at(i*m + j));
}

//--------------------------------------------------------------------------------------
// Set up vector of size M*N (emulates MxN matrix) with current T values
//--------------------------------------------------------------------------------------
void Grid::setupB(std::vector<Real>& b)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			b.at(i*m + j) = value(i, j);
}

//--------------------------------------------------------------------------------------
// Set up sparse matrix of size (M*N)x(M*N)
// (emulates (MxN)x(MxN) matrix) with linear system's coefficients
//--------------------------------------------------------------------------------------
void Grid::setupA(SparseMatrix<Real>& A, double factor)
{
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
		{
			if (isBoudary(i, j))
				A.set_element(i*m + j, i*m + j, 1 + 4*factor);
			else
			{
				// A.set_element(linear access of element in T^n,
				//  linear access of element in T^(n+1));
				A.set_element(i*m + j, i*m + j, 1 + 4*factor);
				A.set_element(i*m + j, (i+1)*m + j, -factor);
				A.set_element(i*m + j, (i-1)*m + j, -factor);
				A.set_element(i*m + j, i*m + (j+1), -factor);
				A.set_element(i*m + j, i*m + (j-1), -factor);
			}
		}
}

//--------------------------------------------------------------------------------------
// Solve AT=b
//--------------------------------------------------------------------------------------
void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep)
{
	const int N = m_iM * m_iN;
	float factor = m_fAlpha * timeStep / (m_fDeltaSpace * m_fDeltaSpace);
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N); // TO-DO add expected zeros per row
	std::vector<Real> *b = new std::vector<Real>(N, 0.0);

	T.setupA(*A, factor);
	T.setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N, 0.0);

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	T.fillT(x);
}

//--------------------------------------------------------------------------------------
// Update current T^n state to T^(n+1) for each frame
//--------------------------------------------------------------------------------------
void DiffusionSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i < m_iM; i++)
		for (int j = 0; j < m_iN; j++)
		{
			// red for negative values and white for positive ones
			//float temperature = T.value(i, j);
			//Vec3 color = Vec3(0.0, 0.0, 0.0);
			//DUC->setUpLighting(color, color, 0.0, color);
			//DUC->drawSphere(Vec3(i, j, 0), 0.5 * Vec3(1, 1, 1));
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
