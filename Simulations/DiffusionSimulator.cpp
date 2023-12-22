#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include<ctime>
using namespace std;

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_fAlpha = 0.01;
	m_fDeltaSpace = 0.1;
	m_iX = m_iNewX = 16;
	m_iY = m_iNewY = 16;
	m_iZ = m_iNewZ = 1;
	m_fMaxValue = m_fMinValue = 0.0;
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
	// dimension values used in real time, can't directly alter them
	TwAddVarRW(DUC->g_pTweakBar, "X", TW_TYPE_INT32, &m_iNewX, "min=16");
	TwAddVarRW(DUC->g_pTweakBar, "Y", TW_TYPE_INT32, &m_iNewY, "min=16");
	if (m_iTestCase > 1) // 3D implementation
		TwAddVarRW(DUC->g_pTweakBar, "Z", TW_TYPE_INT32, &m_iNewZ, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coeficient", TW_TYPE_FLOAT, &m_fAlpha, "min=0.001 step=0.001");
	TwAddVarRW(DUC->g_pTweakBar, "SpaceStep", TW_TYPE_FLOAT, &m_fDeltaSpace, "min=0.001 step=0.001");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "2D Explicit solver!\n";
		m_iNewZ = m_iZ = 1;
		break;
	case 1:
		cout << "2D Implicit solver!\n";
		m_iNewZ = m_iZ = 1;
		break;
	case 2:
		cout << "3D Explicit solver!\n";
		if (!m_b3D) m_iZ = m_iNewZ = 16; // just changed from 2D to 3D, make 3rd dimension visible
		break;
	case 3:
		cout << "3D Implicit solver!\n";
		if (!m_b3D) m_iZ = m_iNewZ = 16; // just changed from 2D to 3D, make 3rd dimension visible
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
	// 3D checks used in real time, altered only if case was changed
	m_b3D = (m_iTestCase > 1);
	// dimensions altered only if simulation was reset or case was changed
	if (m_iNewX != m_iX || m_iNewY != m_iY || m_iNewZ != m_iZ)
	{
		m_iX = m_iNewX;
		m_iY = m_iNewY;
		m_iZ = m_iNewZ;
	}
	setupT();
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep)
{
	int i, j, k;
	std::vector<Real> new_T;
	float factor = (m_fAlpha * timeStep) / (m_fDeltaSpace * m_fDeltaSpace);
	for (int center = 0; center < T.size(); center++) {
		std::tie(i, j, k) = idx(center);
		if (isBoundary(i, j, k))
			new_T.push_back(0.0);
		else
		{
			float new_value = T.at(center) + factor *
				(T.at(idx(i + 1, j, k)) + T.at(idx(i - 1, j, k))
				+ T.at(idx(i, j + 1, k)) + T.at(idx(i, j - 1, k))
				- 4.0 * T.at(center));
			if (m_b3D)
				new_value += factor * (T.at(idx(i, j, k+1)) + T.at(idx(i, j, k - 1))
					- 2.0 * T.at(center));
			new_T.push_back(new_value);
		}
	}
	fillT(new_T);
}

//--------------------------------------------------------------------------------------
// Set up T as vector of size X*Y*Z (emulates XxYxZ matrix)
// with zero in boundary cells and random numbers in others
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupT()
{
	T.clear();
	m_fMaxValue = m_fMinValue = 0.0;
	std::mt19937 eng(time(nullptr));
	std::uniform_real_distribution<float> randVal(-70, 70.0);
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
}

//--------------------------------------------------------------------------------------
// Fill vector T with T^(n+1) values from solved vector x of size X*Y*Z
//--------------------------------------------------------------------------------------
void DiffusionSimulator::fillT(const std::vector<Real>& x)
{
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
					T.at(idx(i, j, k)) = new_value;
				}
			}
}

//--------------------------------------------------------------------------------------
// Set up vector of size X*Y*Z (emulates XxYxZ matrix) with T^n values
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupB(std::vector<Real>& b)
{
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			for (int k = 0; k < m_iZ; k++) {
				int t_index = idx(i, j, k);
				b.at(t_index) = T.at(t_index);
			}
}

//--------------------------------------------------------------------------------------
// Set up sparse matrix of size (X*Y*Z)x(X*Y*Z)
// (emulates (XxYxZ)x(XxYxZ) matrix) with linear system's coefficients
//--------------------------------------------------------------------------------------
void DiffusionSimulator::setupA(SparseMatrix<Real>& A,const double& factor)
{
	int center;
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
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
					A.set_element(center, center, 1 + 4 * factor);
					A.set_element(center, idx(i + 1, j, k), -factor);
					A.set_element(center, idx(i - 1, j, k), -factor);
					A.set_element(center, idx(i, j + 1, k), -factor);
					A.set_element(center, idx(i, j - 1, k), -factor);
					if (m_b3D)
					{
						A.add_to_element(center, center, 2 * factor); // TO-DO factor multiplied by 6?
						A.set_element(center, idx(i, j, k + 1), -factor);
						A.set_element(center, idx(i, j, k - 1), -factor);
					}
				}
			}
}

//--------------------------------------------------------------------------------------
// Solve AT=b
//--------------------------------------------------------------------------------------
void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep)
{
	float factor = (m_fAlpha * timeStep) / (m_fDeltaSpace * m_fDeltaSpace);
	int N = m_iX * m_iY * m_iZ;
	SparseMatrix<Real> A (N);
	std::vector<Real> b (N);

	setupA(A, factor);
	setupB(b);

	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
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
	case 2:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 3:
		diffuseTemperatureImplicit(timeStep);
		break;
	default:
		break;
	}
}

//--------------------------------------------------------------------------------------
// Draw current T^n state for each frame using colorful spheres
// Color key:
//	red for negative values
//	green for positive ones
//	black for boundary cells
//--------------------------------------------------------------------------------------
void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 color;
	for (int i = 0; i < m_iX; i++)
		for (int j = 0; j < m_iY; j++)
			for (int k = 0; k < m_iZ; k++)
			{
				if (isBoundary(i, j, k))
					color = Vec3(0.05, 0.05, 0.05);
				else
				{
					if (T.at(idx(i, j, k)) > 0) // sign of temperature
						color = Vec3(0.05, getNormalValue(i, j, k), 0.05);
					else
						color = Vec3(getNormalValue(i, j, k), 0.05, 0.05);
				}
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
				DUC->drawSphere(0.1 * Vec3(i - m_iX / 2, j - m_iY/ 2, k - m_iZ / 2),
					(m_b3D ? 0.03 : 0.07) * Vec3(1, 1, 1));
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