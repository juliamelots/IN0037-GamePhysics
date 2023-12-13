#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver, Explicit_solver_3D, Implicit_solver_3D";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	// to be implemented
	T.ny = 16;
	T.nx = 16;
	m_alpha = 0.1f;
	T.x_diff_squared = 0.1f * 0.1f;
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	T.t_space.clear();
	for (int i = 0; i < T.nx; i++) {
		std::vector<float> t_row;
		float initial_value = 100;
		for (int j = 0; j < T.ny; j++) {
			for (int k = 0; k < T.nz; k++) {
				if (m_iTestCase < 2)
					t_row.push_back((j == 0 || i == 0 || j == T.ny - 1 || i == T.nx - 1) ? 0 : initial_value);
				else
					t_row.push_back((j == 0 || i == 0 || j == T.ny - 1 || i == T.nx - 1 ||
						k == 0 || k == T.nz - 1) ? 0 : initial_value);
			}
		}
		T.t_space.push_back(t_row);
	}
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0: {
		T.nz = 1;
		cout << "Explicit solver!\n";
		break;
	}
	case 1: {
		T.nz = 1;
		cout << "Implicit solver!\n";
		break;
	}
	case 2: {
		T.nz = T.ny;
		cout << "Implicit solver!\n";
		break;
	}
	case 3: {
		T.nz = T.ny;
		cout << "Implicit solver!\n";
		break;
	}
	default: {
		cout << "Empty Test!\n";
		break;
	}
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	std::vector<std::vector<float>> new_t_space;
	float r{ (m_alpha * timeStep) / (T.x_diff_squared) };
	for (int i = 0; i < T.ny; i++) {
		std::vector<float> t_row;
		float initial_value = 100;
		for (int j = 0; j < T.nx; j++) {
			t_row.push_back((j == 0 || i == 0 || j == T.ny - 1 || i == T.nx - 1) ? 0 :
				T.t_space[i][j] + r * (T.t_space[i + 1][j] + T.t_space[i][j + 1] + T.t_space[i - 1][j]
					+ T.t_space[i][j - 1] - 4 * T.t_space[i][j]));
		}
		new_t_space.push_back(t_row);
	}
	T.t_space = new_t_space;
}



void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	int nx{ T.nx };
	int ny{ T.ny };
	int nz{ T.nz };
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!
	for (int i = 0; i < N; i++) {
		tuple<int, int, int> values = get_space_index(i);
		int space_i = get<0>(values);
		int space_j = get<1>(values);
		int space_k = get<2>(values);
		if (isInBorder(space_i, space_j, space_k)){
			//TODO: borders of the simulation: should set to zero
		}
		else {
			//TODO: set first element for 3D simulation
			A.set_element(i, i, (m_iTestCase < 2) ? (-T.x_diff_squared / (m_alpha * timeStep) - 4) : (0));
			A.set_element(i, get_A_index(space_i + 1, space_j, space_k), 1);
			A.set_element(i, get_A_index(space_i, space_j + 1, space_k), 1);
			A.set_element(i, get_A_index(space_i - 1, space_j, space_k), 1);
			A.set_element(i, get_A_index(space_i, space_j - 1, space_k), 1);
			if (m_iTestCase >= 2) {
				A.set_element(i, get_A_index(space_i, space_j, space_k + 1), 1);
				A.set_element(i, get_A_index(space_i, space_j, space_k - 1), 1);
			}

			b[i] = -T.x_diff_squared / (m_alpha * timeStep) * T.t_space[space_i][space_j];
		}
	}

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
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	for (int i = 0; i < N; i++) {
		tuple<int, int, int> values = get_space_index(i);
		int space_i = get<0>(values);
		int space_j = get<1>(values);
		int space_k = get<2>(values);
		//TODO: add 3D
		if (isInBorder(space_i, space_j, space_k)) {
			T.t_space[space_i][space_j] = 0;
		}
		else {
			T.t_space[space_i][space_j] = x[i];
		}
	}

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
}

int DiffusionSimulator::get_A_index(int i, int j, int k)
{
	int nx{ T.nx }, ny{ T.ny };
	return m_iTestCase < 2 ? nx * i + j : (nx * ny) * i + ny * j + T.nz;
}

std::tuple<int,int,int> DiffusionSimulator::get_space_index(int a_index)
{
	int nx{ T.nx }, ny{ T.ny };
	return m_iTestCase < 2 ? make_tuple( a_index / nx, a_index % nx, 0 ) : make_tuple(a_index / (nx * ny), a_index %(nx *ny) / ny, a_index % (nx * ny) % ny);
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
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

bool DiffusionSimulator::isInBorder(int space_i, int space_j, int space_k)
{
	return (m_iTestCase < 2 && (space_i == 0 || space_i == T.nx - 1 || space_j == 0 || space_j == T.ny - 1)) ||
		(m_iTestCase >= 2 && (space_i == 0 || space_i == T.nx - 1 || space_j == 0 || space_j == T.ny - 1 ||
			space_k == 0 || space_k == T.nz - 1));
}
