#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include<ctime>

using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_coldColor = HSVColor(nVec3i(0,236,255));
	m_hotColor = HSVColor(nVec3i(255, 19, 0));
	T.ny = 50;
	T.nx = 50;
	T.nz = 1;
	m_alpha = 0.001f;
	T.x_diff_squared = 0.1f * 0.1f;
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
	switch (m_iTestCase) {
	case 0:
	case 1:
		//TwAddVarCB(DUC->g_pTweakBar, "nx", TW_TYPE_INT32, DiffusionSimulator::setNx, DiffusionSimulator::getNx, this,"min=1");
		//TwAddVarCB(DUC->g_pTweakBar, "ny", TW_TYPE_INT32, DiffusionSimulator::setNy, DiffusionSimulator::getNy, this, "min=1");
		break;
	case 2:
	case 3:
		//TwAddVarCB(DUC->g_pTweakBar, "nx", TW_TYPE_INT32, DiffusionSimulator::setNx, DiffusionSimulator::getNx, this, "min=1");
		//TwAddVarCB(DUC->g_pTweakBar, "ny", TW_TYPE_INT32, DiffusionSimulator::setNy, DiffusionSimulator::getNy, this, "min=1");
		//TwAddVarCB(DUC->g_pTweakBar, "nz", TW_TYPE_INT32, DiffusionSimulator::setNz, DiffusionSimulator::getNz, this, "min=1");
		break;
	}
	// to be implemented
	
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	T.t_space.clear();
	std::mt19937 mt(time(nullptr));
	std::uniform_real_distribution<float> randCol(0.0f, 500.0f);
	for (int i = 0; i < T.nx; i++) {
		std::vector<float> t_row;
		float initial_value = 500;
		for (int j = 0; j < T.ny; j++) {
			for (int k = 0; k < T.nz; k++) {
				if (m_iTestCase < 2)
					t_row.push_back((j == 0 || i == 0 || j == T.ny - 1 || i == T.nx - 1 ) ? 0 : randCol(mt));
				else
					t_row.push_back((j == 0 || i == 0 || j == T.ny - 1 || i == T.nx - 1 ||
						k == 0 || k == T.nz - 1) ? 0 : randCol(mt));
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
	for (int i = 0; i < T.nx; i++) {
		std::vector<float> t_row;
		for (int j = 0; j < T.ny; j++) {
			if ((j == 0 || i == 0 || j == T.nx - 1 || i == T.ny - 1)) t_row.push_back(0);
			else 
				t_row.push_back( T.t_space[i][j] + r * (T.t_space[i + 1][j] + T.t_space[i][j + 1] + T.t_space[i - 1][j]
					+ T.t_space[i][j - 1] - 4 * T.t_space[i][j]));
		}
		new_t_space.push_back(t_row);
	}

	//for (int i = 0; i < T.nx; i++) {
		//for (int j = 0; j < T.ny; j++) {
			//cout << new_t_space[i][j] << " ";
		//}
		//cout << endl;
	//}
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
			A.set_element(i, i, 1);
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
	const int space_min = -1;
	const int space_max = 1;
	float x_diff = static_cast<float>(space_max - space_min) / static_cast<float>(T.nx);
	float y_diff = static_cast<float>(space_max - space_min) / static_cast<float>(T.ny);
	//Vec3 coldColor = Vec3(0, 236, 255) / 255.0;
	//Vec3 hotColor = Vec3(255, 19, 0) / 255.0;
	Vec3 coldColor = Vec3(0, 0, 0);
	Vec3 hotColor = Vec3(1, 1, 1);
	if (m_iTestCase < 2) {
		for (int i = 0; i < T.nx; i++) {
			for (int j = 0; j < T.ny; j++) {
				float rel_temp = (T.t_space[i][j] - min_temp) / (max_temp - min_temp);
				//Vec3 color = colorLerp(rel_temp);
				Vec3 color = coldColor * (1 - rel_temp) + hotColor * rel_temp;
				DUC->setUpLighting(Vec3(), color, 80,color);
				DUC->drawSphere(Vec3(space_min + x_diff * i, space_min + y_diff * j, 0), Vec3(0.1,0.1,0.1));
			}
		}
	}
	else {
		float z_diff = static_cast<float>(space_max - space_min) / static_cast<float>(T.nz);
	}
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

Vec3 DiffusionSimulator::colorLerp(float rel_t)
{
	float new_h = static_cast<float>(m_coldColor.hsv[0]) * (1 - rel_t) + rel_t * static_cast<float>(m_hotColor.hsv[0]);
	HSVColor new_color = HSVColor(new_h, m_coldColor.hsv[1], m_coldColor.hsv[2]);
	return new_color.toRGB();
}

HSVColor::HSVColor(nVec3i rgbValues)
{
	unsigned char rgbMin, rgbMax;

	rgbMin = rgbValues[0] < rgbValues[1] ? (rgbValues[0] < rgbValues[2] ? rgbValues[0] : rgbValues[2]) : (rgbValues[1] < rgbValues[2] ? rgbValues[1] : rgbValues[2]);
	rgbMax = rgbValues[0] > rgbValues[1] ? (rgbValues[0] > rgbValues[2] ? rgbValues[0] : rgbValues[2]) : (rgbValues[1] > rgbValues[2] ? rgbValues[1] : rgbValues[2]);

	hsv[2] = rgbMax;
	if (hsv[2] == 0)
	{
		hsv[0] = 0;
		hsv[1] = 0;
		return;
	}

	hsv[1] = 255 * long(rgbMax - rgbMin) / hsv[2];
	if (hsv[1] == 0)
	{
		hsv[0] = 0;
		return;
	}

	if (rgbMax == rgbValues[0])
		hsv[0] = 0 + 43 * (rgbValues[1] - rgbValues[2]) / (rgbMax - rgbMin);
	else if (rgbMax == rgbValues[1])
		hsv[0] = 85 + 43 * (rgbValues[2] - rgbValues[0]) / (rgbMax - rgbMin);
	else
		hsv[0] = 171 + 43 * (rgbValues[0] - rgbValues[1]) / (rgbMax - rgbMin);
}

HSVColor::HSVColor(float h, float s, float v)
{
	hsv[0] = h;
	hsv[1] = s;
	hsv[2] = v;
}

Vec3 HSVColor::toRGB()
{
	Vec3 rgb;
	unsigned char region, remainder, p, q, t;

	if (hsv[1] == 0)
	{
		rgb[0] = hsv[2];
		rgb[1] = hsv[2];
		rgb[2] = hsv[2];
		return rgb / 255.0;
	}

	region = hsv[0] / 43;
	remainder = (hsv[0] - (region * 43)) * 6;

	p = (hsv[2] * (255 - hsv[1])) >> 8;
	q = (hsv[2] * (255 - ((hsv[1] * remainder) >> 8))) >> 8;
	t = (hsv[2] * (255 - ((hsv[1] * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
	case 0:
		rgb[0] = hsv[2]; rgb[1] = t; rgb[2] = p;
		break;
	case 1:
		rgb[0] = q; rgb[1] = hsv[2]; rgb[2] = p;
		break;
	case 2:
		rgb[0] = p; rgb[1] = hsv[2]; rgb[2] = t;
		break;
	case 3:
		rgb[0] = p; rgb[1] = q; rgb[2] = hsv[2];
		break;
	case 4:
		rgb[0] = t; rgb[1] = p; rgb[2] = hsv[2];
		break;
	default:
		rgb[0] = hsv[2]; rgb[1] = p; rgb[2] = q;
		break;
	}
	return rgb / 255.0;
}
