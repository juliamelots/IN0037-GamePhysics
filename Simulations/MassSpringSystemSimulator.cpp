#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Set initial data attributes
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	setIntegrator(0);

	// Set initial control attributes
	// [TO-DO] apparently not needed?
	// m_massPoints = new vector<MassPoint*>();
	// m_springs = new vector<Spring*>();

	// Set initial UI attributes
	m_externalForce = Vec3();
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4,Demo 5";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	// [TO-DO] don't allow custom time step for all demos except 4
	case 3:
		TwType TW_TYPE_METHOD = TwDefineEnumFromString("Integrator Method", "Euler,Leap-Frog,Midpoint");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator Method", TW_TYPE_METHOD, &m_iIntegrator, "");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_externalForce = m_externalForce.ZERO;
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (int i = 0; i < getNumberOfMassPoints(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3());
		DUC->drawSphere(getPositionOfMassPoint(i), Vec3(STD_SPHERE_SIZE, STD_SPHERE_SIZE, STD_SPHERE_SIZE);
	}
	for (int i = 0; i < getNumberOfSprings(); i++)
	{
		Spring* currentSpring = m_springs.at(i);
		DUC->beginLine();
		DUC->drawLine(currentSpring->m_point1.m_position, Vec3(0, 1, 0),
					  currentSpring->m_point2.m_position, Vec3(0, 1, 0));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1: simple one-step test\n";
		setUp2PointSystem();
		setIntegrator(EULER);
		simulateTimestep(0.1);
		setIntegrator(MIDPOINT);
		simulateTimestep(0.1);
		break;
	case 1:
		cout << "Demo 2: simple Euler simulation\n";
		setUp2PointSystem();
		setIntegrator(EULER);
		// [TO-DO] h = 0.005
		break;
	case 2:
		cout << "Demo 3: simple Midpoint simulation\n";
		setUp2PointSystem();
		setIntegrator(MIDPOINT);
		// [TO-DO] h = 0.005
		break;
	case 3:
		cout << "Demo 4: complex simulation\n";
		// [TO-DO] set up demo 4
		// [TO-DO] add gravity and collision with floor/walls
		break;
	case 4:
		cout << "Demo 5: optional Leap-frog\n";
		setUp2PointSystem();
		setIntegrator(LEAPFROG);
		// [TO-DO] h = 0.005
		break;
	default:
		cout << "Empty Test\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame using selected integration method
	switch (m_iIntegrator)
	{
	case EULER:
		break;
	case LEAPFROG:
		break;
	case MIDPOINT:
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping= damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint* massPoint = new MassPoint(position, Velocity, isFixed);
	m_massPoints.push_back(massPoint);
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring* spring = new Spring(*m_massPoints.at(masspoint1),
								*m_massPoints.at(masspoint2),
								initialLength);
	m_springs.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{ 
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	MassPoint* massPoint = m_massPoints.at(index);
	return massPoint->m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	MassPoint* massPoint = m_massPoints.at(index);
	return massPoint->m_velocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	// [TO-DO]
}

void MassSpringSystemSimulator::setUp2PointSystem()
{
	setMass(10.0);
	setStiffness(40);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), STD_FIXED);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), STD_FIXED);
	addSpring(0, 1, 1.0);
}