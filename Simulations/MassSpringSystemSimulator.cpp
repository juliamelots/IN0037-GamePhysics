#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 0.0;
	m_fStiffness = 0.0;
	m_fDamping = 0.0;
	m_iIntegrator = EULER;
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
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
		// [TO-DO] don't allow custom time step for all demos except 4
	case 3:
		// allow user to choose integration method
		TwAddVarRW(DUC->g_pTweakBar, "Integrator Method",
			TwDefineEnumFromString("Integrator Method", "Euler,Leap-Frog,Midpoint"),
			&m_iIntegrator, "");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_externalForce = Vec3();
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (MassPoint* massPoint : m_vMassPoints)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, 1, 0));
		DUC->drawSphere(massPoint->m_position, Vec3(0.05, 0.05, 0.05));
	}
	for (Spring* spring : m_vSprings)
	{
		DUC->beginLine();
		DUC->drawLine(spring->m_point1->m_position, Vec3(0.5f, 0.5f, 0.5f),
			spring->m_point2->m_position, Vec3(0.5f, 0.5f, 0.5f));
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
		// [TO-DO] set up 2-point mass-spring system
		setIntegrator(EULER);
		simulateTimestep(0.1);
		// [TO-DO] print result after one Euler step

		// [TO-DO] reset up 2-point mass-spring system
		setIntegrator(MIDPOINT);
		simulateTimestep(0.1);

		// [TO-DO] limit Demo 1 to only these steps
		break;
	case 1:
		cout << "Demo 2: simple Euler simulation\n";
		// [TO-DO] set up 2-point mass-spring system
		setIntegrator(EULER);
		// [TO-DO] h = 0.005
		break;
	case 2:
		cout << "Demo 3: simple Midpoint simulation\n";
		// [TO-DO] set up 2-point mass-spring system
		setIntegrator(MIDPOINT);
		// [TO-DO] h = 0.005
		break;
	case 3:
		cout << "Demo 4: complex simulation\n";
		// [TO-DO] set up 10-point mass-spring system
		break;
	case 4:
		cout << "Demo 5: optional Leap-frog\n";
		// [TO-DO] set up 2-point mass-spring system
		setIntegrator(LEAPFROG);
		// [TO-DO] h = 0.005
		break;
	default:
		cout << "Empty Test\n";
		break;
	}
}

//--------------------------------------------------------------------------------------
// Calculate gravity and user interaction forces
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// [TO-DO]
}

//--------------------------------------------------------------------------------------
// Update system using selected integration method
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iIntegrator)
	{
	case EULER:
		for (Spring* spring : m_vSprings)
		{
			applyInternalForce(spring);
		}
		for (MassPoint* massPoint : m_vMassPoints)
		{
			massPoint->m_position = calculateNewPosition(massPoint->m_position, massPoint->m_velocity, timeStep);
			massPoint->m_velocity = calculateNewVelocity(massPoint->m_velocity, massPoint->m_internalForce, timeStep);
			// reset internal force, should be calculated again on next system update
			massPoint->m_internalForce = Vec3();
		}
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
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	MassPoint* massPoint = new MassPoint(position, velocity, isFixed);
	m_vMassPoints.push_back(massPoint);
	return m_vMassPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int indexPoint1, int indexPoint2, float initialLength)
{
	MassPoint* point1 = m_vMassPoints.at(indexPoint1);
	MassPoint* point2 = m_vMassPoints.at(indexPoint2);
	Spring* spring = new Spring(point1, point2, initialLength);
	m_vSprings.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_vMassPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	MassPoint* massPoint = m_vMassPoints.at(index);
	return massPoint->m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	MassPoint* massPoint = m_vMassPoints.at(index);
	return massPoint->m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	// [TO-DO]
}

//--------------------------------------------------------------------------------------
// Calculate and apply internal force of spring to its endpoints
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::applyInternalForce(Spring* spring)
{
	float springTerm = -m_fStiffness
		* (spring->getCurrentLength() - spring->m_fInitialLength)
		/ spring->getCurrentLength();
	spring->m_point1->m_internalForce += springTerm * (spring->m_point1->m_position - spring->m_point2->m_position);
	spring->m_point2->m_internalForce += springTerm * (spring->m_point2->m_position - spring->m_point1->m_position);
}

//--------------------------------------------------------------------------------------
// Calculate new position after time step using velocity as derivative
//--------------------------------------------------------------------------------------
Vec3 MassSpringSystemSimulator::calculateNewPosition(Vec3 position, Vec3 velocity, float timeStep)
{
	return position + timeStep * velocity;
}

//--------------------------------------------------------------------------------------
// Calculate new velocity after time step using accerelation as derivative
//--------------------------------------------------------------------------------------
Vec3 MassSpringSystemSimulator::calculateNewVelocity(Vec3 velocity, Vec3 internalForce, float timeStep)
{
	Vec3 acceleration = (internalForce - m_fDamping * velocity) / m_fMass;
	return velocity + timeStep * acceleration;
}