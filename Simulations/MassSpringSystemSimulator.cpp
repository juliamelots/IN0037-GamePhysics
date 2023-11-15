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
		DUC->drawSphere(massPoint->m_position, m_fRadius * Vec3(1, 1, 1));
	}
	DUC->beginLine();
	for (Spring* spring : m_vSprings)
	{
		DUC->drawLine(spring->m_point1->m_position, Vec3(0.5f, 0.5f, 0.5f),
			spring->m_point2->m_position, Vec3(0.5f, 0.5f, 0.5f));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1: simple one-step test\n";
		// reset
		removeMassPoints();
		removeSprings();

		// euler
		setMass(10.0);
		setStiffness(40.0);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1.0);
		setIntegrator(EULER);
		simulateTimestep(0.1);
		// print result after one Euler step
		cout << "Euler\n";
		cout << "positions: " << m_vMassPoints[0]->m_position << " " << m_vMassPoints[1]->m_position << "\n";
		cout << "velocities: " << m_vMassPoints[0]->m_velocity << " " << m_vMassPoints[1]->m_velocity << "\n";
		

		// reset
		removeMassPoints();
		removeSprings();

		// midpoint
		setMass(10.0);
		setStiffness(40.0);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1.0);
		setIntegrator(MIDPOINT);
		simulateTimestep(0.1);
		// print result after one midpoint step
		cout << "Midpoint\n";
		cout << "positions: " << m_vMassPoints[0]->m_position << " " << m_vMassPoints[1]->m_position << "\n";
		cout << "velocities: " << m_vMassPoints[0]->m_velocity << " " << m_vMassPoints[1]->m_velocity << "\n";

		// reset
		removeMassPoints();
		removeSprings();

		setMass(10.0);
		setStiffness(40.0);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1.0);
		setIntegrator(EULER);

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
		// calculate the leap-frog positions and velocities


		break;
	case MIDPOINT:
		// calculate the midpoint positions and velocities
		for (Spring* spring : m_vSprings)
		{
			applyInternalForce(spring);
		}
		for (MassPoint* massPoint : m_vMassPoints)
		{
			massPoint->m_OldPosition = massPoint->m_position;
			massPoint->m_OldVelocity = massPoint->m_velocity;
			massPoint->m_position = calculateNewPosition(massPoint->m_position, massPoint->m_velocity, timeStep / 2.0);
			massPoint->m_velocity = calculateNewVelocity(massPoint->m_velocity, massPoint->m_internalForce, timeStep / 2.0);
			// reset internal force, should be calculated again on next system update
			massPoint->m_internalForce = Vec3();
		}

		for (Spring* spring : m_vSprings)
		{
			applyInternalForce(spring);
		}
		for (MassPoint* massPoint : m_vMassPoints)
		{
			massPoint->m_position = calculateNewPosition(massPoint->m_OldPosition, massPoint->m_velocity, timeStep);
			massPoint->m_velocity = calculateNewVelocity(massPoint->m_velocity, massPoint->m_internalForce, timeStep);
			// reset internal force, should be calculated again on next system update
			massPoint->m_internalForce = Vec3();
		}
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

void MassSpringSystemSimulator::removeMassPoints()
{
	m_vMassPoints.clear();
	m_vMassPoints.shrink_to_fit();
}

void MassSpringSystemSimulator::removeSprings() {
	m_vSprings.clear();
	m_vSprings.shrink_to_fit();
}

//--------------------------------------------------------------------------------------
// Check collision of mass points and (maybe) the floor
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::checkCollision() {
	float minDistanceSquared = (2 * m_fRadius) * (2 * m_fRadius);
	for (auto point : m_vMassPoints) {
		for (auto otherPoint : m_vMassPoints) {
			if (point == otherPoint) continue;
			if (point->m_position.squaredDistanceTo(otherPoint->m_position) < minDistanceSquared) {
				Vec3 diff = point->m_position - otherPoint->m_position;
				normalize(diff);
				point->m_position = otherPoint->m_position + (2 * m_fRadius) * diff;
			}
		}
		/*if (point.getPosition().z < m_fRadius) {
			Vec3 position = point.getPosition();
			position.z = m_fRadius;
			point.correctPosition(position);
		} */
	}
}