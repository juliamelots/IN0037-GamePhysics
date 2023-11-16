#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 0.0;
	m_fStiffness = 0.0;
	m_fDamping = 0.0;
	m_fGravity= 0.0;
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
	if (m_iTestCase == 3)
	{
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.1 min=0.0");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0.0");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator Method",
			TwDefineEnumFromString("Integrator Method", "Euler,Leap-Frog,Midpoint"),
			&m_iIntegrator, "");
	}
	// control of time step done in main.cpp
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
		DUC->drawSphere(massPoint->m_position, RADIUS * Vec3(1, 1, 1));
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

		// continue as normal euler simulation
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1.0);
		setIntegrator(EULER);
		break;
	case 1:
		cout << "Demo 2: simple Euler simulation\n";
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
		break;
	case 2:
		cout << "Demo 3: simple Midpoint simulation\n";
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
		break;
	case 3:
	{
		cout << "Demo 4: complex simulation\n";
		// reset
		removeMassPoints();
		removeSprings();

		// generate random 10-point system
		setMass(10);
		setStiffness(40);
		std::mt19937 eng(time(nullptr));
		int ballNumber = 10;
		int springNumber = 10;
		std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
		std::uniform_int_distribution<> randPoint(0, ballNumber - 2);
		std::uniform_real_distribution<float> randLength(-0.5f, 0.5f);
		cout << "mass points:\n";
		for (int i = 0; i < ballNumber; i++) {
			Vec3 pos = Vec3(randPos(eng), randPos(eng), randPos(eng));
			Vec3 speed = Vec3(randPos(eng), randPos(eng), randPos(eng));
			addMassPoint(pos, speed, false);
			cout << i << ": position " << pos << " velocity " << speed << ".\n";
		}
		cout << endl;

		for (int i = 0; i < ballNumber; i++) {
			int point1Index = i;
			int point2Index = randPoint(eng);
			if (point2Index >= i) point2Index++;
			float distance = normalize(m_vMassPoints[point1Index]->m_position - m_vMassPoints[point2Index]->m_position) + randLength(eng);
			if (distance < 0) {
				distance = -distance;
			}
			if (distance < 0.001f) {
				distance = 0.001f;
			}
			addSpring(point1Index, point2Index, distance);
		}
		break;
	}
	case 4:
		cout << "Demo 5: optional Leap-frog\n";
		// reset
		removeMassPoints();
		removeSprings();

		// leap-frog
		setMass(10.0);
		setStiffness(40.0);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1.0);
		setIntegrator(LEAPFROG);
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
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	//cout << "world matrix: " << Mat4(DUC->g_camera.GetWorldMatrix()) << "\n";
	//cout << "view matrix: " << Mat4(DUC->g_camera.GetViewMatrix()) << "\n";
	if (mouseDiff.x != 0 || mouseDiff.y != 0) 
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		for (MassPoint* massPoint : m_vMassPoints)
		{
			// project the mass point to the camera plane.
			Vec3 pointPos = massPoint->m_position;
			// check if the L1 distance between the projected point and the old mouse position is below the preset threshold.
			// if true, move the point(s) with mouse.
			if (isClickedPoint(pointPos))
			{
				massPoint->m_oldPosition = massPoint->m_position;
				massPoint->m_position += inputWorld;
			}
		}
	}
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
		for (Spring* spring : m_vSprings)
		{
			applyInternalForce(spring);
		}
		for (MassPoint* massPoint : m_vMassPoints)
		{
			massPoint->m_velocity = calculateNewVelocity(massPoint->m_velocity, massPoint->m_internalForce, timeStep);
			massPoint->m_position = calculateNewPosition(massPoint->m_position, massPoint->m_velocity, timeStep);
			// reset internal force, should be calculated again on next system update
			massPoint->m_internalForce = Vec3();
		}
		break;
	case MIDPOINT:
		// calculate the midpoint positions and velocities
		for (Spring* spring : m_vSprings)
		{
			applyInternalForce(spring);
		}
		for (MassPoint* massPoint : m_vMassPoints)
		{
			massPoint->m_oldPosition = massPoint->m_position;
			massPoint->m_oldVelocity = massPoint->m_velocity;
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
			massPoint->m_position = calculateNewPosition(massPoint->m_oldPosition, massPoint->m_velocity, timeStep);
			massPoint->m_velocity = calculateNewVelocity(massPoint->m_oldVelocity, massPoint->m_internalForce, timeStep);
			// reset internal force, should be calculated again on next system update
			massPoint->m_internalForce = Vec3();
		}
		break;
	default:break;
	}
	checkCollision();
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
	Vec3 acceleration = (internalForce + Vec3(0, -m_fGravity, 0) - m_fDamping * velocity) / m_fMass;
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

void MassSpringSystemSimulator::addRandomMassPoints(int number)
{	
	std::mt19937 eng(time(nullptr));
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-1.0f, 1.0f);
	std::uniform_real_distribution<float> randVel(-1.0f, 1.0f);

	for (int i = 0; i < number; i++) {
		addMassPoint(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(randVel(eng), randVel(eng), randVel(eng)), false);
	}
}

void MassSpringSystemSimulator::addRandomSprings(int number)
{
	std::mt19937 eng(time(nullptr));
	for (int i = 0; i < number; i++) {
		std::uniform_int_distribution<> randPoint(0, getNumberOfMassPoints() - 1);
		std::uniform_real_distribution<float> randLength(-0.5f, 0.5f);
		int point1Index = randPoint(eng);
		int point2Index = randPoint(eng);
		float distance = norm(m_vMassPoints.at(point1Index)->m_position - m_vMassPoints.at(point2Index)->m_position) + randLength(eng);
		addSpring(point1Index, point2Index, distance);
	}
}

bool MassSpringSystemSimulator::isClickedPoint(Vec3 pointPos)
{
	// Transform from world space to camera space (using view matrix)
	Vec3 worldSpacePoint = Mat4(DUC->g_camera.GetWorldMatrix()) * pointPos;
	Vec3 cameraSpacePoint = Mat4(DUC->g_camera.GetViewMatrix()) * worldSpacePoint;

	// Transform from camera space to clip space (using projection matrix)
	Vec3 ndcPoint = Mat4(DUC->g_camera.GetProjMatrix()) * cameraSpacePoint;

	// Perform perspective division to get normalized device coordinates (NDC)
	normalize(ndcPoint);

	float inputScale = 0.001f;
	/*
	cout << "ndcPoint: " << ndcPoint << "\n";
	cout << "x diff: " << abs(ndcPoint.x - m_oldtrackmouse.x * inputScale) << "\n";
	cout << "y diff: " << abs(ndcPoint.y - m_oldtrackmouse.y * inputScale) << "\n";
	*/

	if (abs(ndcPoint.x - m_oldtrackmouse.x * inputScale) < 0.5 && abs(ndcPoint.y - m_oldtrackmouse.y * inputScale) < 0.5)
	{
		return true;
	}

	return false;
}

//--------------------------------------------------------------------------------------
// Check collision of mass points and (maybe) the floor
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::checkCollision()
{
	float minDistanceSquared = (2 * RADIUS) * (2 * RADIUS);
	for (auto point : m_vMassPoints) {
		for (auto otherPoint : m_vMassPoints) {
			if (point == otherPoint) continue;
			if (point->m_position.squaredDistanceTo(otherPoint->m_position) < minDistanceSquared) {
				Vec3 diff = point->m_position - otherPoint->m_position;
				normalize(diff);
				point->m_position = otherPoint->m_position + (2 * RADIUS) * diff;
				point->m_velocity *= -1;
			}
		}
		if (point->m_position.y < FLOOR + RADIUS) {
			point->m_position.y = FLOOR + RADIUS;
			point->m_velocity *= -1;
		}
	}
}