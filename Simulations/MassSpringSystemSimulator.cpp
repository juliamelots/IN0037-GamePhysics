#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
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
	// [TO-DO] don't allow custom time step for all demos except 4
	case 3:
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
	for (int i = 0; i < getNumberOfMassPoints(); i++)
	{
		DUC->setUpLighting(Vec3(1,0,0), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3());
		DUC->drawSphere(getPositionOfMassPoint(i), Vec3(STD_SPHERE_SIZE, STD_SPHERE_SIZE, STD_SPHERE_SIZE));
	}
	for (int i = 0; i < getNumberOfSprings(); i++)
	{
		Spring* spring = m_springs.at(i);
		DUC->beginLine();
		DUC->drawLine(spring->m_point1->m_position, Vec3(0, 1, 0),
					  spring->m_point2->m_position, Vec3(0, 1, 0));
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
		cout << "euler ";
		printFrame();

		setUp2PointSystem();
		setIntegrator(MIDPOINT);
		simulateTimestep(0.1);
		cout << "midpoint ";
		printFrame();

		// [TO-DO] limit Demo 1 to only these steps
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
//	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
//	Point2D mouseDiff;
//	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
//	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
//	if (mouseDiff.x != 0 || mouseDiff.y != 0)
//	{
//		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
//		worldViewInv = worldViewInv.inverse();
//		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
//		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
//		// find a proper scale!
//float inputScale = 0.001f;
//inputWorld = inputWorld * inputScale;
//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
//	}
//	else {
//		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
//	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame using selected integration method
	switch (m_iIntegrator)
	{
		case EULER:
			for (int i = 0; i < getNumberOfSprings(); i++)
			{
				Spring* spring = m_springs.at(i);
				applyInternalForce(spring);
			}
			for (int i = 0; i < getNumberOfMassPoints(); i++)
			{	
				MassPoint* masspoint = m_massPoints.at(i);
				masspoint->m_position = integratePosition(masspoint->m_position, masspoint->m_velocity, timeStep);
				masspoint->m_velocity = integrateVelocity(masspoint->m_velocity, masspoint->m_internalForce, timeStep);
				masspoint->m_internalForce = Vec3();
			}
			break;
		case LEAPFROG:
			break;
		case MIDPOINT:
			for (int i = 0; i < getNumberOfSprings(); i++)
			{
				Spring* spring = m_springs.at(i);
				applyInternalForce(spring);
			}
			for (int i = 0; i < getNumberOfMassPoints(); i++)
			{
				MassPoint* masspoint = m_massPoints.at(i);
				Vec3 tempPosition = integratePosition(masspoint->m_position, masspoint->m_velocity, timeStep / 2.0);
				Vec3 tempVelocity = integrateVelocity(masspoint->m_velocity, masspoint->m_internalForce, timeStep / 2.0);
				Vec3 tempInternalForce = applyInternalForce(tempPosition, tempVelocity, masspoint->m_attachedSprings);
				masspoint->m_position = integratePosition(masspoint->m_position, tempVelocity, timeStep);
				masspoint->m_velocity = integrateVelocity(masspoint->m_velocity, tempInternalForce, timeStep);
				masspoint->m_internalForce = Vec3(); // reset
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint* massPoint = new MassPoint(position, Velocity, isFixed);
	m_massPoints.push_back(massPoint);
	return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int indexPoint1, int indexPoint2, float initialLength)
{
	MassPoint* point1 = m_massPoints.at(indexPoint1);
	MassPoint* point2 = m_massPoints.at(indexPoint2);
	Spring* spring = new Spring(point1, point2, initialLength);
	point1->m_attachedSprings.push_back(m_springs.size());
	point2->m_attachedSprings.push_back(m_springs.size());
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
	return massPoint->m_velocity;
}

// Simulation Functions
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	// [TO-DO]
}

void MassSpringSystemSimulator::applyInternalForce(Spring* spring)
{	
	float springTerm = -m_fStiffness
					   * (spring->getCurrentLength() - spring->m_fInitialLength)
					   / spring->getCurrentLength();
	cout << spring->getCurrentLength() << "\n";
	spring->m_point1->m_internalForce += springTerm * (spring->m_point1->m_position - spring->m_point2->m_position);
	spring->m_point2->m_internalForce += springTerm * (spring->m_point2->m_position - spring->m_point1->m_position);
	cout << springTerm << "\n";
}

Vec3 MassSpringSystemSimulator::applyInternalForce(Vec3 position, Vec3 velocity, vector<int> attachedSprings)
{
	Vec3 internalForce = Vec3();
	for (int i = 0; i < attachedSprings.size(); i++)
	{
		Spring* spring = m_springs.at(i);
		float springTerm = -m_fStiffness
						   * (spring->getCurrentLength() - spring->m_fInitialLength)
						   / spring->getCurrentLength();
		// unknown which of the endpoints is the current masspoint
		// distance between current masspoint and both of the endpoints are calculated
		// one will be zero (current masspoint = endpoint) and other will be desired value
		Vec3 distance = (spring->m_point1->m_position - spring->m_point2->m_position)
						+ (spring->m_point2->m_position - spring->m_point1->m_position);
		internalForce += springTerm * distance;
	}
	return internalForce;
}

Vec3 MassSpringSystemSimulator::integratePosition(Vec3 position, Vec3 velocity, float timeStep)
{
	return position + timeStep * velocity;
}

Vec3 MassSpringSystemSimulator::integrateVelocity(Vec3 velocity, Vec3 internalForce, float timeStep)
{
	Vec3 acceleration = (internalForce - m_fDamping * velocity) / m_fMass;
	return velocity + timeStep * acceleration;
}

void MassSpringSystemSimulator::setUp2PointSystem()
{	
	m_massPoints.clear();
	m_springs.clear();
	setMass(10.0);
	setStiffness(40);
	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), STD_FIXED);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), STD_FIXED);
	addSpring(0, 1, 1);
}

void MassSpringSystemSimulator::printFrame()
{
	for (int i = 0; i < getNumberOfMassPoints(); i++)
	{
		MassPoint* masspoint = m_massPoints.at(i);
		cout << "pos: " <<  masspoint->m_position
			 << " vel: " << masspoint->m_velocity
			 << " for: " << masspoint->m_internalForce << "\n";
	}
}