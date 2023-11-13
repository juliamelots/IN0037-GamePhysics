#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
	setMass(10);
	setStiffness(40);
	addMassPoint(Vec3(0,0,0), Vec3(-1,0,0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0,1,1);
	m_fRadius = 0.05f;
	m_fSphereSize = m_fRadius * Vec3(1,1,1);
	m_fDamping = 0.0f;
	setIntegrator(EULER);
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return nullptr;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	/*switch (m_DemoNumber) {
	case 0: 
		break;
	case 1:

	}*/
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	//TODO: reset 
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	for (Point point : points) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(point.getPosition(), m_fSphereSize);
	}
	DUC->beginLine();
	for (Spring spring : springs) {
	//std:cout << points[spring.point1].getPosition() << " " << points[spring.point2].getPosition() << std :: endl;
		DUC->drawLine(points[spring.point1].getPosition(), Vec3(0.5f,0.5f,0.5f), points[spring.point2].getPosition(), Vec3(0.5f,0.5f,0.5f));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_DemoNumber = testCase;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	Point newPoint = Point(position, Velocity, isFixed);
	points.push_back(newPoint);
	return points.size();
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring newSpring = Spring(masspoint1, masspoint2, initialLength);
	springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return points[index].getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return points[index].getVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_externalForce = force;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//Calculate current force on all Points
	setForces();
	//Move every point
	if (m_iIntegrator == EULER) {
		for (Point& point : points) {
			point.movePoint(timeStep);
			point.setSpeed(m_externalForce, m_fMass, timeStep);
		}
		checkCollision();
	}
	else if (m_iIntegrator == MIDPOINT) {
		float halfTimeStep = timeStep / 2;
		for (Point point : points) {
			point.movePoint(halfTimeStep);
			point.setSpeed(m_externalForce, m_fMass, halfTimeStep);
		}
		checkCollision();
		setForces();
		for (Point point : points) {
			point.movePoint(halfTimeStep);
			point.setSpeed(m_externalForce, m_fMass, halfTimeStep);
		}
	}
	else if (m_iIntegrator == LEAPFROG) {
		for (Point point : points) {
			point.setSpeed(m_externalForce, m_fMass, timeStep);
			point.movePoint(timeStep);
		}
		checkCollision();
	}
}

void MassSpringSystemSimulator::checkCollision() {
	float minDistanceSquared = (2 * m_fRadius) * (2 * m_fRadius);
	for (Point& point : points) {
		for (Point& otherPoint : points) {
			if (&point == &otherPoint) continue;
			if (point.getPosition().squaredDistanceTo(otherPoint.getPosition()) < minDistanceSquared) {
				Vec3 diff = point.getPosition() - otherPoint.getPosition();
				norm(diff);
				point.correctPosition(otherPoint.getPosition() + (2 * m_fRadius) * diff);
			}
		}
		if (point.getPosition().z < m_fRadius) {
			Vec3 position = point.getPosition();
			position.z = m_fRadius;
			point.correctPosition(position);
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setForces() {
	for (Point& point : points) {
		point.setDamping(m_fDamping);
	}
	for (Spring spring : springs) {
		Point& firstPoint = points[spring.point1];
		Point& secondPoint = points[spring.point2];
		Vec3 delta = firstPoint.getPosition() - secondPoint.getPosition();

		float springSize = normalize(delta);
		float springForce = (springSize - spring.initialLength) * m_fStiffness;
		secondPoint.addSpringForce(springForce * delta);
		firstPoint.addSpringForce((-springForce) * delta);
	}
}