#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fRadius = 0.05f;
	m_fSphereSize = m_fRadius * Vec3(1,1,1);
	m_fDamping = 0.0f;
	setIntegrator(EULER);
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	/*switch (m_iTestCase) {
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
	switch (m_iTestCase) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		std::mt19937 eng;
		std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
		for (Point point : m_points) {
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
			DUC->drawSphere(point.getPosition(), m_fSphereSize);
		}
		DUC->beginLine();
		for (Spring spring : m_springs) {
			//std:cout << points[spring.point1].getPosition() << " " << points[spring.point2].getPosition() << std :: endl;
			DUC->drawLine(m_points[spring.point1].getPosition(), Vec3(0.5f, 0.5f, 0.5f), m_points[spring.point2].getPosition(), Vec3(0.5f, 0.5f, 0.5f));
		}
		DUC->endLine();
		break;
		//TODO add 10 points;

	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	cout << "called notify"  << testCase << std::endl;
	cleanSpace();
	m_printedDemo = false;
	switch (m_iTestCase) {
	case 0:
		cout << "Demo 1!!";
		setMass(10);
		setStiffness(40);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		break;
	case 1:
		cout << "Demo 2!!";
		setMass(10);
		setStiffness(40);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		setIntegrator(EULER);
		break;
	case 2:
		cout << "Demo 3!!";
		setMass(10);
		setStiffness(40);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		setIntegrator(MIDPOINT);
		break;
	}

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::cleanSpace() {
	m_springs.clear();
	m_springs.shrink_to_fit();
	m_points.clear();
	m_points.shrink_to_fit();
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
	m_points.push_back(newPoint);
	return m_points.size();
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring newSpring = Spring(masspoint1, masspoint2, initialLength);
	m_springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return m_points[index].getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return m_points[index].getVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_externalForce = force;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//Calculate current force on all Points
	setForces();
	//Move every point
	if (m_iIntegrator == EULER) {
		for (Point& point : m_points) {
			point.movePoint(timeStep, false);
			point.setSpeed(m_externalForce, m_fMass, timeStep,false);
		}
		checkCollision();
	}
	else if (m_iIntegrator == MIDPOINT) {
		float halfTimeStep = timeStep / 2;
		for (Point& point : m_points) {
			point.movePoint(halfTimeStep, true);
			point.setSpeed(m_externalForce, m_fMass, halfTimeStep, true);
		}
		setForces();
		for (Point& point : m_points) {
			point.movePoint(timeStep, false);
			cout << "force: " << point.getForceVelocity() << " " << point.getForcePosition() << endl;
			point.setSpeed(m_externalForce, m_fMass, timeStep, false);
			cout << "force2: " << point.getForceVelocity() << " " << point.getForcePosition() << endl;
		}
	}

	else if (m_iIntegrator == LEAPFROG) {
		for (Point& point : m_points) {
			point.setSpeed(m_externalForce, m_fMass, timeStep, false);
			point.movePoint(timeStep, false);
		}
		checkCollision();
	}

	if ( !m_printedDemo) {
		cout << "positions: " << m_points[0].getPosition() << " " << m_points[1].getPosition() << std::endl;
		cout << "speeds: " << m_points[0].getVelocity() << " " << m_points[1].getVelocity() << std::endl;
		m_printedDemo = true;
	}
}

void MassSpringSystemSimulator::checkCollision() {
	float minDistanceSquared = (2 * m_fRadius) * (2 * m_fRadius);
	for (Point& point : m_points) {
		for (Point& otherPoint : m_points) {
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
	for (Point& point : m_points) {
		point.setDamping(m_fDamping);
	}
	for (Spring spring : m_springs) {
		Point& firstPoint = m_points[spring.point1];
		Point& secondPoint = m_points[spring.point2];
		Vec3 delta = firstPoint.getForcePosition() - secondPoint.getForcePosition();

		float springSize = normalize(delta);
		float springForce = (springSize - spring.initialLength) * m_fStiffness;
		secondPoint.addSpringForce(springForce * delta);
		firstPoint.addSpringForce((-springForce) * delta);
	}
}