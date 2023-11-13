#include "MassSpringSystemSimulator.h"

void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	Point point{};
	point.position = position;
	point.velocity = velocity;
	point.isFixed = isFixed;
	point.damping = this->m_fDamping;
	
	this->points.push_back(point);
	return this->points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring spring{};
	spring.point1 = masspoint1;
	spring.point2 = masspoint2;
	spring.stiffness = this->m_fStiffness;;
	spring.initialLength = initialLength;

	this->springs.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return this->points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return this->springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return this->points.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return this->points.at(index).velocity;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	this->DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	Vec3 color{ 1, 1, 1 };
	for (auto spring : this->springs) {
		this->DUC->beginLine();
		auto point1 = this->getPositionOfMassPoint(spring.point1);
		auto point2 = this->getPositionOfMassPoint(spring.point2);		
		this->DUC->drawLine(point1, color, point2, color);
		this->DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	this->m_iTestCase = testCase;
}


const char* MassSpringSystemSimulator::getTestCasesStr() {
	 return std::to_string(this->m_iTestCase).c_str();
}

void MassSpringSystemSimulator::reset() {
	this->points = vector<Point>();
	this->springs = vector<Spring>();

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

MassSpringSystemSimulator::MassSpringSystemSimulator() {

	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	this->m_fMass = 0;
	this->m_fDamping = 0;
	this->m_fStiffness = 0;
	this->m_iIntegrator = EULER;
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

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	this->m_externalForce = force;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	for (auto &pos : this->points) {
		pos.velocity += this->accExternalForce = (this->m_externalForce / this->m_fMass / this->points.size()) * timeElapsed;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeElapsed)
{
	const int dim = this->points.size();
	vector<vector<Vec3>> accs(dim, vector<Vec3>(dim));
	vector<Vec3> sumAccs(dim);

	externalForcesCalculations(timeElapsed);

	for (auto spring : this->springs) {
		auto p1 = this->getPositionOfMassPoint(spring.point1);
		auto p2 = this->getPositionOfMassPoint(spring.point2);
		auto diff = p2 - p1;
		Vec3 acc = -spring.stiffness * (diff - spring.initialLength);
		accs[spring.point1][spring.point2] = acc;
	}
	for (int i = 0; i < this->points.size(); i++) {
		sumAccs[i] = 0;
		for (int j = 0; j < this->points.size(); j++) {
			sumAccs[i] += accs[i][j];
		}
	}

	auto mi = this->m_fMass / dim;

	for (int i = 0; i < this->points.size(); i++) {
		auto &pos = this->points[i];
		if (this->m_iIntegrator == EULER) {
			pos.position += timeElapsed * pos.velocity;
			pos.velocity += (sumAccs[i] - this->m_fDamping * pos.velocity / mi) * timeElapsed;
		}
		if (this->m_iIntegrator == MIDPOINT) {
			auto vtmp = pos.velocity + 0.5 * timeElapsed * (this->accExternalForce + sumAccs[i] - this->m_fDamping * pos.velocity);
			pos.position += timeElapsed * vtmp;
			pos.velocity += timeElapsed * (sumAccs[i] - this->m_fDamping * vtmp[i] / mi);

		}

	}
}
