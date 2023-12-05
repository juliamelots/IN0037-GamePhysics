#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1: One-Step, Demo2: Single-body, Demo3: Two-body, Demo4: Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    auto i = 0;
    std::mt19937 eng;
    std::uniform_real_distribution<float> randColor(0, 1);
    for (auto rigidbody : m_rigidBodies) {
        DUC->setUpLighting(rigidbody.m_position, 0.4 * Vec3(1, 1, 1), 100, Vec3(randColor(eng), randColor(eng), randColor(eng)));
        Mat4 drawMat = rigidbody.getlocalToWorldMat();
        DUC->drawRigidBody(drawMat);
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
    {
        cout << "Demo 1: A simple one-step test" << endl;
        break;
    }
    case 1:
    {
        cout << "Demo 2: Simple single-body simulation" << endl;
        break;
    }
    case 2:
    {
        cout << "Demo 3: Simple collision simulation" << endl;
        break;
    }
    case 3:
    {
        cout << "Demo 4: Complex simulation" << endl;
        break;
    }
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    m_rigidBodies.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::removeRigidBodies()
{
    m_rigidBodies.clear();
    m_rigidBodies.shrink_to_fit();
}



Vec3 RigidBody::getVelocityOfPosition(Vec3 pos) {
	return m_velocity + cross(m_angularVelocity, pos);
}

Vec3 RigidBody::getWorldPositionOfPoint(Vec3 pos)
{
    return m_position + m_rotation.getRotMat().transformVector(pos);
}

Mat4 RigidBody::getInverseIntertiaTensor()
{
	auto rotMat = m_rotation.getRotMat();
	auto inverseRotMat = m_rotation.getRotMat();
	inverseRotMat.transpose();
	auto newInverseI = (rotMat * m_initialInverseIntertiaTensor) * inverseRotMat;
	return newInverseI;
}

Mat4 RigidBody::getlocalToWorldMat()
{
	return m_scaleMatrix * m_rotation.getRotMat() * m_translationMatrix;
}
