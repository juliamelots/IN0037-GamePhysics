#include "RigidBodySystemSimulator.h"

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Demo1: One-Step, Demo2: Single-body, Demo3: Two-body, Demo4: Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{

}

void RigidBodySystemSimulator::reset()
{
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
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

void RigidBodySystemSimulator::calculateTorque(float timeStep)
{
    for (auto rigidbody : m_rigidbodies) {
        Vec3 externalForceDiff = m_eternalForcePosition - rigidbody.m_position;
        Vec3 torque = cross(externalForceDiff, m_externalForce);
        rigidbody.m_rotation += (timeStep / 2) * 
            Quat(0, rigidbody.m_angularVelocity.x, rigidbody.m_angularVelocity.y, rigidbody.m_angularVelocity.z) * rigidbody.m_rotation;
        rigidbody.m_angularMomentum += timeStep * torque;


    }


}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
}








