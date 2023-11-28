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
        //positional stuff
        rigidbody.m_position += timeStep * rigidbody.m_velocity;
        rigidbody.m_velocity += timeStep * m_externalForce / rigidbody.m_mass;

        //rotational stuff
        Vec3 externalForceDiff = m_externalForcePosition - rigidbody.m_position;
        Vec3 torque = cross(externalForceDiff, m_externalForce);
        rigidbody.m_rotation += (timeStep / 2) * 
            Quat(0, rigidbody.m_angularVelocity.x, rigidbody.m_angularVelocity.y, rigidbody.m_angularVelocity.z) * rigidbody.m_rotation;
        rigidbody.m_angularMomentum += timeStep * torque;
        auto rotMat = rigidbody.m_rotation.getRotMat();
        auto inverseRotMat = rigidbody.m_rotation.getRotMat();
        inverseRotMat.transpose();
        auto newI = (rotMat * rigidbody.m_initialIntertiaTensor) * inverseRotMat;
        rigidbody.m_angularVelocity = newI * rigidbody.m_angularMomentum;

    }


}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return m_rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return m_rigidbodies[i].m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return m_rigidbodies[i].m_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return m_rigidbodies[i].m_angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    m_rigidbodies.push_back(Rigidbody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    m_rigidbodies[i].m_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    m_rigidbodies[i].m_velocity = velocity;
}








