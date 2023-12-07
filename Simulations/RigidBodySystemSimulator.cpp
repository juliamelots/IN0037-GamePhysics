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
    float c = 1;
    for (size_t i = 0; i < m_rigidBodies.size(); i++) {
        Mat4 worldPosMat = m_rigidBodies[i].getlocalToWorldMat();
        for (size_t j = i + 1; j < m_rigidBodies.size(); j++) {
            CollisionInfo collision = checkCollisionSAT(worldPosMat, m_rigidBodies[j].getlocalToWorldMat());
            if (collision.isValid) {

                RigidBody& rigidBody_a = m_rigidBodies[i];
                RigidBody& rigidBody_b = m_rigidBodies[j];
                auto x_a = collision.collisionPointWorld - rigidBody_a.m_position;
                auto x_b = collision.collisionPointWorld - rigidBody_b.m_position;
                auto v_tild_a = rigidBody_a.getVelocityOfPosition(x_a);
                auto v_tild_b = rigidBody_b.getVelocityOfPosition(x_b);
                auto v_rel = v_tild_b - v_tild_a;
                cout << collision.normalWorld << endl;
                if (dot(v_rel, collision.normalWorld) > 0) {
                    auto j = -(1 + c) * dot(v_rel, collision.normalWorld) / (1 / rigidBody_a.m_mass + 1 / rigidBody_b.m_mass + dot(
                        cross(rigidBody_a.getInverseIntertiaTensor().transformVector(cross(x_a, collision.normalWorld)), x_a) +
                        cross(rigidBody_b.getInverseIntertiaTensor().transformVector(cross(x_b, collision.normalWorld)), x_b), collision.normalWorld)
                        );
                    cout << "before velocity" << rigidBody_a.m_velocity;
                    rigidBody_a.m_velocity -= (j / rigidBody_a.m_mass) * collision.normalWorld;
                    cout << "after velocity" << rigidBody_a.m_velocity;
                    rigidBody_b.m_velocity += (j / rigidBody_b.m_mass) * collision.normalWorld;
                    cout << endl;
                    rigidBody_a.m_angularMomentum = rigidBody_a.m_angularMomentum + cross(x_a, j * collision.normalWorld);
                    rigidBody_b.m_angularMomentum = rigidBody_b.m_angularMomentum - cross(x_b, j * collision.normalWorld);
                    cout << "collide" << endl;
                }
            }
        }
    }


    for (auto& rigidBody : m_rigidBodies) {
        //positional stuff
        rigidBody.m_position += timeStep * rigidBody.m_velocity;
        rigidBody.m_translationMatrix.initTranslation(rigidBody.m_position.x, rigidBody.m_position.y, rigidBody.m_position.z);

        rigidBody.m_velocity += timeStep * m_externalForce / rigidBody.m_mass;
        //rotational stuff
        Vec3 externalForceDiff = m_externalForcePosition - rigidBody.m_position;
        Vec3 torque = cross(externalForceDiff, m_externalForce);
        rigidBody.m_rotation += (timeStep / 2) *
            Quat(0, rigidBody.m_angularVelocity.x, rigidBody.m_angularVelocity.y, rigidBody.m_angularVelocity.z) * rigidBody.m_rotation;
        rigidBody.m_rotation = rigidBody.m_rotation.unit();
        rigidBody.m_angularMomentum += timeStep * torque;
        auto rotMat = rigidBody.m_rotation.getRotMat();
        auto inverseRotMat = rigidBody.m_rotation.getRotMat();
        inverseRotMat.transpose();
        auto newI = (rotMat * rigidBody.m_initialInverseIntertiaTensor) * inverseRotMat;
        rigidBody.m_angularVelocity = newI * rigidBody.m_angularMomentum;
    }
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
