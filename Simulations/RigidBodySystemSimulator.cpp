#include "RigidBodySystemSimulator.h"


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Demo1: One-Step, Demo2: Single-body, Demo3: Two-body, Demo4: Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    for (auto rigidbody : m_rigidbodies) {
        DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, 1, 0));
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

            removeRigidbodies();
            auto newRigidbody = Rigidbody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
            newRigidbody.m_rotation = Quat(0, 0, 90 / 180 * M_PI);
            m_rigidbodies.push_back(newRigidbody);
            m_externalForcePosition = Vec3(0.3, 0.5, 0.25);
            m_externalForce = Vec3(1, 1, 0);
            simulateTimestep(2);

            cout << "Demo 1 results, position: " << m_rigidbodies[0].m_position << endl;
            cout << "cm velocity: " << m_rigidbodies[0].m_velocity << endl;
            cout << "cm ang. veloctiy: " << m_rigidbodies[0].m_angularVelocity << endl;
            cout << "cm velocity of  (0.3, 0.5, 0.25): " << m_rigidbodies[0].getVelocityOfPosition(Vec3(0.3, 0.5, 0.25), true);
            m_externalForce = Vec3();
            break;
        }
        case 1:
        {
            cout << "Demo 2: Simple single-body simulation" << endl;

            removeRigidbodies();
            std::mt19937 eng(time(nullptr));
            std::uniform_real_distribution<float> randSpeed(-0.5f, 0.5f);
            std::uniform_real_distribution<float> randPos(-3.0f, 3.0f);
            std::uniform_real_distribution<float> randSize(-0.5f, 0.5f);
            auto newRigidbody = Rigidbody(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(randSize(eng), randSize(eng), randSize(eng)), 2);
            newRigidbody.m_rotation = Quat(0, 0, 90 / 180 * M_PI);
            m_rigidbodies.push_back(newRigidbody);
            m_externalForcePosition = Vec3(0.3, 0.5, 0.25);
            m_externalForce = Vec3(0.5, 0.5, 0);
            break;
        }
        case 2:
        {
            cout << "Demo 2: Simple collision simulation" << endl;

        }
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    float c = 1;
    for (size_t i = 0; i < m_rigidbodies.size(); i++) {
        Mat4 worldPosMat = m_rigidbodies[i].getlocalToWorldMat();
        for (size_t j = i + 1; j < m_rigidbodies.size(); j++) {
            CollisionInfo collision = checkCollisionSAT(worldPosMat, m_rigidbodies[j].getlocalToWorldMat());
            if (collision.isValid) {
                auto rigidbody_a = m_rigidbodies[i];
                auto rigidbody_b = m_rigidbodies[j];
                auto x_a = collision.collisionPointWorld - rigidbody_a.m_position;
                auto x_b = collision.collisionPointWorld - rigidbody_b.m_position;
                auto v_tild_a = rigidbody_a.getVelocityOfPosition(x_a);
                auto v_tild_b = rigidbody_b.getVelocityOfPosition(x_b);
                auto v_rel = v_tild_b - v_tild_a;
                auto j = -(1 + c) * dot(v_rel, collision.normalWorld) / (1/rigidbody_a.m_mass + 1/rigidbody_b.m_mass + dot(
                    cross(rigidbody_a.getIntertiaTensor().transformVector(cross(x_a, collision.normalWorld)), x_a) + 
                    cross(rigidbody_b.getIntertiaTensor().transformVector(cross(x_b, collision.normalWorld)), x_b), collision.normalWorld)
                    );
                rigidbody_a.m_velocity += j / rigidbody_a.m_mass;
                rigidbody_b.m_velocity -= j / rigidbody_b.m_mass;
                rigidbody_a.m_angularMomentum = rigidbody_a.m_angularMomentum + cross(x_a, j * collision.normalWorld);
                rigidbody_b.m_angularMomentum = rigidbody_b.m_angularMomentum - cross(x_b, j * collision.normalWorld);
            }
        }
    }


    for (auto& rigidbody : m_rigidbodies) {
        //positional stuff
        rigidbody.m_position += timeStep * rigidbody.m_velocity;
        rigidbody.m_velocity += timeStep * m_externalForce / rigidbody.m_mass;
        //rotational stuff
        Vec3 externalForceDiff = m_externalForcePosition - rigidbody.m_position;
        Vec3 torque = cross(externalForceDiff, m_externalForce);
        rigidbody.m_rotation += (timeStep / 2) *
            Quat(0, rigidbody.m_angularVelocity.x, rigidbody.m_angularVelocity.y, rigidbody.m_angularVelocity.z) * rigidbody.m_rotation;
        rigidbody.m_rotation = rigidbody.m_rotation.unit();
        rigidbody.m_angularMomentum += timeStep * torque;
        auto rotMat = rigidbody.m_rotation.getRotMat();
        auto inverseRotMat = rigidbody.m_rotation.getRotMat();
        inverseRotMat.transpose();
        auto newI = (rotMat * rigidbody.m_initialIntertiaTensor) * inverseRotMat;
        rigidbody.m_angularVelocity = newI * rigidbody.m_angularMomentum;
    }


    if (m_iTestCase == 0) {
        //cout << "Demo 1 results, position: " << m_rigidbodies[0].m_position << endl;
        //cout << "cm velocity: " << m_rigidbodies[0].m_velocity << endl;
        //cout << "cm ang. veloctiy: " << m_rigidbodies[0].m_angularVelocity << endl;
        //cout << "cm velocity of  (0.3, 0.5, 0.25): " << m_rigidbodies[0].getVelocityOfPosition(Vec3(0.3, 0.5, 0.25), true);
    }
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
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
    //m_rigidbodies[i].m_;
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

void RigidBodySystemSimulator::removeRigidbodies()
{
    m_rigidbodies.clear();
    m_rigidbodies.shrink_to_fit();
}

Vec3 Rigidbody::getVelocityOfPosition(Vec3 pos, bool positionInObjectSpace)
{
    if (positionInObjectSpace) {
        pos = getPositionAfterRotation(pos);
    }
    return getVelocityOfPosition(pos);
}

Vec3 Rigidbody::getVelocityOfPosition(Vec3 pos) {
    return m_velocity + cross(m_angularVelocity, pos);
}

Vec3 Rigidbody::getPositionAfterRotation(Vec3 initialPos)
{
    return m_rotation.getRotMat().transformVector(initialPos);
}

Vec3 Rigidbody::getWorldPositionOfPoint(Vec3 point)
{
    return m_position + getPositionAfterRotation(point);
}

Mat4 Rigidbody::getIntertiaTensor()
{
    auto rotMat = m_rotation.getRotMat();
    auto inverseRotMat = m_rotation.getRotMat();
    inverseRotMat.transpose();
    auto newI = (rotMat * m_initialIntertiaTensor) * inverseRotMat;
    return newI;
}

Mat4 Rigidbody::getlocalToWorldMat()
{
    auto scaleMatrix = GamePhysics::Mat4d();
    scaleMatrix.initScaling(m_size.x, m_size.y, m_size.z);
    auto translationMatrix = GamePhysics::Mat4d();
    translationMatrix.initTranslation(m_position.x, m_position.y, m_position.z);
    return scaleMatrix * m_rotation.getRotMat() * translationMatrix;
}
