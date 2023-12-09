#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
    m_externalForcePosition = Vec3();
    m_fCoefRestitution = 1.0;
    m_fGravity = 9.8;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo1: One-Step, Demo2: Single-body, Demo3: Two-body, Demo4: Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    if (m_iTestCase == 3)
    {
        TwAddVarRW(DUC->g_pTweakBar, "Coefficient of Restitution", TW_TYPE_FLOAT, &m_fCoefRestitution, "step=0.1 min=0.0 max=1.0");
        TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.1 min=0.0");
    }
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

        removeRigidBodies();
        auto newRigidbody = RigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody.m_rotation = Quat(0, 0, 90.0 / 180.0 * M_PI);
        m_rigidBodies.push_back(newRigidbody);
        m_externalForcePosition = Vec3(0.3, 0.5, 0.25);
        m_externalForce = Vec3(1, 1, 0);
        simulateTimestep(2);

        cout << "Demo 1 results, position: " << m_rigidBodies[0].m_position << endl;
        cout << "cm velocity: " << m_rigidBodies[0].m_linearVelocity << endl;
        cout << "cm ang. veloctiy: " << m_rigidBodies[0].m_angularVelocity << endl;
        cout << "cm velocity of  (0.3, 0.5, 0.25): " << m_rigidBodies[0].getVelocityOfPosition(Vec3(0.3, 0.5, 0.25)) << endl;
        m_externalForce = Vec3();
        break;
    }
    case 1:
    {
        cout << "Demo 2: Simple single-body simulation" << endl;

        removeRigidBodies();
        std::mt19937 eng(time(nullptr));
        std::uniform_real_distribution<float> randSpeed(-0.5f, 0.5f);
        std::uniform_real_distribution<float> randPos(-3.0f, 3.0f);
        std::uniform_real_distribution<float> randSize(0.1f, 0.5f);
        auto newRigidbody = RigidBody(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(randSize(eng), randSize(eng), randSize(eng)), 2);
        newRigidbody.m_rotation = Quat(0, 0, 90 / 180 * M_PI);
        m_rigidBodies.push_back(newRigidbody);
        m_externalForcePosition = Vec3(0.3, 0.5, 0.25);
        m_externalForce = Vec3(0.5, 0.5, 0);
        break;
    }
    case 2:
    {
        cout << "Demo 3: Simple collision simulation" << endl;
        removeRigidBodies();
        auto newRigidbody = RigidBody(Vec3(0, 0.5, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody.m_rotation = Quat(0, 0, 0);
        newRigidbody.m_linearVelocity = Vec3(0, 0.1f, 0);
        m_rigidBodies.push_back(newRigidbody);

        auto newRigidbody2 = RigidBody(Vec3(0, 2, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody2.m_linearVelocity = Vec3(0, -0.1f, 0);
        newRigidbody2.m_rotation = Quat(30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0);
        m_rigidBodies.push_back(newRigidbody2);
        break;
    }
    case 3:
    {
        cout << "Demo 4: Complex simulation" << endl;
        removeRigidBodies();

        auto newRigidbody = RigidBody(Vec3(0, 0.5, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody.m_rotation = Quat(0, 0, 0);
        newRigidbody.m_linearVelocity = Vec3(0, 0.1f, 0);
        m_rigidBodies.push_back(newRigidbody);

        auto newRigidbody2 = RigidBody(Vec3(0, 2, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody2.m_linearVelocity = Vec3(0, -0.1f, 0);
        newRigidbody2.m_rotation = Quat(30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0);
        m_rigidBodies.push_back(newRigidbody2);

        auto newRigidbody3 = RigidBody(Vec3(0, 3, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody3.m_linearVelocity = Vec3(0, -0.2f, 0);
        newRigidbody3.m_rotation = Quat(35.0 / 180.0 * M_PI, 35.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI, 0);
        m_rigidBodies.push_back(newRigidbody3);

        auto newRigidbody4 = RigidBody(Vec3(0, 4, 0), Vec3(1, 0.6, 0.5), 2);
        newRigidbody4.m_linearVelocity = Vec3(0, -0.3f, 0);
        newRigidbody4.m_rotation = Quat(40.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0);
        m_rigidBodies.push_back(newRigidbody4);
        break;
    }
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
    // User interaction force
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    Vec3 inputForce = Vec3();
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        float inputScale = 0.00001f;
        inputForce = inputWorld * inputScale;
    
        // Applying to all bodies
        for (size_t i = 0; i < m_rigidBodies.size(); i++)
            applyForceOnBody(i, Vec3(m_trackmouse.x, m_trackmouse.y, 0), inputForce);
    }

    // Gravity
    m_externalForce = Vec3(0, -m_fGravity, 0) + inputForce;

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    externalForcesCalculations(timeStep);

    // update position
    for (RigidBody& rigidBody : m_rigidBodies) {
        rigidBody.m_oldPosition = rigidBody.m_position;
        rigidBody.m_position += timeStep * rigidBody.m_linearVelocity;
    }

    RigidBody& ground = RigidBody(Vec3(0, 0, 0), Vec3(10000000000, 0.0000001, 10000000000), 10000000000);

    //collision
    for (size_t i = 0; i < m_rigidBodies.size(); i++)
    {
        RigidBody& rigidBody_a = m_rigidBodies.at(i);
        Mat4 worldPosMat = rigidBody_a.getlocalToWorldMat();

        // collision between rigid bodies
        for (size_t j = 0; j < m_rigidBodies.size(); j++)
        {
            if (j == i) continue;
            RigidBody& rigidBody_b = m_rigidBodies.at(j);
            CollisionInfo collision = checkCollisionSAT(worldPosMat, rigidBody_b.getlocalToWorldMat());
            if (collision.isValid)
            {
                Vec3 x_a = collision.collisionPointWorld - rigidBody_a.m_position;
                Vec3 x_b = collision.collisionPointWorld - rigidBody_b.m_position;
                Vec3 v_rel = rigidBody_a.getVelocityOfPosition(x_a) - rigidBody_b.getVelocityOfPosition(x_b);
                // cout << collision.normalWorld << endl;
                if (dot(v_rel, collision.normalWorld) < 0)
                {
                    float impulse = -(1 + m_fCoefRestitution) * dot(v_rel, collision.normalWorld) /
                        (1 / rigidBody_a.m_mass + 1 / rigidBody_b.m_mass + dot(
                            cross(rigidBody_a.getInverseIntertiaTensor().transformVector(cross(x_a, collision.normalWorld)), x_a) +
                            cross(rigidBody_b.getInverseIntertiaTensor().transformVector(cross(x_b, collision.normalWorld)), x_b)
                            , collision.normalWorld));
                    cout << "before velocity" << rigidBody_a.m_linearVelocity;
                    rigidBody_a.m_linearVelocity += (impulse / rigidBody_a.m_mass) * collision.normalWorld;
                    cout << "after velocity" << rigidBody_a.m_linearVelocity;
                    rigidBody_b.m_linearVelocity -= (impulse / rigidBody_b.m_mass) * collision.normalWorld;
                    cout << endl;
                    rigidBody_a.m_angularMomentum += cross(x_a, impulse * collision.normalWorld);
                    rigidBody_b.m_angularMomentum -= cross(x_b, impulse * collision.normalWorld);
                    cout << "body collide" << endl;
                }
            }
        }

        CollisionInfo collision = checkCollisionSAT(ground.getlocalToWorldMat(), rigidBody_a.getlocalToWorldMat());
        if (collision.isValid)
        {
            Vec3 x_a = collision.collisionPointWorld - rigidBody_a.m_position;
            Vec3 x_b = collision.collisionPointWorld - ground.m_position;
            Vec3 v_rel = rigidBody_a.getVelocityOfPosition(x_a); // the velocity of the ground collision point is (0, 0, 0)
            // cout << collision.normalWorld << endl;
            if (dot(v_rel, collision.normalWorld) < 0) {
                float impulse = -(1 + m_fCoefRestitution) * dot(v_rel, collision.normalWorld) /
                    (1 / rigidBody_a.m_mass + dot(
                        cross(rigidBody_a.getInverseIntertiaTensor().transformVector(cross(x_a, collision.normalWorld)), x_a)
                        , collision.normalWorld));
                cout << "before velocity" << rigidBody_a.m_linearVelocity << endl;
                rigidBody_a.m_linearVelocity += (impulse / rigidBody_a.m_mass) * collision.normalWorld;
                cout << "velocity delta" << (impulse / rigidBody_a.m_mass) * collision.normalWorld << endl; 
                cout << "after velocity" << rigidBody_a.m_linearVelocity << endl;
                rigidBody_a.m_angularMomentum += cross(x_a, impulse * collision.normalWorld);
                cout << " ground collide" << endl;
                rigidBody_a.restoreOldPosition = true;
            }
        }
    }

    for (RigidBody& rigidBody : m_rigidBodies) {
        if (rigidBody.restoreOldPosition) rigidBody.m_position = rigidBody.m_oldPosition;
        rigidBody.m_translationMatrix.initTranslation(rigidBody.m_position.x, rigidBody.m_position.y, rigidBody.m_position.z);

        // Linear velocity update
        rigidBody.m_linearVelocity += timeStep * m_externalForce / rigidBody.m_mass;

        // Angular velocity and rotation update
        rigidBody.m_rotation += (timeStep / 2.0) * Quat(rigidBody.m_angularVelocity.x, rigidBody.m_angularVelocity.y, rigidBody.m_angularVelocity.z, 0) * rigidBody.m_rotation;
        rigidBody.m_rotation = rigidBody.m_rotation.unit();
        rigidBody.m_angularMomentum += timeStep * rigidBody.m_torqueExternalForce;
        Mat4 rotMat = rigidBody.m_rotation.getRotMat();
        Mat4 inverseRotMat = rigidBody.m_rotation.getRotMat();
        inverseRotMat.transpose();
        Mat4 inverseInertiaTensor = inverseRotMat * rigidBody.m_initialInverseIntertiaTensor * rotMat;
        rigidBody.m_angularVelocity = inverseInertiaTensor.transformVector(rigidBody.m_angularMomentum);
        cout << "linear velocity: " << rigidBody.m_linearVelocity << endl;
        cout << "angular velocity: " << rigidBody.m_angularVelocity << endl;
    }
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    return rigidBody.m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    return rigidBody.m_linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    return rigidBody.m_angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    rigidBody.m_torqueExternalForce = cross((loc - rigidBody.m_position), force);
    // nothing added to m_externalForce because it's not applied on whole system
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    m_rigidBodies.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    rigidBody.m_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    RigidBody& rigidBody = m_rigidBodies.at(i);
    rigidBody.m_linearVelocity = velocity;
}

void RigidBodySystemSimulator::removeRigidBodies()
{
    m_rigidBodies.clear();
    m_rigidBodies.shrink_to_fit();
}

Vec3 RigidBody::getVelocityOfPosition(Vec3 pos)
{
	return m_linearVelocity + cross(m_angularVelocity, pos);
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