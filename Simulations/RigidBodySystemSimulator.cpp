#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_externalForce = Vec3();
    m_externalForcePosition = Vec3();
    m_fCoefRestitution = 0.5;
    m_fGravity = 9.8;
    m_fDamping = 0.8;
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
    for (RigidBody& rigidBody : m_rigidBodies) {
        DUC->setUpLighting(rigidBody.m_position, 0.4 * Vec3(1, 1, 1), 100, Vec3(randColor(eng), randColor(eng), randColor(eng)));
        Mat4 drawMat = rigidBody.getlocalToWorldMat();
        DUC->drawRigidBody(drawMat);
    }

    DUC->beginLine();
    for (Spring& spring : m_vSprings)
    {
        DUC->drawLine(m_rigidBodies.at(spring.bodyIdx1).m_position, Vec3(0.5f, 0.5f, 0.5f),
            m_rigidBodies.at(spring.bodyIdx2).m_position, Vec3(0.5f, 0.5f, 0.5f));
    }
    DUC->drawLine(Vec3(-1, 0, 0), Vec3(0.5f, 0.5f, 0.5f),
        Vec3(1, 0, 0), Vec3(0.5f, 0.5f, 0.5f));
    DUC->endLine();
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
    {
        cout << "Demo 1: A simple one-step test" << endl;

        addRigidBody(Vec3(), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(0, Quat(0.0, 0.0, 0.5 * M_PI));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
        simulateTimestep(2);

        cout << "Demo 1 results:\n\tPosition: " << m_rigidBodies[0].m_position << endl;
        cout << "\tCM Linear velocity: " << m_rigidBodies[0].m_linearVelocity << endl;
        cout << "\tCM Angular veloctiy: " << m_rigidBodies[0].m_angularVelocity << endl;
        cout << "\tCM World velocity of  (0.3, 0.5, 0.25): " << m_rigidBodies[0].getVelocityOfPosition(Vec3(0.3, 0.5, 0.25)) << endl;
        break;
    }
    case 1:
    {
        cout << "Demo 3: Simple collision simulation" << endl;
        removeSprings();
        removeRigidBodies();
        addRigidBody(Vec3(), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(0, Quat(0.0, 0.0, 0.5 * M_PI));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
        simulateTimestep(0.01);
        break;
    }
    case 2:
    {
        cout << "Demo 3: Simple collision simulation" << endl;

        removeRigidBodies();
        addRigidBody(Vec3(0.0, 0.5, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(0, Quat(0.0, 0.0, 0.0));
        setVelocityOf(0, Vec3(0.0, 1.0, 0.0));

        addRigidBody(Vec3(0.0, 2.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(1, Quat(30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0));
        setVelocityOf(1, Vec3(0.0, -1.0, 0.0));
        
        removeSprings();
        break;
    }
    case 3:
    {
        cout << "Demo 4: Complex simulation" << endl;
        removeSprings();
        removeRigidBodies();

        addRigidBody(Vec3(0.0, 0.5, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(0, Quat(0.0, 0.0, 0.0));
        setVelocityOf(0, Vec3(0.0, 0.1f, 0.0));

        addRigidBody(Vec3(0.0, 2.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(1, Quat(30.0 / 180.0 * M_PI, 30.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0));
        setVelocityOf(1, Vec3(0.0, -0.1f, 0.0));

        addRigidBody(Vec3(0.0, 3.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(2, Quat(35.0 / 180.0 * M_PI, 35.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI, 0));
        setVelocityOf(2, Vec3(0.0, -0.2f, 0.0));

        addRigidBody(Vec3(0.0, 4.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
        setOrientationOf(3, Quat(40.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI, 0));
        setVelocityOf(3, Vec3(0.0, -0.3f, 0.0));

        addSpring(0, 1, 1.0, 1.0);
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
        float inputScale = 0.005f;
        inputForce = inputWorld * inputScale;
        for (RigidBody& rigidBody : m_rigidBodies)
            rigidBody.m_torqueExternalForce = cross((Vec3(m_trackmouse.x, m_trackmouse.y, 0) * inputScale - rigidBody.m_position), inputForce);
    }
    else
        for (RigidBody& rigidBody : m_rigidBodies)
            rigidBody.m_torqueExternalForce = Vec3();

    // Gravity
    Vec3 gravityForce = Vec3();
    gravityForce = Vec3(0, -m_fGravity, 0);
    // if (m_iTestCase == 3)
        // gravityForce = Vec3(0, -m_fGravity, 0);

    m_externalForce = gravityForce + inputForce;
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    // Collision handling
    for (size_t i = 0; i < m_rigidBodies.size(); i++)
    {
        RigidBody& rigidBody_a = m_rigidBodies.at(i);
        Mat4 worldPosMat = rigidBody_a.getlocalToWorldMat();
        rigidBody_a.m_internalForce = Vec3();
        // Collision between rigid bodies
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
                    //cout << "before velocity" << rigidBody_a.m_linearVelocity;
                    rigidBody_a.m_linearVelocity += (impulse / rigidBody_a.m_mass) * collision.normalWorld;
                    //cout << "after velocity" << rigidBody_a.m_linearVelocity;
                    rigidBody_b.m_linearVelocity -= (impulse / rigidBody_b.m_mass) * collision.normalWorld;
                    rigidBody_a.m_angularMomentum += cross(x_a, impulse * collision.normalWorld);
                    rigidBody_b.m_angularMomentum -= cross(x_b, impulse * collision.normalWorld);
                }
            }
        }

        // Ground collision
        CollisionInfo collision = checkCollisionSAT(worldPosMat, m_ground.getlocalToWorldMat());
        if (collision.isValid)
        {
            Vec3 x_a = collision.collisionPointWorld - rigidBody_a.m_position;
            Vec3 x_b = collision.collisionPointWorld - Vec3(0.0, -1.0, 0.0);
            Vec3 v_rel = rigidBody_a.getVelocityOfPosition(x_a); // the velocity of the m_ground collision point is (0, 0, 0)
            // cout << collision.normalWorld << endl;
            if (dot(v_rel, collision.normalWorld) < 0)
            {
                cout << "Collision" << endl;
                cout << collision.collisionPointWorld << endl;
                // rigidBody_a.m_isCollision = true;
                float impulse = -(1 + m_fCoefRestitution) * dot(v_rel, collision.normalWorld) /
                    (1 / rigidBody_a.m_mass + dot(
                        cross(rigidBody_a.getInverseIntertiaTensor().transformVector(cross(x_a, collision.normalWorld)), x_a)
                        , collision.normalWorld));
                //cout << "before velocity" << rigidBody_a.m_linearVelocity << endl;
                rigidBody_a.m_linearVelocity += (impulse / rigidBody_a.m_mass) * collision.normalWorld;
                //cout << "after velocity" << rigidBody_a.m_linearVelocity << endl;
                rigidBody_a.m_angularMomentum += cross(x_a, impulse * collision.normalWorld);
            }
        }
    }

    for (Spring& spring : m_vSprings) applyInternalForce(spring);

    for (RigidBody& rigidBody : m_rigidBodies) {
        // Position update
        
        if (!rigidBody.m_isCollision) {
            rigidBody.m_oldPosition = rigidBody.m_position;
            rigidBody.m_position += timeStep * rigidBody.m_linearVelocity;
        }
        else { 
            rigidBody.m_position = rigidBody.m_oldPosition; 
        }
        rigidBody.m_translationMatrix.initTranslation(rigidBody.m_position.x, rigidBody.m_position.y, rigidBody.m_position.z);
        
        // Linear velocity update
        Vec3 aggregatedInternalForce = rigidBody.m_internalForce - m_fDamping * rigidBody.m_linearVelocity;
        // Vec3 aggregatedInternalForce = rigidBody.m_internalForce *(1 - m_fDamping * dot(rigidBody.m_linearVelocity, rigidBody.m_internalForce) / (norm(rigidBody.m_internalForce) * norm(rigidBody.m_internalForce)));
        if (!(rigidBody.m_internalForce.x == 0 && rigidBody.m_internalForce.y == 0 && rigidBody.m_internalForce.z == 0)) {
            cout << rigidBody.m_internalForce << endl;
        }
        rigidBody.m_linearVelocity += timeStep * (m_externalForce + aggregatedInternalForce) / rigidBody.m_mass;

        // Angular velocity and rotation update
        rigidBody.m_rotation += (timeStep / 2.0) * Quat(rigidBody.m_angularVelocity.x, rigidBody.m_angularVelocity.y, rigidBody.m_angularVelocity.z, 0) * rigidBody.m_rotation;
        rigidBody.m_rotation = rigidBody.m_rotation.unit();
        rigidBody.m_angularMomentum += timeStep * rigidBody.m_torqueExternalForce;
        Mat4 rotMat = rigidBody.m_rotation.getRotMat();
        Mat4 inverseRotMat = rigidBody.m_rotation.getRotMat();
        inverseRotMat.transpose();
        Mat4 inverseInertiaTensor = inverseRotMat * rigidBody.m_initialInverseIntertiaTensor * rotMat;
        rigidBody.m_angularVelocity = inverseInertiaTensor.transformVector(rigidBody.m_angularMomentum);
        //cout << "linear velocity: " << rigidBody.m_linearVelocity << endl;
        //cout << "angular velocity: " << rigidBody.m_angularVelocity << endl;
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
    rigidBody.m_torqueExternalForce += cross((loc - rigidBody.m_position), force);
    m_externalForce += force;
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

void RigidBodySystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}


void RigidBodySystemSimulator::addSpring(int index1, int index2, float initialLength, float stiffness)
{
    m_vSprings.push_back(Spring(index1, index2, initialLength, stiffness));
}

int RigidBodySystemSimulator::getNumberOfSprings()
{
    return m_vSprings.size();
}

void RigidBodySystemSimulator::removeSprings() {
    m_vSprings.clear();
    m_vSprings.shrink_to_fit();
}

void RigidBodySystemSimulator::addRandomSprings(int number)
{
    std::mt19937 eng(time(nullptr));
    for (int i = 0; i < number; i++) {
        std::uniform_int_distribution<> randPoint(0, getNumberOfRigidBodies() - 1);
        std::uniform_real_distribution<float> randLength(-0.5f, 0.5f);
        std::uniform_real_distribution<float> randStiffness(0.01f, 0.5f);
        int point1Index = randPoint(eng);
        int point2Index = randPoint(eng);
        float distance = norm(m_rigidBodies.at(point1Index).m_position - m_rigidBodies.at(point2Index).m_position) + randLength(eng);
        addSpring(point1Index, point2Index, distance, randStiffness(eng));
    }
}

//--------------------------------------------------------------------------------------
// Check collision of mass points and (maybe) the floor
//--------------------------------------------------------------------------------------

void RigidBodySystemSimulator::applyInternalForce(Spring spring)
{
    float currentLengthOfString = getCurrentLength(spring);
    float springTerm = -spring.m_fStiffness
        * (currentLengthOfString - spring.m_fInitialLength)
        / currentLengthOfString;
    m_rigidBodies.at(spring.bodyIdx1).m_internalForce += springTerm * (m_rigidBodies.at(spring.bodyIdx1).m_position - m_rigidBodies.at(spring.bodyIdx2).m_position);
    m_rigidBodies.at(spring.bodyIdx2).m_internalForce += springTerm * (m_rigidBodies.at(spring.bodyIdx2).m_position - m_rigidBodies.at(spring.bodyIdx1).m_position);
}

float RigidBodySystemSimulator::getCurrentLength(Spring spring) {
    return norm(m_rigidBodies.at(spring.bodyIdx1).m_position - m_rigidBodies.at(spring.bodyIdx2).m_position);
};

Vec3 RigidBodySystemSimulator::calculateNewPosition(Vec3 position, Vec3 velocity, float timeStep)
{
    return position + timeStep * velocity;
}

//--------------------------------------------------------------------------------------
// Calculate new velocity after time step using accerelation as derivative
//--------------------------------------------------------------------------------------
Vec3 RigidBodySystemSimulator::calculateNewVelocity(Vec3 velocity, Vec3 internalForce, float timeStep, float mass)
{
    Vec3 acceleration = (internalForce + Vec3(0, -m_fGravity, 0) - m_fDamping * velocity) / mass;
    return velocity + timeStep * acceleration;
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