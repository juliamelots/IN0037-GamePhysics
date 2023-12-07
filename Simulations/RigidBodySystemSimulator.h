#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#include <vector>

#define TESTCASEUSEDTORUNTEST 2

Mat4 transpose3x3(Mat4 matrix)
{
	Mat4 trans = Mat4();
	for (int i = 0;i < 3;i++)
		for (int j = i + 1;j < 3;j++)
		{
			trans.value[i][j] = matrix.value[j][i];
			trans.value[j][i] = matrix.value[i][j];
		}
	return trans;
};

class RigidBody {
public:
	// Constructors
	RigidBody(Vec3 position, Vec3 size, int mass);

	// Functions
	Mat4 getObj2WorldMatrix();

	// Data Attributes
	Mat4 m_scale;
	int m_iMass;
	Vec3 m_position;
	Quat m_orientation;
	Vec3 m_linearVelocity;
	Vec3 m_angularVelocity;
	Mat4 m_rotationMatrix;
	Mat4 m_initInertiaTensor;
	Mat4 m_invInitInertiaTensor;
	Mat4 m_invInertiaTensor;
	Vec3 m_angularMomentum;
	Vec3 m_torqueByExternalForce;
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Extra Functions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	Vec3 localToWorldPosition(int i, Vec3 localPoint);
	Vec3 localToWorldVelocity(int i, Vec3 localPoint);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem;
	vector<RigidBody *> m_vRigidBodies;
	float m_fCoefRestitution;
	float m_fGravity;
	bool m_bActiveFloor;
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

RigidBody::RigidBody(Vec3 position, Vec3 size, int mass)
{
	// reference for all values is center of mass
	m_scale = Mat4();
	m_scale.initScaling(size.x, size.y, size.z);
	m_iMass = mass;
	m_position = position;
	m_orientation = Quat();
	m_linearVelocity = Vec3();
	m_angularVelocity = Vec3();
	m_rotationMatrix = m_orientation.getRotMat();
	m_initInertiaTensor = Mat4();
	m_initInertiaTensor.initScaling(mass * (size.y * size.y + size.z * size.z) / 12.0,
		mass * (size.x * size.x + size.y * size.y) / 12.0,
		mass * (size.x * size.x + size.z * size.z) / 12.0);
	m_invInitInertiaTensor = m_initInertiaTensor.inverse();
	m_invInertiaTensor = transpose3x3(m_rotationMatrix) * m_invInitInertiaTensor * m_rotationMatrix;
	m_angularMomentum = m_initInertiaTensor.transformVector(m_angularVelocity);
	m_torqueByExternalForce = Vec3();
}

Mat4 RigidBody::getObj2WorldMatrix()
{
	Mat4 translate = Mat4();
	translate.initTranslation(m_position.x, m_position.y, m_position.z);
	return m_scale * m_rotationMatrix * translate;
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	m_fCoefRestitution = 1.0;
	m_fGravity = 9.8;
	m_bActiveFloor = false;
	m_externalForce = 0.0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2, Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	if (m_iTestCase == 3)
	{
		TwAddVarRW(DUC->g_pTweakBar, "Coefficient of Restitution", TW_TYPE_FLOAT, &m_fCoefRestitution, "step=0.1 min=0.0 max=1.0");
		TwAddVarRW(DUC->g_pTweakBar, "Activate Floor", TW_TYPE_BOOLCPP, &m_bActiveFloor, "");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.1 min=0.0");
	}
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vRigidBodies.clear();
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1: simple one-step test\n";
		addRigidBody(Vec3(), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), 0.5 * M_PI));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		simulateTimestep(2.0);
		cout << "Linear velocity: " << getLinearVelocityOfRigidBody(0) << "\n";
		cout << "Angular velocity: " << getAngularVelocityOfRigidBody(0) << "\n";
		cout << "World velocity: " << localToWorldVelocity(0, Vec3(-0.3, -0.5, -0.25)) << "\n";
		break;
	case 1:
		cout << "Demo 2: simple single-body simulation\n";
		break;
	case 2:
		cout << "Demo 3: two-rigid-body collision simulation\n";
		break;
	case 3:
		cout << "Demo 4: complex simulation\n";
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (RigidBody * body : m_vRigidBodies)
	{
		body->m_position += timeStep * body->m_linearVelocity;
		body->m_linearVelocity += timeStep * m_externalForce / body->m_iMass;
		// transforms angular velocity W into quaternion (W,0)T to allow multiplication
		body->m_orientation += (timeStep / 2) * Quat(body->m_angularVelocity, M_PI) * body->m_orientation;
		body->m_rotationMatrix = body->m_orientation.getRotMat();
		body->m_angularMomentum += timeStep * body->m_torqueByExternalForce;
		body->m_invInertiaTensor = transpose3x3(body->m_rotationMatrix) *
			body->m_invInitInertiaTensor *
			body->m_rotationMatrix;
		body->m_angularVelocity = body->m_invInertiaTensor.transformVector(body->m_angularMomentum);
	}
	for (RigidBody * bodyA : m_vRigidBodies)
	{
		for (RigidBody* bodyB : m_vRigidBodies)
		{
			if (bodyA == bodyB) continue;
			Mat4 bodyA2World = bodyA->getObj2WorldMatrix();
			Mat4 bodyB2World = bodyB->getObj2WorldMatrix();
			CollisionInfo info = checkCollisionSAT(bodyA2World, bodyB2World);
			if (info.isValid)
			{
				Vec3 vRel = bodyA->m_linearVelocity + cross(bodyA->m_angularVelocity, bodyA->m_position)
							- bodyB->m_linearVelocity - cross(bodyB->m_angularVelocity, bodyB->m_position);
				if (vRel > Vec3())
				{
					Vec3 impulse = -(1 + m_fCoefRestitution) * dot(vRel, info.normalWorld);
					impulse /= (1 / bodyA->m_iMass) + (1 / bodyB->m_iMass)
						+ dot((cross(bodyA->m_invInertiaTensor.transformVector(cross(bodyA->m_position, info.normalWorld)), bodyA->m_position)
						+ cross(bodyB->m_invInertiaTensor.transformVector(cross(bodyB->m_position, info.normalWorld)), bodyB->m_position)),
						info.normalWorld);
						bodyA->m_linearVelocity += impulse * info.normalWorld / bodyA->m_iMass;
						bodyB->m_linearVelocity -= impulse * info.normalWorld / bodyB->m_iMass;
						bodyA->m_angularMomentum += cross(bodyA->m_position, impulse * info.normalWorld);
						bodyB->m_angularMomentum -= cross(bodyB->m_position, impulse * info.normalWorld);
				}
			}
		}
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (RigidBody * body : m_vRigidBodies)
	{
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		Mat4 translate = Mat4();
		translate.initTranslation(body->m_position.x, body->m_position.y, body->m_position.z);
		DUC->drawRigidBody(body->getObj2WorldMatrix());
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
	return m_vRigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	return rigidBody->m_position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	return rigidBody->m_linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	return rigidBody->m_angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	m_externalForce += force;
	rigidBody->m_torqueByExternalForce = (loc - rigidBody->m_position) * force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody* rigidBody = new RigidBody(position, size, mass);
	m_vRigidBodies.push_back(rigidBody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	rigidBody->m_orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	rigidBody->m_linearVelocity = velocity;
}

Vec3 RigidBodySystemSimulator::localToWorldPosition(int i, Vec3 localPoint)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	return rigidBody->m_position + rigidBody->m_rotationMatrix.transformVector(localPoint);
}

Vec3 RigidBodySystemSimulator::localToWorldVelocity(int i, Vec3 localPoint)
{
	RigidBody* rigidBody = m_vRigidBodies.at(i);
	return rigidBody->m_linearVelocity + cross(rigidBody->m_angularVelocity, localPoint);
}
#endif