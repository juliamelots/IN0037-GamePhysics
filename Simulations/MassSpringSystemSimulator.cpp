#include "MassSpringSystemSimulator.h"

class MassSpringSystemSimulator : public Simulator {


	void setMass(float mass) {
		m_fMass = mass;
	}

	void setStiffness(float stiffness) {
		m_fStiffness = stiffness;
	}

	void setDampingFactor(float damping) {
		m_fDamping = damping;
	}

	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
		Point newPoint = Point(position, Velocity, isFixed);
		points.push_back(newPoint);
	}

	void addSpring(int masspoint1, int masspoint2, float initialLength) {
		Spring newSpring = Spring(masspoint1, masspoint2, initialLength);
	}

	int getNumberOfMassPoints() {
		return positions.size();
	}

	int getNumberOfSprings() {
		return springs.size();
	}

	Vec3 getPositionOfMassPoint(int index) {
		return points[index].getPosition();
	}

	Vec3 getVelocityOfMassPoint(int index) {
		return points[index].getVelocity();
	}

	void applyExternalForce(Vec3 force) {

	}

	void simulateTimeStep(float timeStep) {
		//Calculate current force on all Points
		SetForces();
		//Move every point
		if (m_iIntegrator == EULER) {
			for (Point point : points) {
				point.movePoint(timeStep);
				point.setSpeed(m_externalForce, m_fMass, timeStep);
			}
		}
		else if (m_iIntegrator == MIDPOINT) {
			var halfTimeStep = timeStep / 2;
			for (Point point : points) {
				point.movePoint(halfTimeStep);
				point.setSpeed(m_externalForce, m_fMass, halfTimeStep);
			}
			SetForces();
			for (Point point : points) {
				point.movePoint(halfTimeStep);
				point.setSpeed(m_externalForce, m_fMass, halfTimeStep);
			}
		}
		else if (m_iIntegrator == LEAPFROG) {
			for (Point point : points) {
				point.setSpeed(m_externalForce, m_fMass, timeStep);
				point.movePoint(timeStep);
			}
		}
	}

	void SetForces() {
		for (Point point : points) {
			point.setDamping(m_fDamping);
		}
		for (Spring spring : springs) {
			Point firstPoint = points[spring.point1];
			Point secondPoint = points[spring.point2];
			Vec3 delta = firstPoint.getPosition() - secondPoint.getPosition();
			float springSize = normalize(delta);
			float springForce = (springSize - spring.initialLength) * m_fStiffness;
			secondPoint.addSpringForce(springForce * delta);
			firstPoint.addSpringForce((-springForce) * delta);
		}
	}


private:
	float length(Vec3 vector) {
		return 
	}


private:
	// Data Attributes 
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	std::vector<Point> points;
	std::vector<Spring> springs;


	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};