#include "Point.h"


class Point {

public:
	Point() {}
	Point(Vec3 position, Vec3 velocity, bool isFixed) {
		this->velocity = velocity;
		this->position = position;
		this->isFixed = isFixed;
	}

	void setDamping(float dampingFactor) {
		force = -velocity * dampingFactor;
	}

	void addSpringForce(Vec3 force) {
		force += force;
	}

	void movePoint(float timeStep) {
		if (isFixed) return;
		position += velocity * timeStep;
	}

	void setSpeed(Vec3 externalForce, float mass, float timeStep) {
		if (isFixed) return;
		velocity += ((force + externalForce) / mass) * timeStep;
	}


private:
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;

};
