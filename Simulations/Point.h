#ifndef POINT_h
#define POINT_h
#include <cmath>
#include <iostream>
#include "util/vectorbase.h"
#include "DrawingUtilitiesClass.h"

class Point{

public:
	Point();
	Point(Vec3 position, Vec3 velocity, bool isFixed);

	Vec3 getPosition() {
		return position;
	}

	Vec3 getVelocity() {
		return velocity;
	}

	void correctPosition(Vec3 position) {
		this->position = position;
	}

	void setDamping(float dampingFactor);

	void addSpringForce(Vec3 force);

	void movePoint(float timeStep);

	void setSpeed(Vec3 externalForce, float mass, float timeStep);

private: 
	float mass{};
	float dampingFactor{};
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;

};
#endif // !POINT_h

