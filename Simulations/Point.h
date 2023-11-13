#ifndef POINT_h
#define POINT_h
#include <cmath>
#include <iostream>
#include "util/vectorbase.h"
#include "DrawingUtilitiesClass.h"

class Point{

public:
	Point();
	Point(Vec3 position, Vec3 velocity, bool isFixed, Vec3 color);

	Vec3 getPosition() {
		return position;
	}

	Vec3 getVelocity() {
		return velocity;
	}

	Vec3 getForcePosition() {
		return forcePosition;
	}

	Vec3 getForceVelocity() {
		return forceVelocity;
	}

	Vec3 getColor() {
		return color;
	}

	void correctPosition(Vec3 position) {
		this->position = position;
		forcePosition = position;
	}

	void setDamping(float dampingFactor);

	void addSpringForce(Vec3 force);

	void movePoint(float timeStep, bool forcePositionOnly);

	void setSpeed(Vec3 externalForce, float mass, float timeStep, bool forceVelocityOnly);

	void setExternalForce(Vec3 force);

private: 
	float mass{};
	float dampingFactor{};
	Vec3 position;
	Vec3 forcePosition;
	Vec3 forceVelocity;
	Vec3 velocity;
	Vec3 force;
	Vec3 color;
	bool isFixed;

};
#endif // !POINT_h

