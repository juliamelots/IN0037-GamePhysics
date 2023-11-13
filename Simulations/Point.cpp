#include "Point.h"
#include <iostream>

Point::Point(Vec3 position, Vec3 velocity, bool isFixed, Vec3 color) {
	this->velocity = velocity;
	this->forceVelocity = velocity;
	this->position = position;
	this->forcePosition = position;
	this->isFixed = isFixed;
	this->color = color;
}

void Point::setDamping(float dampingFactor) {
	force = -forceVelocity * dampingFactor;
}

void Point::addSpringForce(Vec3 force) {
	this->force += force;
	//std::cout << "force: " << this->force << std::endl;
}

void Point::movePoint(float timeStep, bool forcePositionOnly) {
	if (isFixed) return;
	if (forcePositionOnly) {
		forcePosition += velocity * timeStep;
	}
	else {
		position += velocity * timeStep;
		forcePosition = position;
	}
	//std::cout << "position: " << position << std::endl;
}

void Point::setSpeed(Vec3 externalForce, float mass, float timeStep, bool forceVelocityOnly) {
	if (isFixed) return;
	if (forceVelocityOnly) {
		forceVelocity += ((force + externalForce) / mass) * timeStep;
	}
	else {
		velocity += ((force + externalForce) / mass) * timeStep;
		forceVelocity = velocity;
	}
	//std::cout << "velocity: " << velocity << std::endl;
}

void Point::setExternalForce(Vec3 force) {
	if (isFixed) return;
	force += force;
}
