#include "Point.h"
#include <iostream>

Point::Point(Vec3 position, Vec3 velocity, bool isFixed) {
	this->velocity = velocity;
	this->position = position;
	this->isFixed = isFixed;
}

void Point::setDamping(float dampingFactor) {
	force = -velocity * dampingFactor;
}

void Point::addSpringForce(Vec3 force) {
	this->force += force;
	std::cout << "force: " << this->force << std::endl;
}

void Point::movePoint(float timeStep) {
	if (isFixed) return;
	position += velocity * timeStep;
	std::cout << "position: " << position << std::endl;
}

void Point::setSpeed(Vec3 externalForce, float mass, float timeStep) {
	if (isFixed) return;
	velocity += ((force + externalForce) / mass) * timeStep;
	std::cout << "velocity: " << velocity << std::endl;

}
