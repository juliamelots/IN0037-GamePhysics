#ifndef MASSSPRINGSYSTEMCOMPONENTS_h
#define MASSSPRINGSYSTEMCOMPONENTS_h
#include "Simulator.h"
#include <vector>

class MassPoint {
public:
	// Construtors
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed);

	// Specific Functions
	void setPosition(Vec3 position);
	void setVelocity(Vec3 velocity);
	Vec3 getPosition();
	Vec3 getVelocity();
	bool getIsFixed();

private:
	// Data Attributes
	Vec3 m_position;
	Vec3 m_velocity;
	bool m_bIsFixed;
};

void MassPoint::setPosition(Vec3 position)
{
	m_position = position;
}

void MassPoint::setVelocity(Vec3 velocity)
{
	m_velocity = velocity;
}

Vec3 MassPoint::getPosition()
{
	return m_position;
}

Vec3 MassPoint::getVelocity()
{
	return m_velocity;
}

bool MassPoint::getIsFixed()
{
	return m_bIsFixed;
}

class Spring {
public:
	Spring(MassPoint point1, MassPoint point2, float initialLength);

	// Specific Functions
	void addToLength(float displacement);
	vector<MassPoint*> getMassPoints();
	float getInitialLength();
	float getCurrentLength();

private:
	// Data Attributes
	MassPoint m_point1;
	MassPoint m_point2;
	float m_fInitialLength;
	float m_fCurrentLength;
};

void Spring::addToLength(float displacement)
{
	m_fCurrentLength += displacement;
}

vector<MassPoint*> Spring::getMassPoints()
{
	vector<MassPoint*> endpoints{ &m_point1, &m_point2 };
	return endpoints;
}

float Spring::getInitialLength()
{
	return m_fInitialLength;
}

float Spring::getCurrentLength()
{
	return m_fCurrentLength;
}

#endif