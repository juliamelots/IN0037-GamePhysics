#ifndef MASSSPRINGSYSTEMCOMPONENTS_h
#define MASSSPRINGSYSTEMCOMPONENTS_h
#include "Simulator.h"
#include <vector>

class MassPoint {
public:
	// Construtors
	MassPoint();
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed);

	// Data Attributes
	Vec3 m_position;
	Vec3 m_velocity;
	bool m_bIsFixed;

};

MassPoint::MassPoint()
{
	m_position = Vec3();
	m_velocity = Vec3();
	m_bIsFixed = false;
}

MassPoint::MassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	m_position = position;
	m_velocity = velocity;
	m_bIsFixed = isFixed;
}

class Spring {
public:
	Spring();
	Spring(MassPoint point1, MassPoint point2, float initialLength);

	// Data Attributes
	MassPoint m_point1;
	MassPoint m_point2;
	float m_fInitialLength;
	float m_fCurrentLength;
};

Spring::Spring()
{
	m_point1 = MassPoint();
	m_point2 = MassPoint();
	m_fInitialLength = m_fCurrentLength = 0.0;
}

Spring::Spring(MassPoint point1, MassPoint point2, float initialLength)
{
	m_point1 = point1;
	m_point2 = point2;
	m_fInitialLength = m_fCurrentLength = initialLength;
}

#endif