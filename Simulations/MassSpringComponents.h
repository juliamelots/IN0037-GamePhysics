#ifndef MASSSPRINGCOMPONENTS_h
#define MASSSPRINGCOMPONENTS_h
#include "Simulator.h"
#include <vector>

class MassPoint {
public:
	// Constructors
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed)
	{
		m_position = position;
		m_velocity = velocity;
		m_internalForce = Vec3();
		m_bIsFixed = isFixed;
	};

	// Data Attributes
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_internalForce;
	vector<int> m_attachedSprings;
	bool m_bIsFixed;

};

class Spring {
public:
	// Constructors
	Spring(int point1, int point2, float initialLength)
	{
		m_iPoint1 = point1;
		m_iPoint2 = point2;
		m_fInitialLength = initialLength;
	};

	Spring(MassPoint* point1, MassPoint* point2, float initialLength)
	{
		m_point1 = point1;
		m_point2 = point2;
		m_fInitialLength = initialLength;
	};

	// Data Attributes
	int m_iPoint1;
	int m_iPoint2;
	MassPoint* m_point1;
	MassPoint* m_point2;
	float m_fInitialLength;

	// Functions
	float getCurrentLength() { return norm(m_point1->m_position - m_point2->m_position); };
};
#endif