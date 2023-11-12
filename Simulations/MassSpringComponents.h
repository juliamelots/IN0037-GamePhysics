#ifndef MASSSPRINGCOMPONENTS_h
#define MASSSPRINGCOMPONENTS_h
#include "Simulator.h"
#include <vector>

class MassPoint {
public:
	// Construtors
	//MassPoint();
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

//MassPoint::MassPoint()
//{
//	m_position = Vec3();
//	m_velocity = Vec3();
//	m_internalForce = Vec3();
//	m_bIsFixed = false;
//}

//MassPoint::MassPoint(Vec3 position, Vec3 velocity, bool isFixed)
//{
//	m_position = position;
//	m_velocity = velocity;
//	m_internalForce = Vec3();
//	m_bIsFixed = isFixed;
//}

class Spring {
public:
	//Spring();
	Spring(int point1, int point2, float initialLength)
	{
		m_iPoint1 = point1;
		m_iPoint2 = point2;
		m_fInitialLength = m_fCurrentLength = initialLength;
	};

	// Data Attributes
	int m_iPoint1;
	int m_iPoint2;
	float m_fInitialLength;
	float m_fCurrentLength;
};

//Spring::Spring()
//{
//	m_iPoint1 = m_iPoint2 = 0;
//	m_fInitialLength = m_fCurrentLength = 0.0;
//}

//Spring::Spring(int point1, int point2, float initialLength)
//{
//	m_iPoint1 = point1;
//	m_iPoint2 = point2;
//	m_fInitialLength = m_fCurrentLength = initialLength;
//}

#endif