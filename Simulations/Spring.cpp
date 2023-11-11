#include "Spring.h"


class Spring {
	
public:
	Spring(int point1, int point2, float initialLength) {
		this->point1 = point1;
		this->point2 = point2;
		this->initialLength = initialLength;
	}

public:
	int point1;
	int point2;
	float initialLength;
	float currentLength;

};