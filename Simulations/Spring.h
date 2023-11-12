#include <cmath>
#include <iostream>
#include "util/vectorbase.h"
#include "DrawingUtilitiesClass.h"

class Spring{
public:
	Spring(int point1, int point2, float initialLength);

public:
	int point1;
	int point2;
	float initialLength;
	float currentLength;
};

