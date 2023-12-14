#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
public:
	std::vector<std::vector<float>> t_space;
	int nx;
	int ny;
	int nz;
	float x_diff_squared;
};

class HSVColor {
public:
	HSVColor()=default;
	explicit HSVColor(nVec3i rgbValues);
	explicit HSVColor(float h, float s, float v);
	nVec3i hsv;
	Vec3 toRGB();
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);
	int get_A_index(int i, int j, int k);
	std::tuple<int, int, int> get_space_index(int a_index);
	bool isInBorder(int space_i, int space_j, int space_k);
	Vec3 colorLerp(float rel_t);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_alpha;
	Grid T;
	HSVColor m_coldColor;
	HSVColor m_hotColor;
	float min_temp = 0;
	float max_temp = 500;
};

#endif