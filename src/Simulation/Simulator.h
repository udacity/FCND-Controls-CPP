#pragma once

#include <vector>

using namespace std;

class BaseDynamics;

class Simulator
{
public:
	Simulator();

	void Reset();

	void Run(float dt);

	void AddVehicle(shared_ptr<BaseDynamics> vehicle);

	vector<shared_ptr<BaseDynamics> > _vehicles;
	float _simTime;
};