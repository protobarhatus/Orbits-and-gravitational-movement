#pragma once
#include <vector>
#include "Object.h"
#include "ButchersTable.h"
#include <list>

class Space
{

public:
	Space();
	void setIntegrationMethod(const ButchersTable& table);
	virtual void integrateStep(double step);

	void addPlanet(const Object& obj);

	long double getTime() const;

	void prepareForSimulation();

	int getAmountOfObjects() const;
	Vector getObjectPosition(int index) const;
	//is not necessary
	void updateObjectsParams();
protected:

	long double time = 0;

	ButchersTable integration_method;

	std::vector<Object> objects;
	std::vector<Vector> objects_params;

	std::vector<std::function<Vector(long double, const std::vector<Vector>&)>> integrating_functions;

	const long double G = 1;

};

