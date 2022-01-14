#pragma once
#include <vector>
#include "Object.h"
#include "ButchersTable.h"
#include "Orbit.h"
#include <list>
#include "EventedAction.h"
class Space
{
public:
	Space();

	void integrateStep(double step);

	void addPlanet(const Object& obj);
	void setProbe(const Object& probe);

	const Object& getProbe() const;
	const std::vector<Object>& getPlanets() const;

	Orbit& getProbesOrbit() ;

	void addEvent(const EventedAction& ev);


	Object* getProbePtr();

	long double getTime() const;
private:

	
	void sendEvent(EventedAction::Event ev);

	long double time = 0;

	ButchersTable integration_method;

	Object probe;
	Orbit probes_orbit;
	int points_to_orbit_counter = 0;
	int points_to_add_to_orbit_interval = 10;
	std::vector<Object> immovable_objects;
	//const long double G = 6.6743015151515e-11;
	const long double G = 1;
	long double meas_angle;

	std::list<EventedAction> subscribed_event;
};

