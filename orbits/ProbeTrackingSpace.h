#pragma once
#include <vector>
#include "Object.h"
#include "ButchersTable.h"
#include "Orbit.h"
#include <list>
#include "EventedAction.h"
#include "Space.h"
/*In this class, there is a highlighted object called probe. Class will keep track of it movement, make orbit, call events, etc. Also, probe have its engines so should update
object_params of probe every integration step*/
class ProbeTrackingSpace : public Space
{
public:
	ProbeTrackingSpace();

	virtual void integrateStep(double step);


	void setProbe(const Object& probe);

	const Object& getProbe() const;
	const std::vector<Object>& getPlanets() const;

	Orbit& getProbesOrbit() ;

	void addEvent(const EventedAction& ev);


	Object* getProbePtr();

	int getProbeIndex() const;
	
private:

	
	void sendEvent(EventedAction::Event ev);

	//this is for orbital maneuver program. Although this may be activated in any program - and space will count orbit, but with three or more bodies there is almost never
	//a stable eliptic orbit, so effect will be chaotic
	Object* probe;
	Orbit probes_orbit;
	int points_to_orbit_counter = 0;
	int points_to_add_to_orbit_interval = 10;
	int probe_index;


	long double meas_angle;

	std::list<EventedAction> subscribed_event;
};

