#pragma once
#include <functional>
#include "Math and data works/Linear algebra and 3d geometry/Vector.h"
class EventedAction
{
public:
	enum Event
	{
		APHELION,
		PERIHELION,
		DISTANCE
	};

	EventedAction(Event awaited_event, const std::function<bool()>& action);
	bool notify(Event event_type, const std::vector<long double> & params);
	void setDistanceParam(long double dist);
private:
	Event awaited_event;
	//true is event disappears, false is event stays
	std::function<bool ()> action;

	long double dist_param;
};

EventedAction distanceReachedEvent(long double distance, const std::function<bool()>& action);