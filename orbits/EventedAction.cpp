#include "EventedAction.h"

EventedAction::EventedAction(Event awaited_event, const std::function<bool()>& action)
{
	this->awaited_event = awaited_event;
	this->action = action;
}
static bool equal(const Vector& a, const Vector& b)
{
	for (int i = 0; i < a.size(); ++i)
	{
		if (abs(a[i] - b[i]) > 0.1)
			return false;
	}
	return true;
}

bool EventedAction::notify(Event event_type, const std::vector<long double> & params)
{
	if (event_type == this->awaited_event)
	{
		if (event_type == DISTANCE)
		{
			if (abs(params[0] - this->dist_param) <= 0.1)
				return this->action();
			return false;
		}
		else
			return this->action();
	}
	return false;
}

void EventedAction::setDistanceParam(long double dist)
{
	this->dist_param = dist;
}

EventedAction distanceReachedEvent(long double distance, const std::function<bool()>& action)
{
	EventedAction res(EventedAction::DISTANCE, action);
	res.setDistanceParam(distance);
	return res;
}
