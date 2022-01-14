#include "Space.h"
#include <functional>
#include <assert.h>
#include "Math and data works/Math.h"
Space::Space()
{
	this->integration_method = tableForRK6();
	this->probes_orbit.setInterestingSpaceWidth(600);
	this->probes_orbit.setPointsForComputation(30);
	this->points_to_add_to_orbit_interval = 20;
	this->meas_angle = 5;
	//this->probes_orbit.setUpdateOrbitEveryStep(true);
}

//y' = f(t, y)
Vector RK4(const Vector& old_value, double t, double step, const std::function<Vector(double, const Vector&)>& f)
{
	Vector k1 = f(t, old_value);
	Vector k2 = f(t + step / 2, old_value + (step / 2) * k1);
	Vector k3 = f(t + step / 2, old_value + (step / 2) * k2);
	Vector k4 = f(t + step, old_value + step * k3);

	return old_value + step / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}
//returns {V, X}
std::pair<Vector, Vector> integrateProbeByRK4(const Vector& position, const Vector& old_vel, double step, const std::function<Vector (const Vector &)> & force)
{
	Vector v_half = RK4(old_vel, 0, step / 2, [&force, &position](double t, const Vector& vel) { return force(position + t * vel); });
	Vector v_new = RK4(old_vel, 0, step, [&force, &position](double t, const Vector& vel) { return force(position + t * vel); });

	Vector x_new = RK4(position, 0, step, [&old_vel, &v_half, &v_new, step](double t, const Vector&)->Vector {
		if (abs(t) < step / 10)
			return old_vel;
		if (abs(t - step / 2) < step / 10)
			return v_half;
		if (abs(t - step) < step / 10)
			return v_new;
		assert(false);
		});

	return { std::move(v_new), std::move(x_new) };
}

std::pair<Vector, Vector> integrateProbeByButchersTable(const Vector& position, const Vector& velocity, double step, const ButchersTable & table, const std::function<Vector(const Vector&)>& force)
{
	//[0] - V' = f(t, {V, X}) = force(X)
	//[1] - X' = f(t, {V, X}) = V;

	auto res = ButcherRkForSystem(table, { velocity, position }, 0, step, {
		[&force](long double t, const std::vector<Vector>& arguments)->Vector
		{
			return force(arguments[1]);
},
		[](long double t, const std::vector<Vector>& arguments)->Vector {
			return arguments[0];
}
		});
	return { res[0], res[1] };
	
}

std::pair<Vector, Vector> naiveRK4Integrate(const Vector& position, const Vector& velocity, double step, const std::function<Vector(const Vector&)>& force)
{
	Vector k1v = force(position);
	Vector k1x = velocity;

	Vector k2v = force(position + step / 2 * k1x);
	Vector k2x = velocity + step / 2 * k1v;

	Vector k3v = force(position + step / 2 * k2x);
	Vector k3x = velocity + step / 2 * k2v;

	Vector k4v = force(position + step * k3x);
	Vector k4x = velocity + step * k3v;

	return { velocity + (k1v + 2 * k2v + 2 * k3v + k4v) / 6 * step, position + (k1x + 2 * k2x + 2 * k3x + k4x) / 6 * step};
	
}

std::pair<Vector, Vector> integrateProbeByInexplicitEuler(const Vector& position, const Vector& velocity, double step, const std::function<Vector(const Vector&)>& force)
{
	Vector new_v = velocity + force(position) * step;
	return { new_v, position + new_v * step };
}

static double cube(double a)
{
	return a * a * a;
}
#include <iostream>
void Space::integrateStep(double step)
{
	this->time += step;
	//just to pass to lambda
	const std::vector<Object>& immov_obj = this->immovable_objects;
	double probe_mass = probe.getMass();
	double G = this->G;

	Vector vel = probe.getVelocity();
	auto integr_res = integrateProbeByButchersTable(this->probe.getPosition(), this->probe.getVelocity(), step, this->integration_method, [immov_obj, probe_mass, G, vel](const Vector& pos)->Vector {
		Vector gravit_accel = pos.null();
		for (auto& it : immov_obj)
			gravit_accel += G * it.getMass() * (it.getPosition() - pos) / cube(length(pos - it.getPosition()));
		return gravit_accel;

	});
	/*auto integr_res = naiveRK4Integrate(this->probe.getPosition(), this->probe.getVelocity(), step, [immov_obj, probe_mass, G](const Vector& pos)->Vector {
		Vector gravit_accel = pos.null();
		for (auto& it : immov_obj)
			gravit_accel += G * it.getMass() * (it.getPosition() - pos) / cube(length(pos - it.getPosition()));
		return gravit_accel;
		});*/

	this->probe.setVelocity(std::move(integr_res.first));

	static Vector prev_V = this->probe.getVelocity();
	this->probe.setPosition(std::move(integr_res.second));
	
	if (angle(prev_V, this->probe.getVelocity()) >= meas_angle * 3.14159265358 / 180)
	{
		this->points_to_orbit_counter += this->points_to_add_to_orbit_interval;
		prev_V = this->probe.getVelocity();
	}
	if (this->points_to_orbit_counter == this->points_to_add_to_orbit_interval)
	{
		//std::cout << "Dist = " << length(this->probe.getPosition()) << "\n";
		//std::cout << "Vel = " << length(this->probe.getVelocity()) << "\n";
		this->points_to_orbit_counter = 0;
		this->probes_orbit.addPoint(this->probe.getPosition(), this->probe.getVelocity());
	}
	if (probes_orbit.defined())
	{

		//std::cout << abs(angle(this->probe.getPosition() - this->probes_orbit.gravitationalCentre(), this->probes_orbit.getAphelion() - this->probes_orbit.getPerihelion())) << "\n";
		if (angle(this->probe.getPosition() - this->probes_orbit.gravitationalCentre(), this->probes_orbit.getAphelion() - this->probes_orbit.getPerihelion())  < 0.001 &&
			length(this->probes_orbit.getAphelion() - this->probe.getPosition()) < length(this->probes_orbit.getPerihelion() - this->probe.getPosition()))
			sendEvent(EventedAction::APHELION);
		if (abs(angle(this->probe.getPosition() - this->probes_orbit.gravitationalCentre(), this->probes_orbit.getAphelion() - this->probes_orbit.getPerihelion()) - 3.1415926535)  < 0.02 &&
			length(this->probes_orbit.getPerihelion() - this->probe.getPosition()) < length(this->probes_orbit.getAphelion() - this->probe.getPosition()))
			sendEvent(EventedAction::PERIHELION);
		
		//if (abs(length(this->probe.getPosition()) - 200) < 1)
		//	sendEvent(EventedAction::APHELION);
	}
	sendEvent(EventedAction::DISTANCE);

}

Object* Space::getProbePtr()
{
	return &this->probe;
}

long double Space::getTime() const
{
	return this->time;
}

void Space::sendEvent(EventedAction::Event ev)
{
	for (auto it = this->subscribed_event.begin(); it != this->subscribed_event.end(); )
	{
		if (ev == EventedAction::DISTANCE)
		{
			long double len = length(this->probe.getPosition());
			if (it->notify(ev, { len }))
				it = this->subscribed_event.erase(it);
			else
				++it;
		}
		else
		{
			if (it->notify(ev, {  }))
				it = this->subscribed_event.erase(it);
			else
				++it;
		}
	}
}


void Space::addPlanet(const Object& obj)
{
	this->immovable_objects.push_back(obj);
}

void Space::setProbe(const Object& probe)
{
	this->probe = probe;
	this->probe.setOrbitPtr(&this->probes_orbit);
	this->probe.setSpacePtr(this);
}

const Object& Space::getProbe() const
{
	return this->probe;
}

const std::vector<Object>& Space::getPlanets() const
{
	return this->immovable_objects;
}

Orbit& Space::getProbesOrbit()
{
	return this->probes_orbit;
}

void Space::addEvent(const EventedAction& ev)
{
	this->subscribed_event.push_front(ev);
}
