#include "Object.h"
#include <functional>
#include <assert.h>
#include "Space.h"
#include "EventedAction.h"
Object::Object(double mass, const Vector& pos, const Vector& vel, const Vector& acc, bool movable) :
	mass(mass), position(pos), velocity(vel), acceleration(acc)
{

}


Object::Object()
{

}

void Object::setOrbitPtr(Orbit* orbit)
{
	this->orbit = orbit;
}

void Object::setSpacePtr(Space* space)
{
	this->space = space;
}

const Vector& Object::getPosition() const
{
	return position;
}

const Vector& Object::getVelocity() const
{
	return velocity;
}

const Vector& Object::getAcceleration() const
{
	return acceleration;
}

double Object::getMass() const
{
	return this->mass;
}

void Object::setPosition(const Vector& position)
{
	this->position = position;
}

void Object::setVelocity(const Vector& velocity)
{
	this->velocity = velocity;
}

void Object::setAcceleration(const Vector& acceleration)
{
	this->acceleration = acceleration;
}
#include <iostream>
void Object::executeHohmannManeur(long double r2)
{
	//assert(this->orbit->getType() == Orbit::CIRCLE);

	long double r1 = length(this->getPosition());
	
	long double r = r1 / r2;

	long double V1 = length(this->getVelocity());

	std::cout << "Hohmann: first impuls\n";
	std::cout << "Current velocity: " << round(V1 * 100) / 100 << "\n";

	this->velocity *= sqrt(2 / (1 + r));

	std::cout << "Delta-V: " << round((length(this->velocity) - V1) * 100) / 100 << "\n";

	this->orbit->orbitChanged();

	Orbit* orb = this->orbit;
	
	long double t1 = this->space->getTime();

	Space* sp = space;

	this->space->addEvent(EventedAction(r < 1 ? EventedAction::APHELION : EventedAction::PERIHELION, [this, orb, V1, r, sp, t1]()->bool {
		std::cout << "Hohmann: second impuls\n";
		long double cur_V = length(this->getVelocity());
		std::cout << "Current velocity: " << round(cur_V * 100) / 100 << "\n";
		this->setVelocity(this->getVelocity() + (this->getVelocity() / length(this->getVelocity())) * V1 * sqrt(r) * (1 - sqrt(2 * r / (1 + r))));
		std::cout << "Delta-V: " << round((length(this->getVelocity()) - cur_V) * 100) / 100 << "\n\n";
		//this->orbit->setUpdateOrbitEveryStep(false);
		this->orbit->orbitChanged();

		long double t = sp->getTime() - t1;
		std::cout << "Time spent on maneur: " << round(t * 100) / 100 << "\n\n";
		return true;
		}));
	/*this->space->addEvent(distanceReachedEvent(r2, [this, orb, V1, r, sp, t1]()->bool {
		std::cout << "Hohmann: second impuls\n";
		long double cur_V = length(this->getVelocity());
		std::cout << "Current velocity: " << round(cur_V * 100) / 100 << "\n";
		this->setVelocity(this->getVelocity() + (this->getVelocity() / length(this->getVelocity())) * V1 * sqrt(r) * (1 - sqrt(2 * r / (1 + r))));
		std::cout << "Delta-V: " << round((length(this->getVelocity()) - cur_V) * 100) / 100 << "\n\n";
		//this->orbit->setUpdateOrbitEveryStep(false);
		this->orbit->orbitChanged();

		long double t = sp->getTime() - t1;
		std::cout << "Time spent on maneur: " << round(t * 100) / 100 << "\n\n";
		return true;
		}));*/

}

void Object::executeBiellipticManeur(long double r, long double b)
{
	long double V = length(this->getVelocity());
	long double DV1 = V * (sqrt(2 - 2 / (1 + b)) - 1);
	long double DV2 = V * (sqrt(2 / b - 2 / (b + r)) - sqrt(2 / b - 2 / (b + 1)));
	long double DV3 = -V * (sqrt(2 / r - 2 / (b + r)) - sqrt(1 / r));

	std::cout << "Bi-elliptic: first impuls\n";
	std::cout << "Current velocity: " << round(V * 100) / 100 << "\n";
	std::cout << "Delta-V: " << round(DV1 * 100) / 100 << "\n\n";

	this->velocity += (this->velocity / length(this->velocity)) * DV1;

	this->orbit->orbitChanged();

	Orbit* orb = this->orbit;

	Space* space = this->space;
	this->space->addEvent(EventedAction((b > 1 ? EventedAction::APHELION : EventedAction::PERIHELION), [DV2, DV3, orb, space, this, b, r]()->bool {
		long double cur_V = length(this->getVelocity());
		std::cout << "Bi-elliptic: second impuls\n";
		std::cout << "Current velocity: " << round(cur_V * 100) / 100 << "\n";
		std::cout << "Delta-V: " << round(DV2 * 100) / 100 << "\n\n";

		this->setVelocity(this->getVelocity() + this->getVelocity() / cur_V * DV2);
		orb->orbitChanged();

		space->addEvent(EventedAction((b > r ? EventedAction::PERIHELION : EventedAction::APHELION), [DV3, orb, space, this]()->bool {
			long double cur_V = length(this->getVelocity());
			std::cout << "Bi-elliptic: third impuls\n";
			std::cout << "Current velocity: " << round(cur_V * 100) / 100 << "\n";
			std::cout << "Delta-V: " << round(DV3 * 100) / 100 << "\n\n";

			this->setVelocity(this->getVelocity() + this->getVelocity() / cur_V * DV3);
			orb->orbitChanged();
			return true;
			}));
		return true;
		}));
}

void Object::executeDoublePulseManeur(long double r, long double dv_to_v)
{
	long double V1 = length(this->getVelocity());
	long double DV = dv_to_v * V1;
	long double r1 = length(this->getPosition());
	long double r2 = r * r1;

	Vector vV1 = this->getVelocity();
	Vector vr1 = this->getPosition();

	std::cout << "Double-pulse maneur: first impuls\n";
	std::cout << "Current velocity: " << round(V1 * 100) / 100 << "\n";
	std::cout << "Delta-V: " << round(DV * 100) / 100 << "\n\n";

	this->velocity *= (1 + dv_to_v);

	this->orbit->orbitChanged();

	Orbit* orb = this->orbit;

	long double t1 = space->getTime();

	Space* sp = this->space;

	this->space->addEvent(distanceReachedEvent(r2, [V1, DV, r1, r2, vr1, vV1, this, r, dv_to_v, orb, t1, sp]()->bool {
		std::cout << "Double-pulse maneur: second impuls\n";
		long double U = length(this->getVelocity());
		std::cout << "Current velocity: " << U << "\n";

		Vector vU = this->getVelocity();

		Vector inm1 = addDimension(vr1) * addDimension(vV1 * (1 + dv_to_v)) / r1 / (V1 + DV);

		Vector O = (addDimension(vr1) * addDimension(vV1 * (1 + dv_to_v))  / r1  / (V1 + DV)/ r1 * V1 / pow(r, 1.5));
		Vector dv2 = takeAwayDimension(O * addDimension(this->getPosition())) - vU;

		std::cout << "Delta-V: " << round(length(dv2) * 100) / 100 << "\n\n";
		this->setVelocity(vU + dv2);

		orb->orbitChanged();

		long double t2 = sp->getTime();

		std::cout << "Time spent on maneur: " << round((t2 - t1) * 100) / 100 << "\n\n";

		return true;
		}));
	//
}

Object getEarth(const Vector& position)
{
	//return Object(5.9722e24, position, position.null(), position.null(), false);
	return Object(1000, position, position.null(), position.null(), false);
}
