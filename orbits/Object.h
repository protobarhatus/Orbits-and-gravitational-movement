#pragma once
#include "Math and data works/Linear algebra and 3d geometry/Vector.h"
#include "Orbit.h"
class Space;
class Object
{
public:
	Object(double mass, const Vector& pos, const Vector& vel, const Vector& acct, bool movable = true);
	Object();
	
	void setOrbitPtr(Orbit* orbit);
	void setSpacePtr(Space* space);
	
	const Vector& getPosition() const;
	const Vector& getVelocity() const;
	const Vector& getAcceleration() const;

	double getMass() const;

	void setPosition(const Vector& position);
	void setVelocity(const Vector& velocity);
	void setAcceleration(const Vector& acceleration);

	void executeHohmannManeur(long double r2);
	void executeBiellipticManeur(long double r, long double b);

	void executeDoublePulseManeur(long double r2, long double dv_to_v);
private:

	Space* space;
	Orbit* orbit;

	double mass;
	Vector position;
	Vector velocity;
	Vector acceleration;

	bool is_movable = true;

};

Object getEarth(const Vector& position);

