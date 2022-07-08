#pragma once
#include "Math and data works/Linear algebra and 3d geometry/Vector.h"
#include "Orbit.h"
class ProbeTrackingSpace;
class Object
{
public:
	Object(double mass, const Vector& pos, const Vector& vel, sf::Color color, int pix_rad, bool movable = true);
	Object();
	
	void setOrbitPtr(Orbit* orbit);
	void setSpacePtr(ProbeTrackingSpace* space);
	
	const Vector& getPosition() const;
	const Vector& getVelocity() const;
	const Vector& getAcceleration() const;

	double getMass() const;

	void setPosition(const Vector& position);
	void setVelocity(const Vector& velocity);
	void setAcceleration(const Vector& acceleration);

	void executeHohmannManeuver(long double r2);
	void executeBiellipticManeuver(long double r, long double b);

	void executeDoublePulseManeuver(long double r2, long double dv_to_v);

	bool movable() const;

	sf::Color getColor() const;
	int getRadius() const;
private:

	ProbeTrackingSpace* space;
	Orbit* orbit;

	double mass;
	Vector position;
	Vector velocity;
	Vector acceleration;

	bool is_movable = true;
	sf::Color color;
	int radius;

};

Object getEarth(const Vector& position);

