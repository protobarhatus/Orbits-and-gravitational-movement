#include "Space.h"
#include <iostream>
Space::Space()
{
	this->integration_method = tableForRK4();
}

void Space::setIntegrationMethod(const ButchersTable& table)
{
	this->integration_method = table;
}
static bool equal(long double a, long double b)
{
	return abs(a - b) < 0.0000001;
}
void Space::prepareForSimulation()
{
	int amount_of_objects = this->objects.size();
	this->integrating_functions = std::vector<std::function<Vector(long double, const std::vector<Vector>&)>>(2*amount_of_objects);
	long double G = this->G;
	for (int i = 0; i < amount_of_objects; ++i)
	{
		if (!this->objects[i].movable())
		{
			integrating_functions[2 * i] = integrating_functions[2 * i + 1] = [](long double, const std::vector<Vector>& objects) {return Vector::create(objects[0].size()); };
			continue;
		}
		integrating_functions[2 * i] = [amount_of_objects, i](long double t, const std::vector<Vector>& objects) ->Vector {
			//as x_i' = f_i(t, x1, .., xn), so f_i = v_i
			return objects[2 * i + 1];
		};

		std::vector<Object>* obj = &this->objects;;
		integrating_functions[2 * i + 1] = [amount_of_objects, i, G, obj](long double t, const std::vector<Vector>& objects) -> Vector {
			Vector force = Vector::create(objects[0].size());
			for (int j = 0; j < amount_of_objects; ++j)
			{
				if (i == j)
					continue;
				Vector r = objects[2 * j] - objects[2 * i];
				long double r_l = length(r);
				if (equal(r_l, 0))
				{
					std::cout << "Objects collided!\n";
					throw "Objects collided\n";
				}
				force += G * ((*obj)[j].getMass()) * r / (r_l * r_l * r_l);
			}
			return force;
		};
	}

}

int Space::getAmountOfObjects() const
{
	return this->objects.size();
}

Vector Space::getObjectPosition(int index) const
{
	return this->objects_params[index * 2];
}

void Space::updateObjectsParams()
{
	for (int i = 0; i < this->objects.size(); ++i)
	{
		this->objects[i].setPosition(this->objects_params[i * 2]);
		this->objects[i].setVelocity(this->objects_params[i * 2 + 1]);
	}
}

void Space::integrateStep(double step)
{
	this->objects_params = ButcherRkForSystem(this->integration_method, this->objects_params, this->time, step, this->integrating_functions);
}

void Space::addPlanet(const Object& obj)
{
	this->objects.push_back(obj);
	this->objects_params.push_back(obj.getPosition());
	this->objects_params.push_back(obj.getVelocity());
}

long double Space::getTime() const
{
	return this->time;
}
