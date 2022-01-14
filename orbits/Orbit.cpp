#include "Orbit.h"
#include <assert.h>
Orbit::Orbit() : equation_coefficients_from_points(points_for_computation, 6), transposed_equation_coefficients_from_points(6, points_for_computation)
{
	this->graphical.setPrimitiveType(sf::LineStrip);
	min_velocity = Vector(1e100, 1e100);
	point_of_min_velocity;
}
//long double divcoe = 1000000;
long double divcoe = 1;
void Orbit::addPoint(const Vector& point, const Vector& velocity)
{


	this->equation_coefficients_from_points[this->current_point] = Vector(point.x() * point.x() / divcoe / divcoe,
		point.x() * point.y() * 2 / divcoe / divcoe,
		point.y() * point.y() / divcoe / divcoe,
		point.x() * 2 / divcoe,
		point.y() * 2 / divcoe);
	

	this->transposed_equation_coefficients_from_points[0][this->current_point] = point.x() * point.x() / divcoe / divcoe;
	this->transposed_equation_coefficients_from_points[1][this->current_point] = point.x() * point.y() * 2 / divcoe / divcoe;
	this->transposed_equation_coefficients_from_points[2][this->current_point] = point.y() * point.y() / divcoe / divcoe;
	this->transposed_equation_coefficients_from_points[3][this->current_point] = point.x() * 2 / divcoe;
	this->transposed_equation_coefficients_from_points[4][this->current_point] = point.y() * 2 / divcoe;


	++this->current_point;
	if (this->current_point == this->points_for_computation)
		this->completed_at_least_one_cycle = true;
	if ((this->completed_at_least_one_cycle && this->update_orbit_every_step) || this->current_point == this->points_for_computation)
		this->computeOrbit(velocity);
	if (this->current_point == this->points_for_computation)
		this->current_point = 0;
		
	if (length(velocity) < length(this->min_velocity))
	{
		point_of_min_velocity = point;
		min_velocity = velocity;
	}

	if (this->in_trace_mode)
	{
		this->traject_points.push_back(point);
		this->last_vertex = point;
	}
}

void Orbit::setInterestingSpaceWidth(int width)
{
	this->intresting_space_width = width;
}

void Orbit::setOrbit(double a, double b, double c, double d, double e, double f)
{
	this->orbits_params = Vector(a, b / 2, c, d / 2, e / 2, f);
	this->defineOrbitType();
	this->countOrbitsParams();
}

const std::vector<Vector> Orbit::trajectoryPoints() const
{
	return this->traject_points;
}

bool Orbit::defined() const
{
	return this->completed_at_least_one_cycle;
}

const Vector& Orbit::gravitationalCentre() const
{
	return *this->gravitation_centre;
}

const Vector& Orbit::getAphelion() const
{
	return aphelion;
}

const Vector& Orbit::getPerihelion() const
{
	return this->perihelion;
}

Orbit::Type Orbit::getType() const
{
	return this->orbits_type;
}

long double Orbit::radius() const
{
	assert(this->orbits_type == CIRCLE);
	return length(this->aphelion - *this->gravitation_centre);
}

void Orbit::orbitChanged()
{
	this->in_trace_mode = true;
	this->current_point = 0;
	this->completed_at_least_one_cycle = false;
	this->traject_points.clear();
	this->graphical.clear();
	this->min_velocity = Vector(1e100, 1e100);
}

bool Orbit::hasChanged() const
{
	return this->changed;
}

const sf::VertexArray& Orbit::graphArray() const
{
	return this->graphical;
}

void Orbit::setGraphArray(sf::VertexArray&& arr)
{
	this->graphical = std::move(arr);
	this->changed = false;
	this->need_to_redraw = false;
}

bool Orbit::inTraceMode() const
{
	return this->in_trace_mode;
}

Vector Orbit::lastVertex() const
{
	return last_vertex;
}

void Orbit::addToGraphArray(const sf::Vertex& v)
{
	this->graphical.append(v);
}

void Orbit::needToRedraw()
{
	this->need_to_redraw = true;
	this->graphical.clear();
}

bool Orbit::isNeedToRedraw() const
{
	return this->need_to_redraw;
}



void Orbit::setPointsForComputation(int value)
{
	this->points_for_computation = value;
	equation_coefficients_from_points = Matrix(points_for_computation, 5);
	transposed_equation_coefficients_from_points = Matrix(5, points_for_computation);
	this->completed_at_least_one_cycle = false;
}

void Orbit::setUpdateOrbitEveryStep(bool update)
{
	this->update_orbit_every_step = update;
}

static bool equal(long double a, long double b)
{
	return abs(a - b) / abs(a + b) < 0.001;
}

void Orbit::defineOrbitType()
{
	//Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0
	if (equal(A(), 0) && equal(B(), 0) && equal(C(), 0))
	{
		this->orbits_type = LINE;
		countOrbitsParams();
		return;
	}
	long double det = A() * C() - B() * B();
	if (equal(det, 0))
	{
		this->orbits_type = PARABOLA;
		countOrbitsParams();
		return;
	}
	if (det > 0)
	{
		if (equal(B(), 0) && equal(A(), C()))
			this->orbits_type = CIRCLE;
		else
			this->orbits_type = ELLIPSE;
		countOrbitsParams();
		return;
	}
	this->orbits_type = HYPERBOLA;
}
#include <math.h>
#include <iostream>
void Orbit::computeOrbit(const Vector& velocity)
{
	this->in_trace_mode = false;
	this->last_vertex = Vector::create(0);

	Vector b = Vector::create(this->points_for_computation);
	long double SH = 1;
	//иначе все будет просто нулями, т к к фигуре 0 == 0 все точки подходят
	//... + F - SH + SH = 0
	//... == SH
	//F = r - SH
	for (int i = 0; i < this->points_for_computation; ++i)
		//b[i] = 1e12 / divcoe / divcoe;
		b[i] = SH / divcoe / divcoe;;
	Vector new_orbits_params = inverse(transposed_equation_coefficients_from_points * equation_coefficients_from_points) * transposed_equation_coefficients_from_points * b;
	//this->orbits_params = inverse(equation_coefficients_from_points) * b;

	//Matrix check = equation_coefficients_from_points * inverse(equation_coefficients_from_points);
	//Vector check2 = equation_coefficients_from_points * this->orbits_params;
	bool chang = orbits_params.size() == 0;
	for (int i = 0; i < orbits_params.size(); ++i)
	{
		if (!equal(new_orbits_params[i], this->orbits_params[i]))
		{
			chang = true;
			break;
		}
	}
	if (!chang)
	{
		this->changed = false;
		return;
	}
	this->changed = true;
	this->orbits_params = std::move(new_orbits_params);
	
	long double error, error2 = 0;
	for (int i = 0; i < this->equation_coefficients_from_points.lines(); ++i)
		error2 += scalar(this->equation_coefficients_from_points[i], this->orbits_params);
	
	error = length(this->equation_coefficients_from_points * this->orbits_params - b);
	//std::cout << error << "\n";

	//this->orbits_params[5] -= 1e12 / divcoe / divcoe;


	this->orbits_params[3] *= divcoe;
	this->orbits_params[4] *= divcoe;


	this->defineOrbitType();
	countOrbitsParams();

	


}

static long double square(long double x)
{
	return x * x;
}

void Orbit::countOrbitsParams()
{
	this->fully_counted = true;

	if (this->orbits_type == LINE)
	{
		if (equal(E(), 0))
			this->traject_points = { Vector(-F() / 2 / D(), -this->intresting_space_width), Vector(-F() / 2 / D(), this->intresting_space_width) };
		else
			this->traject_points = { Vector(-this->intresting_space_width, (-F() + 2 * D() * intresting_space_width) / 2 / E()),
									Vector(this->intresting_space_width, (-F() - 2 * D() * intresting_space_width) / 2 / E()) };
	}

	long double phi;
	if (!equal(A() - C(), 0))
		phi = atan(2 * B() / (A() - C())) / 2;
	else //pi / 4
		phi = 0.7853981;

	long double cos_phi = cos(phi);
	long double sin_phi = sin(phi);

	long double canon_A, canon_C, precanon_D, precanon_E;

	canon_A = A() * square(cos_phi) + 2 * B() * cos_phi * sin_phi + C() * square(sin_phi);   //A Cos[phi] ^ 2 + 2 B Cos[phi] Sin[phi] + Cc Sin[phi] ^ 2
	canon_C = C() * square(cos_phi) - 2 * B() * cos_phi * sin_phi + A() * square(sin_phi);   //Cc Cos[phi]^2 - 2 B Cos[phi] Sin[phi] + A Sin[phi]^2
	precanon_D = D() * cos_phi + E() * sin_phi;              //(2 Dc Cos[phi] + 2 Ec Sin[phi] ) / 2
	precanon_E = E() * cos_phi - D() * sin_phi;              // (2 Ec Cos[phi] - 2 Dc Sin[phi]) / 2

	if (abs(canon_A) > abs(canon_C))
	{
		phi += 1.570796327;
		cos_phi = cos(phi);
		sin_phi = sin(phi);
		canon_A = A() * square(cos_phi) + 2 * B() * cos_phi * sin_phi + C() * square(sin_phi);   //A Cos[phi] ^ 2 + 2 B Cos[phi] Sin[phi] + Cc Sin[phi] ^ 2
		canon_C = C() * square(cos_phi) - 2 * B() * cos_phi * sin_phi + A() * square(sin_phi);   //Cc Cos[phi]^2 - 2 B Cos[phi] Sin[phi] + A Sin[phi]^2
		precanon_D = D() * cos_phi + E() * sin_phi;              //(2 Dc Cos[phi] + 2 Ec Sin[phi] ) / 2
		precanon_E = E() * cos_phi - D() * sin_phi;
	}


	this->inverse_rotation_to_canon_matrix = Matrix({ Vector{cos_phi, -sin_phi}, {sin_phi, cos_phi} });
	this->rotation_to_canon_matrix = Matrix({ Vector{cos_phi, sin_phi}, {-sin_phi, cos_phi} });

	this->shift_to_canon = -1*Vector(-precanon_D / canon_A, -precanon_E / canon_C);

	long double  canon_F = F() - square(D()) / A() - square(E()) / C();

	//now x^2 / a_squared + y^2 / b_squared
	long double  a_squared = -canon_F / canon_A;
	long double  b_squared = -canon_F / canon_C;

	if (this->orbits_type == ELLIPSE || this->orbits_type == CIRCLE)
	{
		assert(a_squared > 0 && b_squared > 0);
		long double  a = sqrt(a_squared);
		long double  b = sqrt(b_squared);
		this->eccentricity = sqrt(1 - b_squared / a_squared);
		long double  c = sqrt(a_squared) * this->eccentricity;
		this->F1 = (Vector(c, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;
		this->F2 = (Vector(-c, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;

		this->traject_points.clear();
		this->graphical.clear();

		for (int i = 0; i < 75; ++i)
		{
			long double  angle = double(i) / 75 * (6.28319);
			this->traject_points.push_back((Vector(a * cos(angle), b * sin(angle)) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix);
		}

		if (length(F1 - point_of_min_velocity) < length(F2 - point_of_min_velocity))
		{
			gravitation_centre = &F2;
			this->aphelion = (Vector(a, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;
			this->perihelion = (Vector(-a, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;
		}
		else
		{
			gravitation_centre = &F1;
			this->aphelion = (Vector(-a, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;
			this->perihelion = (Vector(a, 0) - this->shift_to_canon) * this->inverse_rotation_to_canon_matrix;
		}

		if (this->eccentricity < 0.02)
			this->orbits_type = CIRCLE;

	}

}


long double Orbit::A() const
{
	return this->orbits_params[0];
}
long double Orbit::B() const
{
	return this->orbits_params[1];
}
long double Orbit::C() const
{
	return this->orbits_params[2];
}
long double Orbit::D() const
{
	return this->orbits_params[3];
}
long double Orbit::E() const
{
	return this->orbits_params[4];
}
long double Orbit::F() const
{
	return -1;
}