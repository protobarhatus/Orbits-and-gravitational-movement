#pragma once
#include <vector>
#include "Math and data works/Linear algebra and 3d geometry/Vector.h"
#include "Math and data works/Linear algebra and 3d geometry/Matrix.h"
#include <Graphics.hpp>
class Orbit
{
public:
	enum Type
	{
		CIRCLE,
		ELLIPSE,
		HYPERBOLA,
		PARABOLA,
		LINE
	};
	Orbit();
	//�������� ������ � ����� ������ ��� ������ ��� ���������� �����
	void setPointsForComputation(int value);

	void setUpdateOrbitEveryStep(bool update);

	void addPoint(const Vector& point, const Vector& velocity);

	void setInterestingSpaceWidth(int width);
	//for debug
	void setOrbit(double a, double b, double c, double d, double e, double f);

	const std::vector<Vector> trajectoryPoints() const;

	bool defined() const;

	const Vector& gravitationalCentre() const;

	const Vector& getAphelion() const;
	const Vector& getPerihelion() const;

	Type getType() const;

	
	long double radius() const;

	void orbitChanged();

	bool hasChanged() const;
	const sf::VertexArray& graphArray() const;
	void setGraphArray(sf::VertexArray&& arr);

	bool inTraceMode() const;

	Vector lastVertex() const;
	void addToGraphArray(const sf::Vertex &v);

	void needToRedraw();
	bool isNeedToRedraw() const;
private:
	Vector min_velocity;
	Vector point_of_min_velocity;

	double intresting_space_width= -10;
	void defineOrbitType();
	void computeOrbit(const Vector& velocity);
	void countOrbitsParams();

	long double A() const;
	long double B() const;
	long double C() const;
	long double D() const;
	long double E() const;
	long double F() const;

	Matrix equation_coefficients_from_points;
	Matrix transposed_equation_coefficients_from_points;
	//��� �������� ��������: ������ �������, ��������� ������ � ������� ������� � ������
	int current_point = 0;
	//���������� �����, ��������� ��� ���������� ������. ����� �������������, ������ 6, ��, ��������� �������� � ������� � ��� �� �����, �� ��� ������ �����, ��� �����
	int points_for_computation = 10;
	//���� true, �� ��������� ������ �� ������ ���������� ����� (����� ���������� ���������� points_for_computation). ���� false, �� ������ ������ points_for_computations �����
	bool update_orbit_every_step = false;
	//������������ � ��������� ����� ������� �������
	//Ax^2 + 2Bxy + Cy^2 + 2Dx + 2Ey + F = 0
	Vector orbits_params;

	Type orbits_type;

	bool completed_at_least_one_cycle = false;
	//��������� ��� �������� ������ � ������������ ��� (����� ������� ������ ��������� �������� ����������) � ������ �������� ��������� ������� ����� ������� ��� ������ ��� �������
	//������
	Matrix rotation_to_canon_matrix;
	Matrix inverse_rotation_to_canon_matrix;
	Vector shift_to_canon;
	//in absolute coordinates
	Vector F1, F2;
	Vector* gravitation_centre;
	Vector aphelion, perihelion;
	double eccentricity;
	//��� ���������
	std::vector<Vector> traject_points;
	bool fully_counted = false;

	bool defined_gravitation_centre = false;
	//its for graphical part mostly;
	//������ ����� ��� ��� ����������� ����� �������� � �������� ��� ��� ������������� ���� ������, �� ��� ������ ��� ����� ������� � �� ������� ������ ����������� �����������
	bool changed = true;
	sf::VertexArray graphical;
	//��� ����� �������� ������� ��������
	bool need_to_redraw = false;

	//��� ��������� ����� ����� ������ ��� �� ����������
	//� ���� ������ ��� � add_point ��������� traject_points �� 0 ���� �� ��������� ������
	bool in_trace_mode = true;
	Vector last_vertex;
	
};

