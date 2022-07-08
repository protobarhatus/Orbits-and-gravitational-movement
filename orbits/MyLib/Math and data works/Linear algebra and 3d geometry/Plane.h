#pragma once
#include "Vector3.h"
#include "Line3d.h"
#include "Vector2.h"
class Plane
{
public:
	Plane();
	Plane(double a, double b, double c, double d, bool make_units_vectors = false);
	~Plane();
	Vector3d normal() const;
	double a() const;
	double b() const;
	double c() const;
	double d() const;
	void setA(double a);
	void setB(double b);
	void setC(double c);
	void setD(double d);
	//���������� ��������� �����, ������������� ���������, �� �� �������� � ��������
	double z(double x, double y);
	//���������� �������� �����, ������������� ���������, �� �� �������� � ���������
	double y(double x, double z);
	//���������� �������� �����, ������������� ���������, �� �� �������� � ���������
	double x(double y, double z);


	//������������ ����, ������� �������� ��������� ������� ��������� � ������ ��������� � ������������ �� � ���� �������� ����������� ������������
	void makeOwnUnitsVectors();

	Line3d getIntersection(const Plane & plane);
	Vector2 getCoordinatesInPlanesUnitsVector(const Vector3d & point) const;
private:
	double _a, _b, _c, _d;
	
	
	bool has_counted_units_vectors = false;
	Vector3d abscissa_unit_vector, ordinata_unit_vector;
	Line3d abscissa_axis, ordinata_axis;
};

Plane getPlaneThroughPointAndTwoVectors(const Vector3d& point, const Vector3d& vector1, const Vector3d& vector2);