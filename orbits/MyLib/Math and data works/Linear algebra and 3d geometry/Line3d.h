#pragma once
#include "Vector3.h"
class Line3d
{
public:
	Line3d(Vector3d first_point, Vector3d second_point);
	~Line3d();
	Line3d();
	Vector3d getBaseVector() const;
	Vector3d getPointOnLine() const;
private:
	//�����-���� �����
	Vector3d point_on_line;
	//������������ ������
	Vector3d base_vector;
};

