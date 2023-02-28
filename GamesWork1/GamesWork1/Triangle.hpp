#pragma once

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Triangle
{
public:
	Vector3f v[3];//��������ε��������㣬�����Ǿ�mvp���ӿڱ�ú�ĵ㣬��rasterizer����ʵ�ִ��

	//����ÿ����������ԣ�������ɫ����������
	Vector3f color[3];
	Vector2f tex_coords[3];
	Vector3f normal[3];

	Triangle();
	//��ȡÿ������ĺ���
	Vector3f a() const { return v[0]; }
	Vector3f b() const { return v[1]; }
	Vector3f c() const { return v[2]; }

	//���������ε�i�����������
	void setVertex(int ind, Vector3f ver);
	void setNormal(int ind, Vector3f n);
	void setColor(int ind, float r, float g, float b);
	void setTexCoord(int ind, float s, float t);

	array<Vector4f, 3> toVector4() const;

};



#endif