#pragma once

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>
#include "Texture.hpp"

using namespace std;
using namespace Eigen;

class Triangle
{
public:
	Vector4f v[3];//��������ε��������㣬�����Ǿ�mvp���ӿڱ�ú�ĵ㣬��rasterizer����ʵ�ִ��

	//����ÿ����������ԣ�������ɫ����������
	Vector3f color[3];
	Vector2f tex_coords[3];
	Vector3f normal[3];

	Triangle();
	Texture* tex = nullptr;

	Vector4f a() const { return v[0]; }
	Vector4f b() const { return v[1]; }
	Vector4f c() const { return v[2]; }

	//���������ε�i�����������
	void setVertex(int ind, Vector4f ver);
	void setNormal(int ind, Vector3f n);
	void setColor(int ind, float r, float g, float b);

	void setNormals(const array<Vector3f, 3>& normals);
	void setColors(const array<Vector3f, 3>& colors);
	void setTexCoord(int ind, Vector2f uv);

	array<Vector4f, 3> toVector4() const;

};



#endif