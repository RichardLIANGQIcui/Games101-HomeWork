#pragma once

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Triangle
{
public:
	Vector3f v[3];//存放三角形的三个顶点，这里是经mvp和视口变幻后的点，在rasterizer类里实现存放

	//声明每个顶点的属性，包括颜色、纹理、法线
	Vector3f color[3];
	Vector2f tex_coords[3];
	Vector3f normal[3];

	Triangle();
	//获取每个顶点的函数
	Vector3f a() const { return v[0]; }
	Vector3f b() const { return v[1]; }
	Vector3f c() const { return v[2]; }

	//设置三角形第i个顶点的属性
	void setVertex(int ind, Vector3f ver);
	void setNormal(int ind, Vector3f n);
	void setColor(int ind, float r, float g, float b);
	void setTexCoord(int ind, float s, float t);

	array<Vector4f, 3> toVector4() const;

};



#endif