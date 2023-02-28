#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>//异常捕获的标准库

using namespace std;
using namespace Eigen;

Triangle::Triangle()
{
	v[0] << 0, 0, 0;
	v[1] << 0, 0, 0;
	v[2] << 0, 0, 0;

	color[0] << 0.0, 0.0, 0.0;
	color[1] << 0.0, 0.0, 0.0;
	color[2] << 0.0, 0.0, 0.0;

	tex_coords[0] << 0.0, 0.0;
	tex_coords[1] << 0.0, 0.0;
	tex_coords[2] << 0.0, 0.0;

}

void Triangle::setVertex(int ind, Vector3f ver)
{
	v[ind] = ver;
}

void Triangle::setNormal(int ind, Vector3f n)
{
	normal[ind] = n;
}

void Triangle::setColor(int ind, float r, float g, float b)
{
	//抛出异常
	if ((r < 0.0) || (r > 255.0) || (g < 0.0) || (g > 255.0) || (b < 0.0) || (b > 255.0))
	{
		throw runtime_error("Invalid color value");
	}

	color[ind] = Vector3f((float)r / 255.0, (float)g / 255.0, (float)b / 255.0);
	return;
}

void Triangle::setTexCoord(int ind, float s, float t)
{
	tex_coords[ind] = Vector2f(s, t);
}

array<Vector4f, 3> Triangle::toVector4() const
{
	array<Vector4f, 3> res;
	//这里使用到了transform函数，避免了代码重复，实现一个序列的元素按照某种计算规则（lambda表达式）计算后再将值返回给另一个序列
	transform(begin(v), end(v), res.begin(), [](auto& vec) {
		return Vector4f(vec.x(), vec.y(), vec.z(), 1.f);
		});

	return res;

}