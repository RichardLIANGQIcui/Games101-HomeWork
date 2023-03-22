#pragma once

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H

#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

using namespace std;

class Bounds3
{
public:
	Vector3f pMin, pMax;//两个最值点确定一个包围盒
	Bounds3()
	{
		double minNum = numeric_limits<double>::lowest();
		double maxNum = numeric_limits<double>::max();
		pMax = Vector3f(minNum, minNum, minNum);
		pMin = Vector3f(maxNum, maxNum, maxNum);
	}
	Bounds3(const Vector3f p) :pMin(p), pMax(p){}
	Bounds3(const Vector3f p1, const Vector3f p2)
	{
		pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
		pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
	}

	Vector3f Diagonal() const { return pMax - pMin; }//包围盒的对角线

	int maxExtent() const//包围盒最大的xx
	{
		Vector3f d = Diagonal();
		if (d.x > d.y && d.x > d.z)
		{
			return 0;
		}
		else if (d.y > d.z)
		{
			return 1;
		}
		else
		{
			return 2;
		}

	}

	double SurfaceArea() const	//求包围盒的表面积
	{
		Vector3f d = Diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }//求包围盒的中心点

	Bounds3 Intersect(const Bounds3& b)//求两个包围盒相交部分
	{
		return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
			fmax(pMin.z, b.pMin.z)),
			Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
				fmin(pMax.z, b.pMax.z)));
	}

	Vector3f Offset(const Vector3f& p) const
	{
		Vector3f o = p - pMin;
		if (pMax.x > pMin.x)
		{
			o.x /= pMax.x - pMin.x;
		}
		if (pMax.y > pMin.y)
		{
			o.y /= pMax.y - pMin.y;
		}
		if (pMax.z > pMin.z)
		{
			o.z /= pMax.z - pMin.z;
		}
		return o;
	}

	bool Overlaps(const Bounds3& b1, const Bounds3& b2)//判断两个包围盒是否重叠,这个只要画两条线段比较即可
	{
		//满足b1.pMax>=b2.pMin&&b2.pMax>=b1.pMin条件就相交，相交的话必然三个方向都相交
		bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
		bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
		bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);

		return x && y && z;
	}

	bool Inside(const Vector3f& p, const Bounds3& b)//判断点是否在包围盒内
	{
		return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y &&
			p.z >= b.pMin.z && p.z <= b.pMax.z);
	}

	inline const Vector3f& operator[](int i) const
	{
		return (i == 0) ? pMin : pMax;
	}

	inline bool IntersectP(const Ray& ray, const Vector3f& invDir, const array<int, 3>& dirisNeg) const;

};

//光线与包围盒有交点满足2个条件即可
//1、t_exit>t_enter;
//2、t_exit>=0;
//t_exit=min(t_xmax,t_ymax,t_zmax), t_enter = max(t_xmin,t_ymin,t_zmin)
// dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
//注意这里为什么要使用invDir,因为后面公式会用到除法，但实际运算中除法比乘法慢，所以这里要作一个转化,
//invDir在传入这个函数时已经计算好了
// invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
//参数dirIsNeg是为了确定光的方向是从pMin->pMax,还是pMax->pMin,tmin和tmax会根据方向的不同而不同
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
	const std::array<int, 3>& dirIsNeg) const
{
	float t_xmin = (pMin.x - ray.origin.x) * invDir.x;
	float t_xmax = (pMax.x - ray.origin.x) * invDir.x;
	float t_ymin = (pMin.y - ray.origin.y) * invDir.y;
	float t_ymax = (pMax.y - ray.origin.y) * invDir.y;
	float t_zmin = (pMin.z - ray.origin.z) * invDir.z;
	float t_zmax = (pMax.z - ray.origin.z) * invDir.z;

	if (!dirIsNeg[0])//如果方向是pMax->pMin,则tmin和tmax须交换
	{
		float t = t_xmin;
		t_xmin = t_xmax;
		t_xmax = t;
	}
	if (!dirIsNeg[1])
	{
		float t = t_ymin;
		t_ymin = t_ymax;
		t_ymax = t;
	}
	if (!dirIsNeg[2])
	{
		float t = t_zmin;
		t_zmin = t_zmax;
		t_zmax = t;
	}

	float t_enter = max(t_xmin, max(t_ymin, t_zmin));
	float t_exit = min(t_xmax, min(t_ymax, t_zmax));

	if (t_exit > t_enter && t_exit >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)//求盒子的并集
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)//求点和盒子的并集
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}

#endif