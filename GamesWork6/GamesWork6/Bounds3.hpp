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
	Vector3f pMin, pMax;//������ֵ��ȷ��һ����Χ��
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

	Vector3f Diagonal() const { return pMax - pMin; }//��Χ�еĶԽ���

	int maxExtent() const//��Χ������xx
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

	double SurfaceArea() const	//���Χ�еı����
	{
		Vector3f d = Diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }//���Χ�е����ĵ�

	Bounds3 Intersect(const Bounds3& b)//��������Χ���ཻ����
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

	bool Overlaps(const Bounds3& b1, const Bounds3& b2)//�ж�������Χ���Ƿ��ص�,���ֻҪ�������߶αȽϼ���
	{
		//����b1.pMax>=b2.pMin&&b2.pMax>=b1.pMin�������ཻ���ཻ�Ļ���Ȼ���������ཻ
		bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
		bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
		bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);

		return x && y && z;
	}

	bool Inside(const Vector3f& p, const Bounds3& b)//�жϵ��Ƿ��ڰ�Χ����
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

//�������Χ���н�������2����������
//1��t_exit>t_enter;
//2��t_exit>=0;
//t_exit=min(t_xmax,t_ymax,t_zmax), t_enter = max(t_xmin,t_ymin,t_zmin)
// dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
//ע������ΪʲôҪʹ��invDir,��Ϊ���湫ʽ���õ���������ʵ�������г����ȳ˷�������������Ҫ��һ��ת��,
//invDir�ڴ����������ʱ�Ѿ��������
// invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
//����dirIsNeg��Ϊ��ȷ����ķ����Ǵ�pMin->pMax,����pMax->pMin,tmin��tmax����ݷ���Ĳ�ͬ����ͬ
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
	const std::array<int, 3>& dirIsNeg) const
{
	float t_xmin = (pMin.x - ray.origin.x) * invDir.x;
	float t_xmax = (pMax.x - ray.origin.x) * invDir.x;
	float t_ymin = (pMin.y - ray.origin.y) * invDir.y;
	float t_ymax = (pMax.y - ray.origin.y) * invDir.y;
	float t_zmin = (pMin.z - ray.origin.z) * invDir.z;
	float t_zmax = (pMax.z - ray.origin.z) * invDir.z;

	if (!dirIsNeg[0])//���������pMax->pMin,��tmin��tmax�뽻��
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

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)//����ӵĲ���
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)//���ͺ��ӵĲ���
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}

#endif