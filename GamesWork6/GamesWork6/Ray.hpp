#pragma once
#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H

#include "Vector.hpp"

using namespace std;

struct Ray {
	Vector3f origin;
	Vector3f direction, direction_inv;//direction_inv”√”⁄Bounds3::IntersectP
	double t;
	double t_min, t_max;

	Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0) :origin(ori), direction(dir), t(_t)
	{
		direction_inv = Vector3f(1. / direction.x, 1. / direction.y, 1. / direction.z);
		t_min = 0.0;
		t_max = numeric_limits<float>::max();
	}

	Vector3f operator()(double t) const {
		return origin + direction * t;
	}

	friend ostream& operator<<(ostream& os, const Ray& r)
	{
		os << "[origin:=" << r.origin << ", direction=" << r.direction << ", time=" << r.t << "]\n";
		return os;
	}
};
#endif