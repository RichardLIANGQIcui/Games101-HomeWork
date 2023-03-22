#pragma once
#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"

class Object;
class Sphere;

struct Intersection
{
	bool happened;
	Vector3f coords;//光线与物体的交点
	Vector3f normal;
	double distance;//光线到交点的距离
	Object* obj;
	Material* m;
	Intersection()
	{
		happened = false;
		coords = Vector3f();
		normal = Vector3f();
		distance = numeric_limits<double>::max();
		obj = nullptr;
		m = nullptr;
	}
};


#endif