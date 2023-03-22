#pragma once

#include <vector>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"

using namespace std;

class Scene
{
public:
	int width = 1280;
	int height = 960;
	double fov = 90;
	Vector3f backGroundColor = Vector3f(0.235294, 0.67451, 0.843137);
	int maxDepth = 5;

	vector<Object*> objects;
	vector<unique_ptr<Light>> lights;

	Scene(int w,int h):width(w),height(h){}

	void Add(Object* object) { objects.push_back(object); }
	void Add(unique_ptr<Light> light) { lights.push_back(move(light)); }

	const vector<Object*>& get_objects() const { return objects; }
	const vector<unique_ptr<Light>>& get_lights() const { return lights; }
	Intersection intersect(const Ray& ray) const;
	BVHAccel* bvh;
	void buildBVH();
	Vector3f castRay(const Ray& ray, int depth) const;
	bool trace(const Ray& ray, const vector<Object*>& objects, float& tNear, uint32_t& index, Object** hitObject);
	tuple<Vector3f, Vector3f> HandleAreaLight(const AreaLight& light, const Vector3f& hitPosition,
		const Vector3f& N, const Vector3f& shadowPointOrig, const vector<Object*>& objects,
		uint32_t& index, const Vector3f& dir, float specularExponent);

	Vector3f reflect(const Vector3f& I, const Vector3f& N)const
	{
		return I - 2 * dotProduct(I, N) * N;
	}

	Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior)const
	{
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		Vector3f n = N;
		if (cosi < 0) {
			cosi = -cosi;
		}
		else
		{
			swap(etai, etat);
			n = -N;
		}
		float eta = etai / etat;
		float k = 1 - eta * eta * (1 - cosi * cosi);
		return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
	}

	void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
	{
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		if (cosi > 0) { swap(etai, etat); }
		float sint = etai / etat * sqrtf(max(0.f, 1 - cosi * cosi));
		if (sint >= 1)
		{
			kr = 1;
		}
		else
		{
			float cost = sqrtf(max(0.f, 1 - sint * sint));
			cosi = fabsf(cosi);
			float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
			float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
			kr = (Rs * Rs + Rp * Rp) / 2;
		}
	}

};