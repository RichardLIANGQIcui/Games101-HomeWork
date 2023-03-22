#pragma once
#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

using namespace std;

struct BVHBuildNode;

struct BVHPrimitiveInfo;

inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel {

public:
	enum class SplitMethod{NAIVE,SAH};//������ٷ�������һ����bvh���ڶ�����sah

	BVHAccel(vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
	Bounds3 WorldBound() const;
	~BVHAccel();

	Intersection Intersect(const Ray& ray) const;
	Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
	bool IntersectP(const Ray& ray) const;
	BVHBuildNode* root;

	BVHBuildNode* recursiveBuild(vector<Object*>objects);

	const int maxPrimsInNode;//�ڵ������������������
	const SplitMethod splitMethod;
	vector<Object*> primitives;
};

//��һ���������ṹ��װ�˰�Χ�С��ӽڵ��һ������ָ��
struct BVHBuildNode {
	Bounds3 bounds;
	BVHBuildNode* left;
	BVHBuildNode* right;
	Object* object;

public:
	int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;

	BVHBuildNode() {
		bounds = Bounds3();
		left = nullptr, right = nullptr;
		object = nullptr;
	}
};



#endif