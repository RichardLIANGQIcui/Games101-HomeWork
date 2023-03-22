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
	enum class SplitMethod{NAIVE,SAH};//定义加速方法，第一个是bvh，第二个是sah

	BVHAccel(vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
	Bounds3 WorldBound() const;
	~BVHAccel();

	Intersection Intersect(const Ray& ray) const;
	Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
	bool IntersectP(const Ray& ray) const;
	BVHBuildNode* root;

	BVHBuildNode* recursiveBuild(vector<Object*>objects);

	const int maxPrimsInNode;//节点包含的物体的最大数量
	const SplitMethod splitMethod;
	vector<Object*> primitives;
};

//用一个二叉树结构封装了包围盒、子节点和一个物体指针
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