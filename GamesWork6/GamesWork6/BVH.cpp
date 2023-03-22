#include <algorithm>
#include <cassert>
#include "BVH.hpp"

using namespace std;

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    :maxPrimsInNode(min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(move(p))
{
    time_t start, stop;
    time(&start);//获取当前时间并赋值给start，这里设置这两个时间的作用好像是为了对比另外一个加速方法sah
    if (primitives.empty())
    {
        return;
    }

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;//获得小时为单位的变量
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(//输出使用bvh加速方法的完成时间
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

//BVH分割两个原理：1、按最长轴分割；2、按场景内的物体数量的中位数进行分割，具体通过排序实现
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    Bounds3 bounds;
    //将场景中每个物体的包围盒合并，求出一个包围整个场景的大包围盒
    for (int i = 0;i < objects.size();++i)
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }
    //该函数递归的终止条件，即当场景被分割只剩1个物体时就返回，并且记录一些信息
    if (objects.size() == 1)
    {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }//当场景中只剩2个物体时，则继续分割。左右子节点各分配一个物体
    else if (objects.size() == 2) {
        node->left = recursiveBuild(vector{ objects[0] });
        node->right = recursiveBuild(vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0;i < objects.size();++i)
        {
            //把场景中所有物体的中心点都包含在一起
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        int dim = centroidBounds.maxExtent();//求出最长轴，0表示x轴，1表示y轴，2表示z轴
        switch (dim)//按最长轴对物体进行排序
        {
        case 0:
            sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
                });
            break;

        case 1:
            sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
                });
            break;

        case 2:
            sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
                });
            break;
        }

        //拍完序后要找出中位数的物体
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        //这里使用迭代器构造一个新的容器
        auto leftshapes = vector<Object*>(beginning, middling);
        auto rightshapes = vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;

}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
    {
        return isect;
    }
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    //这里判断光传播的方向到底是pMin->pMax,还是pMin->pMax
    //原理只要判断ray.direction是否大于0即可，大于0则是往pMax发射，小于0则是往pMin
    array<int, 3> dirIsNeg ;
    dirIsNeg[0] = (ray.direction.x > 0);
    dirIsNeg[1] = (ray.direction.y > 0);
    dirIsNeg[2] = (ray.direction.z > 0);

    Intersection isec;
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return isec;
    }

    if (node->left == nullptr && node->right == nullptr)
    {
        isec = node->object->getIntersection(ray);
        return isec;
    }

    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);

    return hit1.distance < hit2.distance ? hit1 : hit2;
}