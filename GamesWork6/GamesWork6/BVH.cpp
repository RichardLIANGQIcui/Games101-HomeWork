#include <algorithm>
#include <cassert>
#include "BVH.hpp"

using namespace std;

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
    SplitMethod splitMethod)
    :maxPrimsInNode(min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(move(p))
{
    time_t start, stop;
    time(&start);//��ȡ��ǰʱ�䲢��ֵ��start����������������ʱ������ú�����Ϊ�˶Ա�����һ�����ٷ���sah
    if (primitives.empty())
    {
        return;
    }

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;//���СʱΪ��λ�ı���
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(//���ʹ��bvh���ٷ��������ʱ��
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

//BVH�ָ�����ԭ��1�������ָ2���������ڵ�������������λ�����зָ����ͨ������ʵ��
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    Bounds3 bounds;
    //��������ÿ������İ�Χ�кϲ������һ����Χ���������Ĵ��Χ��
    for (int i = 0;i < objects.size();++i)
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }
    //�ú����ݹ����ֹ�����������������ָ�ֻʣ1������ʱ�ͷ��أ����Ҽ�¼һЩ��Ϣ
    if (objects.size() == 1)
    {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }//��������ֻʣ2������ʱ��������ָ�����ӽڵ������һ������
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
            //�ѳ�����������������ĵ㶼������һ��
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        int dim = centroidBounds.maxExtent();//�����ᣬ0��ʾx�ᣬ1��ʾy�ᣬ2��ʾz��
        switch (dim)//�����������������
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

        //�������Ҫ�ҳ���λ��������
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        //����ʹ�õ���������һ���µ�����
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
    //�����жϹ⴫���ķ��򵽵���pMin->pMax,����pMin->pMax
    //ԭ��ֻҪ�ж�ray.direction�Ƿ����0���ɣ�����0������pMax���䣬С��0������pMin
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