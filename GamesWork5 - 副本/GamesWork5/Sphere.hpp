#pragma once

#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object
{
public:
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r* r)
    {}

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        //������Բ���Ǹ��ݷ��̣�o+td-c)^2-R^2=0,�ó�at^2+bt+c=0,����ʱ��t��һԪ���η���
        //a=d���d,b=2*(o-c)���d��c=(o-c)���(o-c)-R^2
        // analytic solution
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)//ֻ��t>0ʱ���ཻ,����Ե�һ��������ж�
            t0 = t1;
        if (t0 < 0)//����Եڶ���������жϣ������С��0���򽻵���Ч
            return false;
        tnear = t0;//��Ч�����Сֵ

        return true;
    }

    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
        Vector3f& N, Vector2f&) const override
    {
        N = normalize(P - center);
    }

    Vector3f center;
    float radius, radius2;
};
