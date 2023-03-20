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
        //光线与圆求交是根据方程（o+td-c)^2-R^2=0,得出at^2+bt+c=0,关于时间t的一元二次方程
        //a=d点乘d,b=2*(o-c)点乘d，c=(o-c)点乘(o-c)-R^2
        // analytic solution
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)//只有t>0时才相交,这里对第一个解进行判断
            t0 = t1;
        if (t0 < 0)//这里对第二个解进行判断，如果都小于0，则交点无效
            return false;
        tnear = t0;//有效解的最小值

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
