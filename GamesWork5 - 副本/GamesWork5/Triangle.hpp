#pragma once

#include "Object.hpp"

#include <cstring>

//这里判断三角形是否和光线相交使用了一个Moller-Trumbore算法，具体参考https://blog.csdn.net/zhanxi1992/article/details/109903792
//o+td = (1-b1-b2)*P0+b1*P1+b2*P2;解一元二次方程，有解才有交点，t>=0&&b1>=0&&b2>=0&&(1-b1-b2)>=0

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
    const Vector3f& dir, float& tnear, float& u, float& v) // u和v表示的是重心坐标。不是纹理坐标
{
    // 解t、b1、b2几个用到的参数
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;
    Vector3f S = orig - v0;
    Vector3f S1 = crossProduct(dir, E2);
    Vector3f S2 = crossProduct(S, E1);

    float t = dotProduct(S2, E2) / dotProduct(E1, S1);
    float b1 = dotProduct(S1, S) / dotProduct(E1, S1);
    float b2 = dotProduct(S2, dir) / dotProduct(E1, S1);
    //由于浮点数比较时在代码中通常是选一个接近0的浮点数进行比较
    double eps = -1e-6;
    if (t > eps && (1 - b1 - b2) > eps && b1 > eps && b2 > eps)
    {
        tnear = t;
        u = b1;
        v = b2;

        return true;
    }
    return false;
}

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];//找出最大索引
        maxIndex += 1;//求出顶点个数，总共4个顶点，用于后续空间的开辟
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        //通过内存拷贝函数将verts中的内容拷贝到vertices中
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);//根据顶点索引个数开辟空间
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
        Vector2f& uv) const override//使用关键字对基类进行覆盖重写
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }
    //st是重心坐标，这个函数是在Renderer类中被使用，光线求交后将求交点的属性传给该函数，属性值就被保存到智能指针上
    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
        Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        //fmodf(x,y)是求余预算，表示x对y求余，如fmodf(9.2,2.0) = 1.2
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
