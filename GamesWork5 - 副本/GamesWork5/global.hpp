#pragma once

#include <cmath>
#include <iostream>
#include <random>

#define M_PI 3.14159265358979323846

constexpr float kInfinity = std::numeric_limits<float>::max();
//�������䷽���ж�
inline float clamp(const float& lo, const float& hi, const float& v)
{
    return std::max(lo, std::min(hi, v));
}
//���һԪ���η��̵ĸ����ο�����https://zhuanlan.zhihu.com/p/374649261
//����ʵ�����ĺ���ѧ����̫һ��������˵��������Ϊfloat��������������һЩ���������£�����ϴ�
// ��������float a,b��ֵ���ʱ����a-b������ϴ�a+b������С����ô�������ʽ��b��������������һ�����ͻ�����ϴ����
//Ϊ���������һ����ȼ�������һ�����С�ĸ�x1���ٸ���Τ�ﶨ��x1*x2=c/a����x2
inline bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0)
        return false;
    else if (discr == 0)
        x0 = x1 = -0.5 * b / a;
    else
    {
        float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));//�������С���Ǹ�x������ʽ�ǣ�a+b)
        x0 = q / a;
        x1 = c / q;//����Τ�ﶨ����x2
    }
    if (x0 > x1)
        std::swap(x0, x1);//��һ��һС��ȡ����ĵ㣬���Ҫswapλ�ñ�֤x0Ҳ����t0����С
    return true;
}


//�������ֲ�ͬ��ģ�ͣ����ÿ�ֲ�ͬ��ģ�ͼ��㷽��Ҳ��һ������һ����phongģ�ͼ��㣬�ڶ��������÷���������

enum MaterialType
{
    DIFFUSE_AND_GLOSSY,
    REFLECTION_AND_REFRACTION,
    REFLECTION
};

inline float get_random_float()
{
    std::random_device dev;//C++11�±�׼�������������
    std::mt19937 rng(dev());//c++11�±�׼��α����������������ڲ��������ܵ������
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]//�Ӿ��ȷֲ������������

    return dist(rng);
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i)
    {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}
