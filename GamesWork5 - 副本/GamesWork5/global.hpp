#pragma once

#include <cmath>
#include <iostream>
#include <random>

#define M_PI 3.14159265358979323846

constexpr float kInfinity = std::numeric_limits<float>::max();
//用于入射方向判断
inline float clamp(const float& lo, const float& hi, const float& v)
{
    return std::max(lo, std::min(hi, v));
}
//求解一元二次方程的根，参考文章https://zhuanlan.zhihu.com/p/374649261
//这里实际求解的和数学还不太一样，简单来说，就是因为float浮点数存在误差，在一些特殊的情况下，误差会较大，
// 例如两个float a,b数值相近时，则a-b的误差会较大，a+b的误差较小，那么若求根公式中b与△相近，则其中一个根就会产生较大误差
//为避免产生误差，一般会先计算其中一个误差小的跟x1，再根据韦达定理x1*x2=c/a计算x2
inline bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0)
        return false;
    else if (discr == 0)
        x0 = x1 = -0.5 * b / a;
    else
    {
        float q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr));//求误差最小的那个x，即形式是（a+b)
        x0 = q / a;
        x1 = c / q;//这里韦达定理求x2
    }
    if (x0 > x1)
        std::swap(x0, x1);//根一大一小，取最近的点，因此要swap位置保证x0也就是t0是最小
    return true;
}


//定义三种不同的模型，针对每种不同的模型计算方法也不一样，第一种用phong模型计算，第二、三种用菲涅耳方程

enum MaterialType
{
    DIFFUSE_AND_GLOSSY,
    REFLECTION_AND_REFRACTION,
    REFLECTION
};

inline float get_random_float()
{
    std::random_device dev;//C++11新标准，随机数生成器
    std::mt19937 rng(dev());//c++11新标准，伪随机数生成器，用于产生高性能的随机数
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]//从均匀分布中生成随机数

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
