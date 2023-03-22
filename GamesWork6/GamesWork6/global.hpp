#pragma once

#include <iostream>
#include <cmath>
#include <random>

using namespace std;

#undef M_PI
#define M_PI 3.141592653589793f

extern const float EPSILON;

const float kInfinity = numeric_limits<float>::max();

inline float clamp(const float& a, const float& b, const float& c)
{
	return max(a, min(b, c));
}

inline bool solevQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
{
	float discr = b * b - 4 * a * c;
	if (discr < 0)
	{
		return false;
	}
	if (discr == 0)
	{
		x0 = x1 = -b / (2 * a);
		
	}
	else
	{
		float q = (b > 0) ? -0.5 * (b + sqrtf(discr)) : -0.5 * (b - sqrtf(discr));
		x0 = q / a;
		x1 = c / q;
	}
	if (x0 > x1)
	{
		swap(x0, x1);
	}

	return true;
}

inline float get_random_float()
{
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

	return dist(rng);
}

inline void UpdateProgress(float progress)
{
	int barWidth = 70;

	std::cout << "[";
	int pos = barWidth * progress;
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "=";
		else if (i == pos) std::cout << ">";
		else std::cout << " ";
	}
	std::cout << "] " << int(progress * 100.0) << " %\r";
	std::cout.flush();
};