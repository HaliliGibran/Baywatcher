#ifndef _USER_IMAGE_MATH_H
#define _USER_IMAGE_MATH_H

#include <cstdint>
#include <cstring>

// 功能: 整型夹紧
// 类型: 全局功能函数
// 关键参数: x-输入值, low/up-范围
inline int clip(int x, int low, int up)
{
	return x > up ? up : x < low ? low
								 : x;
}

// 功能: 浮点夹紧
// 类型: 全局功能函数
// 关键参数: x-输入值, low/up-范围
inline float fclip(float x, float low, float up)
{
	return x > up ? up : x < low ? low
								 : x;
}

// 功能: 快速平方根（Q_sqrt）
// 类型: 全局功能函数
// 关键参数: number-输入值（>0）
// 说明: 先求 1/sqrt(x)，再取倒数得到 sqrt(x)
inline float Q_sqrt(float number)
{
	if (number <= 0.0f)
	{
		return 0.0f;
	}

	const float threehalfs = 1.5F;
	float x2 = number * 0.5F;
	float y = number;

	int32_t i;
	std::memcpy(&i, &y, sizeof(float));
	i = 0x5f3759df - (i >> 1);
	std::memcpy(&y, &i, sizeof(float));

	y = y * (threehalfs - (x2 * y * y));
	y = y * (threehalfs - (x2 * y * y));

	return 1.0f / y;
}

// 功能: 快速倒数平方根（1/sqrt(x)）
// 类型: 全局功能函数
// 关键参数: x-输入值（>0）
inline float fast_rsqrt(float x)
{
	if (x <= 0.0f)
	{
		return 0.0f;
	}

	const float xhalf = 0.5f * x;
	int32_t i;
	std::memcpy(&i, &x, sizeof(float));
	i = 0x5f3759df - (i >> 1);
	float y;
	std::memcpy(&y, &i, sizeof(float));
	y = y * (1.5f - xhalf * y * y);
	return y;
}

#endif
