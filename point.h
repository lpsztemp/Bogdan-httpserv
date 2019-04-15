#include "basedefs.h"

#ifndef CONVERTERS_POINT_H
#define CONVERTERS_POINT_H

struct point_t
{
	double x, y, z;
};

constexpr point_t cross_product(const point_t& left, const point_t& right) noexcept
{
	return
	{
		right.y * left.z - left.y * right.z,
		left.x * right.z - right.x * left.z,
		right.x * left.y - left.x * right.y
	};
};

constexpr point_t operator-(const point_t& left, const point_t& right) noexcept
{
	return
	{
		left.x - right.x,
		left.y - right.y,
		left.z - right.z
	};
}

constexpr bool operator==(const point_t& left, const point_t& right)
{
	return 
		left.x == right.x &&
		left.y == right.y &&
		left.z == right.z;
}

constexpr bool operator!=(const point_t& left, const point_t& right)
{
	return !(left == right);
}

#endif //CONVERTERS_POINT_H