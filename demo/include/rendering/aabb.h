#pragma once

// Project includes
#include "math/types.h"

// System includes
#include <float.h>

struct AABB
{
	float3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
	float3 max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
};