#pragma once

// Project includes
#include "math/types.h"

// System includes
#include <stdint.h>
#include <vector>

class MortonCache
{
public:
	MortonCache();
	~MortonCache();

	// Helper functions
	void build_cache(const float3* positionArray, uint32_t numPositions);
	uint32_t get_closest_element(const float3& position);

	// Internal element holder
	struct Element
	{
		// Morton code of the element
		uint64_t code;

		// Index of the element
		uint32_t index;
	};
private:
	uint64_t evaluate_morton_code(const float3& targetPosition);

private:
	std::vector<Element> m_Cache;
	uint32_t m_NumElements = 0;
	float3 m_MinPosition = { FLT_MAX, FLT_MAX, FLT_MAX };
	float3 m_MaxPosition = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
};