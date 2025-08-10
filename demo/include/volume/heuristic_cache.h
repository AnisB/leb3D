#pragma once

// Internal includes
#include "volume/grid_volume.h"

// External includes
#include <stdint.h>

// Structure that allows us to evalute our heuristic
struct HeuristicCache
{
	// Number of levels in our cache
	uint32_t numLevels;
	// Top resolution of our grid (assume it's cubic texture, but doesn't have to be)
	uint32_t resolution;
	// Per level resolution of our grid
	std::vector<uint32_t> resolutions;
	// Per level offsets to access the heuristic cache
	std::vector<uint64_t> offsets;
	// Global moment buffer
	std::vector<float4> momentArray;
};

namespace heuristic_cache
{
	// Build the cache
	void build_heuristic_cache(const GridVolume& volume, HeuristicCache& cache);

	// Sample the cache
	float4 sample_cache(const HeuristicCache& cache, const float3& position, uint32_t depth);
}