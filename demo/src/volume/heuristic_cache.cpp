// Internal includes
#include "math/operators.h"
#include "volume/heuristic_cache.h"
#include "tools/security.h"

// External includes incldues
#include <algorithm>

namespace heuristic_cache
{
	void build_heuristic_cache(const GridVolume& volume, HeuristicCache& cache)
	{
		assert_msg(volume.resolution.x == volume.resolution.y 
				&& volume.resolution.x == volume.resolution.z, 
				"This code assumes the grid is cubic, but the code can be extended.");

		// Keep track of the top resolution
		cache.resolution = volume.resolution.x >> 1;

		// Count the total number of cells
		uint64_t currentRes = cache.resolution;
		uint64_t initialOffset = 0;
		while (currentRes > 0)
		{
			// Level offset
			cache.offsets.push_back(initialOffset);
			cache.resolutions.push_back((uint32_t)currentRes);
			
			// Global offset
			initialOffset += currentRes * currentRes * currentRes;

			// Reduce resolution
			currentRes >>= 1;
		}

		// Allocate the memory space
		cache.momentArray.resize(initialOffset);
		cache.numLevels = (uint32_t)cache.offsets.size();

		// First we evaluate the lowest level
		#pragma omp parallel for num_threads(32)
		for (int32_t z = 0; z < (int32_t)cache.resolution; ++z)
		{
			for (uint32_t y = 0; y < cache.resolution; ++y)
			{
				for (uint32_t x = 0; x < cache.resolution; ++x)
				{
					// Compute the statistics
					float minV = FLT_MAX;
					float maxV = -FLT_MAX;
					float mean = 0.0;
					float mean2 = 0.0;
					for (uint32_t lz = 0; lz < 2; ++lz)
					{
						for (uint32_t ly = 0; ly < 2; ++ly)
						{
							for (uint32_t lx = 0; lx < 2; ++lx)
							{
								// Offset of the input cell
								uint64_t offset = (uint64_t)x * 2 + (uint64_t)lx + (uint64_t)(y * 2ull + ly) * volume.resolution.x + (uint64_t)(z * 2ull + lz) * volume.resolution.x * volume.resolution.y;

								// Grab the density
								float density = volume.densityArray[offset];

								// Contribute
								mean += density;
								mean2 += density * density;
								minV = std::min(minV, density);
								maxV = std::max(maxV, density);
							}
						}
					}
					// Normalize the average
					uint64_t cacheOffset = (uint64_t)x + (uint64_t)y * cache.resolution + (uint64_t)z * cache.resolution * cache.resolution;
					cache.momentArray[cacheOffset] = { mean * 0.125f, mean2 * 0.125f, minV, maxV };
				}
			}
		}

		// Then process the remaining levels
		uint32_t inputRes = cache.resolution;
		uint32_t outputRes = inputRes >> 1;
		for (uint32_t lvlIdx = 1; lvlIdx < cache.offsets.size(); ++lvlIdx)
		{
			// First we evaluate the lowest level
			#pragma omp parallel for num_threads(32)
			for (int32_t z = 0; z < (int32_t)outputRes; ++z)
			{
				for (uint32_t y = 0; y < outputRes; ++y)
				{
					for (uint32_t x = 0; x < outputRes; ++x)
					{
						float minV = FLT_MAX;
						float maxV = -FLT_MAX;
						float mean = 0.0;
						float mean2 = 0.0;
						for (uint32_t lz = 0; lz < 2; ++lz)
						{
							for (uint32_t ly = 0; ly < 2; ++ly)
							{
								for (uint32_t lx = 0; lx < 2; ++lx)
								{
									// Offset of the input cell
									uint64_t offset = cache.offsets[lvlIdx - 1] + (uint64_t)x * 2 + (uint64_t)lx + (uint64_t)(y * 2 + ly) * inputRes + (uint64_t)(z * 2 + lz) * inputRes * inputRes;
									const float4& data = cache.momentArray[offset];
									// Contribute
									mean += data.x;
									mean2 += data.y;
									minV = std::min(minV, data.z);
									maxV = std::max(maxV, data.w);
								}
							}
						}

						// Normalize the average
						uint64_t cacheOffset = cache.offsets[lvlIdx] + (uint64_t)x + (uint64_t)y * outputRes + (uint64_t)z * outputRes * outputRes;
						cache.momentArray[cacheOffset] = { mean * 0.125f, mean2 * 0.125f, minV, maxV };
					}
				}
			}

			inputRes = outputRes;
			outputRes = inputRes >> 1;
		}
	}

	float4 sample_cache(const HeuristicCache& cache, const float3& position, uint32_t depth)
	{
		int32_t cacheDepth = std::max((int32_t)(cache.numLevels - (depth - 4) / 3 - 1), 0);
		uint32_t resolution = cache.resolutions[cacheDepth];
		uint64_t offset = cache.offsets[cacheDepth];

		// Evalute the normalized positon
		float3 normPos = position + float3({ 0.5, 0.5, 0.5 });

		// Evalute the coords
		int64_t coordX = int64_t(normPos.x * resolution);
		int64_t coordY = int64_t(normPos.y * resolution);
		int64_t coordZ = int64_t(normPos.z * resolution);
		return cache.momentArray[offset + coordX + coordY * resolution + resolution * resolution * coordZ];
	}
}

