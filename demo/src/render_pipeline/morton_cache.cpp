// Internal includes
#include "render_pipeline/morton_cache.h"
#include "math/operators.h"

// External includes
#include <algorithm>

// Adds two empty bits every bit
uint64_t interleave_bits(uint32_t x)
{
    // Cap to 21 bits
    uint64_t v = x & 0x1fffff;
    v = (v | (v << 16)) & 0x0000003F0000FFFFull;
    v = (v | (v << 16)) & 0x003F0000FF0000FFull;
    v = (v | (v << 8)) & 0x300F00F00F00F00Full;
    v = (v | (v << 4)) & 0x30C30C30C30C30C3ull;
    v = (v | (v << 2)) & 0x9249249249249249ull;
    return v;
}

uint64_t morton_encode_3D(uint32_t x, uint32_t y, uint32_t z)
{
    uint64_t xBits = interleave_bits(x);
    uint64_t yBits = interleave_bits(y);
    uint64_t zBits = interleave_bits(z);
    return (xBits << 2) | (yBits << 1) | zBits;
}

struct
{
    bool operator()(const MortonCache::Element& a, const MortonCache::Element& b) const { return a.code < b.code; }
} ElementLess;

MortonCache::MortonCache()
{
}

MortonCache::~MortonCache()
{
}

uint64_t MortonCache::evaluate_morton_code(const float3& targetPosition)
{
    const float3& normPos = (targetPosition - m_MinPosition) / (m_MaxPosition - m_MinPosition);

    // Evaluate and store the morton code
    uint32_t cx = (uint32_t)(normPos.x * (1 << 21));
    uint32_t cy = (uint32_t)(normPos.y * (1 << 21));
    uint32_t cz = (uint32_t)(normPos.z * (1 << 21));
    return morton_encode_3D(cx, cy, cz);
}

void MortonCache::build_cache(const float3* positionArray, uint32_t numPositions)
{
    // Allocate space for the cache
    m_NumElements = numPositions;
	m_Cache.resize(numPositions);

    // First we need to find the min and max values of the range
    for (uint32_t idx = 0; idx < m_NumElements; ++idx)
    {
        // Contribute to the min max
        const float3& currentPos = positionArray[idx];
        m_MinPosition = min(currentPos, m_MinPosition);
        m_MaxPosition = max(currentPos, m_MaxPosition);
    }

    // Now generate the morton codes for every position
    for (uint32_t idx = 0; idx < m_NumElements; ++idx)
    {
        const float3& currentPos = positionArray[idx];
        m_Cache[idx] = { evaluate_morton_code(currentPos), idx };
    }

    // Sort the elements
    std::sort(m_Cache.begin(), m_Cache.end(), ElementLess);
}

uint32_t MortonCache::get_closest_element(const float3& position)
{
    // Evaluate the morton code
    const uint64_t& target = evaluate_morton_code(position);

    // Do the binary search
    int low = 0;
    int high = m_NumElements - 1;
    Element& closest = m_Cache[0];
    while (low <= high)
    {
        int mid = low + (high - low) / 2;

        // Update closest if the current element is closer to the target
        uint64_t leftT = m_Cache[mid].code > target ? m_Cache[mid].code - target : target - m_Cache[mid].code;
        uint64_t rightT = closest.code > target ? closest.code - target : target - closest.code;
        if (leftT < rightT)
            closest = m_Cache[mid];

        // Did we find it? Otherwise, search left or right
        if (m_Cache[mid].code == target)
            return m_Cache[mid].index; // Exact match found
        else if (m_Cache[mid].code < target)
            low = mid + 1; // Search in the right half
        else
            high = mid - 1; // Search in the left half
    }

    return closest.index;
}