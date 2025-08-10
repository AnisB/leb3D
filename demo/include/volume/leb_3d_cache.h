#pragma once

// Internal includes
#include "math/types.h"

// External includes
#include <vector>

class Leb3DCache
{
public:
    // Cst & Dst
    Leb3DCache();
    ~Leb3DCache();
    const float4x4* get_cache() const{ return m_Cache.data(); }

private:
    uint32_t m_CacheDepth = 0;
    std::vector<float4x4> m_Cache = {};
};
