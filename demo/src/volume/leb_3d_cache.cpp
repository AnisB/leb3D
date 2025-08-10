// Internal includes
#include "volume/leb_3d_eval.h"
#include "volume/leb_3d_cache.h"
#include "math/operators.h"

Leb3DCache::Leb3DCache()
{
    // Keep the cache depth
    m_CacheDepth = LEB_CACHE_SIZE;
    uint32_t matrixCount = 2ULL << m_CacheDepth;
    m_Cache.resize(matrixCount);

    // Build the CPU table
    leb__IdentityMatrix4x4(m_Cache[0]);

    // Build the caches
    for (uint64_t heapID = 1ULL; heapID < (2ULL << m_CacheDepth); ++heapID)
    {
        float4x4 mat;
        leb__DecodeTransformationMatrix(heapID, 0, mat);
        m_Cache[heapID] = mat;
    }
}

Leb3DCache::~Leb3DCache()
{
}
