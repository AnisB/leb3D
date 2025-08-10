// Project includes
#include "rendering/frustum.h"
#include "math/operators.h"

void normalize_plane(Plane& plane)
{
    float l = sqrtf(plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y + plane.normal.z * plane.normal.z);
    plane.normal.x /= l;
    plane.normal.y /= l;
    plane.normal.z /= l;
    plane.d /= l;
}

void extract_planes_from_view_projection_matrix(const float4x4 viewProj, Frustum& frustum)
{
    // Compute all the planes
    frustum.planes[0] = {viewProj.m[0] + viewProj.m[3], viewProj.m[4] + viewProj.m[7], viewProj.m[8] + viewProj.m[11], viewProj.m[12] + viewProj.m[15]};
    frustum.planes[1] = {-viewProj.m[0] + viewProj.m[3], -viewProj.m[4] + viewProj.m[7], -viewProj.m[8] + viewProj.m[11], -viewProj.m[12] + viewProj.m[15]};

    frustum.planes[2] = {viewProj.m[1] + viewProj.m[3], viewProj.m[5] + viewProj.m[7], viewProj.m[9] + viewProj.m[11], viewProj.m[13] + viewProj.m[15]};
    frustum.planes[3] = {-viewProj.m[1] + viewProj.m[3], -viewProj.m[5] + viewProj.m[7], -viewProj.m[9] + viewProj.m[11], -viewProj.m[13] + viewProj.m[15]};

    frustum.planes[4] = {viewProj.m[2] + viewProj.m[3], viewProj.m[6] + viewProj.m[7], viewProj.m[10] + viewProj.m[11], viewProj.m[14] + viewProj.m[15]};
    frustum.planes[5] = {-viewProj.m[2] + viewProj.m[3], -viewProj.m[6] + viewProj.m[7], -viewProj.m[10] + viewProj.m[11], -viewProj.m[14] + viewProj.m[15]};

    // Normalize all the planes
    for (uint32_t planeIdx = 0; planeIdx < 6; ++planeIdx)
        normalize_plane(frustum.planes[planeIdx]);
}

bool frustum_aabb_intersect(const Frustum& frustum, const AABB& aabb)
{
   const float3& center = (aabb.max + aabb.min) * 0.5;
   const float3& extents = (aabb.max - aabb.min) * 0.5;
    for (int i = 0; i < 4; i++)
    {
        const Plane& plane = frustum.planes[i];
        const float3& normal_sign = sign(plane.normal);
        const float3& test_point = center + extents * normal_sign;
 
        float dotProd = dot(test_point, plane.normal);
        if (dotProd + plane.d < 0)
            return false;
    }
    return true;
}