#pragma once

// SDK includes
#include "math/types.h"
#include "rendering/aabb.h"

// Structure that describes a frustum plane
struct Plane
{
    float3 normal;
    float d;
};

struct Frustum
{
    Plane planes[6];
};

// Function that extracts the frustum planes from a view projection matrix
void extract_planes_from_view_projection_matrix(const float4x4 viewProj, Frustum& frustum);

// Intersect an aabb and a frustum
bool frustum_aabb_intersect(const Frustum& frustum, const AABB& aabb);