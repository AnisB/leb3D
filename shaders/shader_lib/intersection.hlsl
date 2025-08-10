#ifndef INTERSECTION_HLSL
#define INTERSECTION_HLSL

float3 evaluate_ray_direction(uint2 threadID)
{
    float3 depthBufferPosition = evaluate_world_space_position((threadID.xy + float2(0.5, 0.5)) * _ScreenSize.zw, 0.5, _InvViewProjectionMatrix);
    float distanceToCamera = length(depthBufferPosition);
    return depthBufferPosition / max(distanceToCamera, 0.000001);
}

float2 intersect_ray_aabb(float3 rayOrigin, float3 rayDir, float3 boxMin, float3 boxMax)
{
    float3 tMin = (boxMin - rayOrigin) / rayDir;
    float3 tMax = (boxMax - rayOrigin) / rayDir;
    float3 t1 = min(tMin, tMax);
    float3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);
    return float2(tNear, tFar);
};

float ray_triangle_intersection(float3 rayOrigin, float3 rayDirection, float3 p0, float3 p1, float3 p2)
{
    // Tolerance for avoiding floating-point precision issues
    float epsilon = 1e-10;

    // Edge vectors
    float3 edge1 = p1 - p0;
    float3 edge2 = p2 - p0;

    // Begin calculating determinant - also used to calculate U parameter
    float3 pvec = cross(rayDirection, edge2);

    // If determinant is near zero, ray lies in plane of triangle
    float det = dot(edge1, pvec);

    if (det > -epsilon && det < epsilon)
        return -1.0;

    float invDet = 1.0 / det;

    // Calculate distance from v0 to ray origin
    float3 tvec = rayOrigin - p0;

    // Calculate U parameter and test bounds
    float u = dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0)
        return -1.0;

    // Prepare to test V parameter
    float3 qvec = cross(tvec, edge1);

    // Calculate V parameter and test bounds
    float v = dot(rayDirection, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0)
        return -1.0;

    // Calculate t, ray intersects triangle
    float t = dot(edge2, qvec) * invDet;

    // If t is negative, the intersection is behind the ray origin
    if (t < 0)
        return -1.0;
    return t;
}

float ray_box_intersection(float3 rayOrigin, float3 rayDir, float3 boxMin, float3 boxMax)
{
    float3 invDir = 1.0 / rayDir;
    float3 t0 = (boxMin - rayOrigin) * invDir;
    float3 t1 = (boxMax - rayOrigin) * invDir;

    float3 tMinVec = min(t0, t1);
    float3 tMaxVec = max(t0, t1);

    float tMin = max(tMinVec.x, max(tMinVec.y, tMinVec.z));
    float tMax = min(tMaxVec.x, min(tMaxVec.y, tMaxVec.z));

    if (tMax < tMin || tMax < 0.0)
        return -1.0;

    // Select the first positive intersection
    return (tMin > 0.0) ? tMin : tMax;
}

float signed_volume(float3 p1, float3 p2, float3 p3, float3 p4)
{
    return (1.0f / 6.0f) * (
        (p1.x - p4.x) * (p2.y - p4.y) * (p3.z - p4.z) +
        (p2.x - p4.x) * (p3.y - p4.y) * (p1.z - p4.z) +
        (p3.x - p4.x) * (p1.y - p4.y) * (p2.z - p4.z) -
        (p3.x - p4.x) * (p2.y - p4.y) * (p1.z - p4.z) -
        (p2.x - p4.x) * (p1.y - p4.y) * (p3.z - p4.z) -
        (p1.x - p4.x) * (p3.y - p4.y) * (p2.z - p4.z)
        );
}

// Function to check if a point is inside a tetrahedron
int point_inside_tetrahedron(float3 p, float3 v0, float3 v1, float3 v2, float3 v3)
{
    float v0v1v2v3 = signed_volume(v0, v1, v2, v3);
    float pv1v2v3 = signed_volume(p, v1, v2, v3);
    float v0pv2v3 = signed_volume(v0, p, v2, v3);
    float v0v1pv3 = signed_volume(v0, v1, p, v3);
    float v0v1v2p = signed_volume(v0, v1, v2, p);
    return (v0v1v2v3 >= 0 && pv1v2v3 >= 0 && v0pv2v3 >= 0 && v0v1pv3 >= 0 && v0v1v2p >= 0) ||
        (v0v1v2v3 <= 0 && pv1v2v3 <= 0 && v0pv2v3 <= 0 && v0v1pv3 <= 0 && v0v1v2p <= 0);
}

float ray_plane_intersection(float3 rayOrigin, float3 rayDirection, float3 planeNormal, float planeOffset)
{
    float denom = dot(rayDirection, planeNormal);
    if (denom < 1e-6)
        return FLT_MAX;
    float t = -(dot(rayOrigin, planeNormal) + planeOffset);
    return t > -1e-6 ? t / denom : FLT_MAX;
}

float ray_sphere_intersect_nearest(float3 r0, float3 rd, float3 s0, float sR)
{
    float a = dot(rd, rd);
    float3 s0_r0 = r0 - s0;
    float b = 2.0 * dot(rd, s0_r0);
    float c = dot(s0_r0, s0_r0) - (sR * sR);
    float delta = b * b - 4.0*a*c;
    if (delta < 0.0 || a == 0.0)
        return -1.0;
    float sol0 = (-b - sqrt(delta)) / (2.0*a);
    float sol1 = (-b + sqrt(delta)) / (2.0*a);
    if (sol0 < 0.0 && sol1 < 0.0)
        return -1.0;
    if (sol0 < 0.0)
        return max(0.0, sol1);
    else if (sol1 < 0.0)
        return max(0.0, sol0);
    return max(0.0, min(sol0, sol1));
}

#endif // INTERSECTION_HLSL