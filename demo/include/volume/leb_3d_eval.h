#pragma once

// Internal includes
#include "math/types.h"
#include "math/operators.h"

// External includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// LEB cache size
#define LEB_CACHE_SIZE 9

static uint32_t leb__FindMSB(uint64_t x)
{
    uint32_t depth = 0;

    while (x > 1u) {
        ++depth;
        x>>= 1u;
    }

    return depth;
}

static uint64_t leb__GetBitValue(const uint64_t bitField, int64_t bitID)
{
    return ((bitField >> bitID) & 1u);
}

static void leb__IdentityMatrix4x4(float4x4& m)
{
    m = { 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0 };
}


/*******************************************************************************
 * TransposeMatrix4x4 -- Transposes a 4x4 matrix
 *
 */
static void leb__TransposeMatrix4x4(const float4x4& m, float4x4& out)
{
    for (int64_t i = 0; i < 4; ++i)
    for (int64_t j = 0; j < 4; ++j)
        out.rc[i][j] = m.rc[j][i];
}


/*******************************************************************************
 * DotProduct -- Returns the dot product of two vectors
 *
 */
static float leb__DotProduct(int64_t argSize, const float *x, const float *y)
{
    float dp = 0.0f;

    for (int64_t i = 0; i < argSize; ++i)
        dp+= x[i] * y[i];

    return dp;
}


/*******************************************************************************
 * MulMatrix4x4 -- Computes the product of two 4x4 matrices
 *
 */
static void
leb__Matrix4x4Product(
    const float4x4& m1,
    const float4x4& m2,
    float4x4& out
) {
    float4x4 tra;
    leb__TransposeMatrix4x4(m2, tra);

    for (int64_t j = 0; j < 4; ++j)
    for (int64_t i = 0; i < 4; ++i)
        out.rc[j][i] = leb__DotProduct(4, m1.rc[j], tra.rc[i]);
}


/*******************************************************************************
 * SplittingMatrix -- Computes a LEB splitting matrix from a split bit
 *
 */
static void
leb__SplittingMatrix(float4x4& matrix, uint64_t bitValue, uint8_t type)
{
    float b = (float)bitValue;
    float c = 1.0f - b;

    if (type == 0)
    {
        const float4x4 splitMatrix = {
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f, 0.5f,
            b, 0.0f, c, 0.0,
            c, 0.0f, 0.0f, b
        };
        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(splitMatrix, tmp, matrix);
    }
    if (type == 3)
    {
        const float4x4 splitMatrix = {
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f, 0.5f,
            b, 0.0f, c, 0.0,
            c, 0.0f, 0.0f, b
        };
        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(splitMatrix, tmp, matrix);
    }
    else if (type == 1)
    {
        const float4x4 splitMatrix = {
            b, c, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f, 0.5f,
            0.0f, 0.0f, c, b,
            c, b, 0.0f, 0.0f
        };

        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(splitMatrix, tmp, matrix);
    }
    else if (type == 2)
    {
        const float4x4 splitMatrix = {
            c, b, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f, 0.5f,
            b, c, 0.0f, 0.0f,
            0.0f, 0.0f, c, b
        };

        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(splitMatrix, tmp, matrix);
    }
}

/*******************************************************************************
 * DecodeTransformationMatrix -- Computes the matrix associated to a LEB
 * node
 *
 */
static void
leb__DecodeTransformationMatrix(
    uint64_t heapID,
    uint8_t currentType,
    float4x4& matrix
) {
    uint64_t depth = leb__FindMSB(heapID);
    leb__IdentityMatrix4x4(matrix);

    for (int64_t bitID = depth - 1; bitID >= 0; --bitID) {
        uint64_t bitValue = leb__GetBitValue(heapID, bitID);
        leb__SplittingMatrix(matrix, leb__GetBitValue(heapID, bitID), currentType);
        switch (currentType)
        {
        case 0:
            currentType = bitValue == 0 ? 1 : 2;
            break;
        case 1:
        case 2:
            currentType = 3;
            break;
        case 3:
            currentType = 0;
            break;
        }
    }
}

static void leb__DecodeTransformationMatrix(uint64_t heapID, uint8_t, float4x4& matrix, const float4x4* matrixCache)
{
    leb__IdentityMatrix4x4(matrix);
    const uint64_t msb = (1ULL << LEB_CACHE_SIZE);
    const uint64_t mask = ~(~0ULL << LEB_CACHE_SIZE);
    uint64_t depth = leb__FindMSB(heapID);
    uint32_t remainder = depth % LEB_CACHE_SIZE;

    // Align on the power
    if (remainder != 0)
    {
        uint32_t index = (heapID & ((1ULL << remainder) - 1)) | (1ULL << remainder);
        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(tmp, matrixCache[index], matrix);
        heapID >>= remainder;
    }

    while (heapID > mask)
    {
        uint32_t index = uint32_t((heapID & mask) | msb);
        float4x4 tmp;
        memcpy(tmp.rc, matrix.rc, sizeof(tmp));
        leb__Matrix4x4Product(tmp, matrixCache[index], matrix);
        heapID >>= LEB_CACHE_SIZE;
    }
}

/*******************************************************************************
 * DecodeNodeAttributeArray -- Compute the triangle attributes at the input node
 *
 */
inline void leb_DecodeNodeAttributeArray(
    uint64_t heapID,
    uint8_t baseType,
    const float4x4* cache,
    float4 attributeArray[3]
) {
    float4x4 m;
    float attributeVector[4];

    // Evaluate the global matrix
    leb__DecodeTransformationMatrix(heapID, baseType, m, cache);

    // Apply it to each dimension
    for (int64_t i = 0; i < 3; ++i)
    {
        memcpy(attributeVector, &attributeArray[i].x, sizeof(attributeVector));
        attributeArray[i].x = leb__DotProduct(4, m.rc[0], attributeVector);
        attributeArray[i].y = leb__DotProduct(4, m.rc[1], attributeVector);
        attributeArray[i].z = leb__DotProduct(4, m.rc[2], attributeVector);
        attributeArray[i].w = leb__DotProduct(4, m.rc[3], attributeVector);
    }
}

inline void leb_DecodeNodeAttributeArray(
    uint64_t heapID,
    uint8_t baseType,
    float4 attributeArray[3]
) {
    float4x4 m;
    float attributeVector[4];

    // Evaluate the global matrix
    leb__DecodeTransformationMatrix(heapID, baseType, m);

    // Apply it to each dimension
    for (int64_t i = 0; i < 3; ++i)
    {
        memcpy(attributeVector, &attributeArray[i].x, sizeof(attributeVector));
        attributeArray[i].x = leb__DotProduct(4, m.rc[0], attributeVector);
        attributeArray[i].y = leb__DotProduct(4, m.rc[1], attributeVector);
        attributeArray[i].z = leb__DotProduct(4, m.rc[2], attributeVector);
        attributeArray[i].w = leb__DotProduct(4, m.rc[3], attributeVector);
    }
}