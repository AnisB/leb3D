// Internal includes
#include "volume/volume_generation.h"
#include "volume/grid_volume.h"
#include "volume/leb_3d_cache.h"
#include "tools/security.h"
#include "math/operators.h"
#include "rendering/frustum.h"
#include "rendering/aabb.h"

namespace leb_volume
{
    // Forward declarations
    void recursive_split_element(LEBVolume& volume, uint32_t targetElement);

    // Function that allocates new elements
    uint32_t allocate_new_elements(LEBVolume& volume, uint32_t numElements)
    {
        uint32_t prevElementCount = volume.totalNumElements;
        volume.heapIDArray.resize(prevElementCount + numElements);
        volume.typeArray.resize(prevElementCount + numElements);
        volume.neighborsArray.resize(prevElementCount + numElements);
        volume.modifArray.resize(prevElementCount + numElements);
        volume.depthArray.resize(prevElementCount + numElements);
        volume.tetraCacheArray.resize(prevElementCount + numElements);
        volume.totalNumElements += numElements;
        return prevElementCount;
    }

    void patch_neighbor(LEBVolume& cpuVolume, uint32_t target, uint32_t oldRef, uint32_t newRef)
    {
        uint4& neighbors = cpuVolume.neighborsArray[target];
        if (neighbors.x == oldRef)
            neighbors.x = newRef;
        if (neighbors.y == oldRef)
            neighbors.y = newRef;
        if (neighbors.z == oldRef)
            neighbors.z = newRef;
        if (neighbors.w == oldRef)
            neighbors.w = newRef;
    }

    bool check_diamond(LEBVolume& volume, uint32_t targetElement, uint32_t& rightElements, uint32_t& leftElements)
    {
        // Reset the counters
        rightElements = 0;
        leftElements = 0;

        // Get the type of the current element (this one will not be divded
        uint8_t currentType = volume.typeArray[targetElement];

        // Initialize the loop
        uint32_t prevElement = targetElement;
        uint32_t currentElement = volume.neighborsArray[prevElement].z;
        bool firstTwin = true;

        // Loop till we get back to the initial element
        while (currentElement != targetElement && currentElement != UINT32_MAX)
        {
            rightElements++;
            uint8_t type = volume.typeArray[currentElement];
            if (!equivalent_types(type, currentType))
            {
                // We divide it
                recursive_split_element(volume, currentElement);

                // We need to find the index of the new element of the diamond (that potentially replaced this one
                if (firstTwin)
                    currentElement = volume.neighborsArray[prevElement].z;
                else
                    currentElement = volume.neighborsArray[prevElement].w;

                // Make sure this one is valid
                assert(currentElement != UINT32_MAX);
            }

            // Let's move to the next element
            if (volume.neighborsArray[currentElement].z == prevElement)
            {
                firstTwin = false;
                prevElement = currentElement;
                currentElement = volume.neighborsArray[currentElement].w;
            }
            else
            {
                firstTwin = true;
                prevElement = currentElement;
                currentElement = volume.neighborsArray[currentElement].z;
            }
        }

        // This means there is a discontinuity on the other side, we'll loop on this side
        if (currentElement != targetElement)
        {
            prevElement = targetElement;
            currentElement = volume.neighborsArray[targetElement].w;
            firstTwin = false;
            while (currentElement != UINT32_MAX)
            {
                leftElements++;
                uint8_t type = volume.typeArray[currentElement];
                if (!equivalent_types(type, currentType))
                {
                    // We divide it
                    recursive_split_element(volume, currentElement);

                    // We need to find the index of the new element of the diamond (that potentially replaced this one
                    uint4 newNeighbors = volume.neighborsArray[prevElement];
                    if (firstTwin)
                        currentElement = newNeighbors.z;
                    else
                        currentElement = newNeighbors.w;

                    // Make sure this one is valid
                    assert(currentElement != UINT32_MAX);
                }

                // Let's move to the next element
                if (volume.neighborsArray[currentElement].z == prevElement)
                {
                    firstTwin = false;
                    prevElement = currentElement;
                    currentElement = volume.neighborsArray[currentElement].w;
                }
                else
                {
                    firstTwin = true;
                    prevElement = currentElement;
                    currentElement = volume.neighborsArray[currentElement].z;
                }
            }
            return false;
        }
        else
            return true;
    }

    void diamond_split(LEBVolume& volume, uint32_t targetElement)
    {
        // Get the type of the current element
        uint8_t currentType = volume.typeArray[targetElement];

        // Split the diamonds
        // 8 elements to process here
        if (currentType == 0)
        {
            // Retrieve the 4 elements
            uint32_t i0 = targetElement;
            uint32_t i1 = volume.neighborsArray[targetElement].z;
            uint32_t i2 = volume.neighborsArray[i1].w;
            uint32_t i3 = volume.neighborsArray[i2].z;
            uint32_t i4 = volume.neighborsArray[i3].w;
            uint32_t i5 = volume.neighborsArray[i4].z;
            uint32_t i6 = volume.neighborsArray[i5].w;
            uint32_t i7 = volume.neighborsArray[i6].z;

            // Just making sure we did the round trip
            assert(volume.neighborsArray[i0].w == i7);

            // Make sure all the types are the right ones
            assert(volume.typeArray[i0] == 0);
            assert(volume.typeArray[i1] == 0);
            assert(volume.typeArray[i2] == 0);
            assert(volume.typeArray[i3] == 0);
            assert(volume.typeArray[i4] == 0);
            assert(volume.typeArray[i5] == 0);
            assert(volume.typeArray[i6] == 0);
            assert(volume.typeArray[i7] == 0);

            // Allocate all the slots
            uint32_t slot0 = allocate_new_elements(volume, 8);
            uint32_t slot1 = slot0 + 1;
            uint32_t slot2 = slot1 + 1;
            uint32_t slot3 = slot2 + 1;
            uint32_t slot4 = slot3 + 1;
            uint32_t slot5 = slot4 + 1;
            uint32_t slot6 = slot5 + 1;
            uint32_t slot7 = slot6 + 1;

            // A son is not more dividable it's father
            volume.modifArray[i0] = (volume.modifArray[i0] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i1] = (volume.modifArray[i1] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i2] = (volume.modifArray[i2] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i3] = (volume.modifArray[i3] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i4] = (volume.modifArray[i4] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i5] = (volume.modifArray[i5] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i6] = (volume.modifArray[i6] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i7] = (volume.modifArray[i7] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;

            // A son is not more dividable it's father
            volume.modifArray[slot0] = volume.modifArray[i0];
            volume.modifArray[slot1] = volume.modifArray[i1];
            volume.modifArray[slot2] = volume.modifArray[i2];
            volume.modifArray[slot3] = volume.modifArray[i3];
            volume.modifArray[slot4] = volume.modifArray[i4];
            volume.modifArray[slot5] = volume.modifArray[i5];
            volume.modifArray[slot6] = volume.modifArray[i6];
            volume.modifArray[slot7] = volume.modifArray[i7];

            // Let's evaluate all the heap IDs
            volume.heapIDArray[i0] = volume.heapIDArray[i0] * 2;
            volume.heapIDArray[slot0] = volume.heapIDArray[i0] + 1;

            volume.heapIDArray[i1] = volume.heapIDArray[i1] * 2;
            volume.heapIDArray[slot1] = volume.heapIDArray[i1] + 1;

            volume.heapIDArray[i2] = volume.heapIDArray[i2] * 2;
            volume.heapIDArray[slot2] = volume.heapIDArray[i2] + 1;

            volume.heapIDArray[i3] = volume.heapIDArray[i3] * 2;
            volume.heapIDArray[slot3] = volume.heapIDArray[i3] + 1;

            volume.heapIDArray[i4] = volume.heapIDArray[i4] * 2;
            volume.heapIDArray[slot4] = volume.heapIDArray[i4] + 1;

            volume.heapIDArray[i5] = volume.heapIDArray[i5] * 2;
            volume.heapIDArray[slot5] = volume.heapIDArray[i5] + 1;

            volume.heapIDArray[i6] = volume.heapIDArray[i6] * 2;
            volume.heapIDArray[slot6] = volume.heapIDArray[i6] + 1;

            volume.heapIDArray[i7] = volume.heapIDArray[i7] * 2;
            volume.heapIDArray[slot7] = volume.heapIDArray[i7] + 1;

            // Update the types
            volume.typeArray[i0] = 1;
            volume.typeArray[slot0] = 2;

            volume.typeArray[i1] = 1;
            volume.typeArray[slot1] = 2;

            volume.typeArray[i2] = 1;
            volume.typeArray[slot2] = 2;

            volume.typeArray[i3] = 1;
            volume.typeArray[slot3] = 2;

            volume.typeArray[i4] = 1;
            volume.typeArray[slot4] = 2;

            volume.typeArray[i5] = 1;
            volume.typeArray[slot5] = 2;

            volume.typeArray[i6] = 1;
            volume.typeArray[slot6] = 2;

            volume.typeArray[i7] = 1;
            volume.typeArray[slot7] = 2;

            // Now let's do the pointer dance
            uint4 nI0 = volume.neighborsArray[i0];
            uint4 nI1 = volume.neighborsArray[i1];
            uint4 nI2 = volume.neighborsArray[i2];
            uint4 nI3 = volume.neighborsArray[i3];
            uint4 nI4 = volume.neighborsArray[i4];
            uint4 nI5 = volume.neighborsArray[i5];
            uint4 nI6 = volume.neighborsArray[i6];
            uint4 nI7 = volume.neighborsArray[i7];

            // Schemes
            // UP (1 0 -1 / 0 1 -1)
            // DOWN (-1 0 1 / 0 -1 1) 

            // Set all the connections inside the loop
            volume.neighborsArray[i0] = { slot1, slot0, slot7, nI0.x };
            volume.neighborsArray[slot0] = { i0, i1, i7, nI0.y };

            volume.neighborsArray[i1] = { slot0, slot1, slot2 , nI1.x };
            volume.neighborsArray[slot1] = { i1, i0, i2, nI1.y };

            volume.neighborsArray[i2] = { slot3, slot2, slot1, nI2.x };
            volume.neighborsArray[slot2] = { i2, i3, i1, nI2.y };

            volume.neighborsArray[i3] = { slot2, slot3, slot4, nI3.x };
            volume.neighborsArray[slot3] = { i3, i2, i4, nI3.y };

            volume.neighborsArray[i4] = { slot5, slot4, slot3, nI4.x };
            volume.neighborsArray[slot4] = { i4, i5, i3, nI4.y };

            volume.neighborsArray[i5] = { slot4, slot5, slot6, nI5.x };
            volume.neighborsArray[slot5] = { i5, i4, i6, nI5.y };

            volume.neighborsArray[i6] = { slot7, slot6, slot5, nI6.x };
            volume.neighborsArray[slot6] = { i6, i7, i5, nI6.y };

            volume.neighborsArray[i7] = { slot6, slot7, slot0, nI7.x };
            volume.neighborsArray[slot7] = { i7, i6, i0, nI7.y };

            // Patch the neighbors
            if (nI0.y != UINT32_MAX)
                patch_neighbor(volume, nI0.y, i0, slot0);
            if (nI1.y != UINT32_MAX)
                patch_neighbor(volume, nI1.y, i1, slot1);
            if (nI2.y != UINT32_MAX)
                patch_neighbor(volume, nI2.y, i2, slot2);
            if (nI3.y != UINT32_MAX)
                patch_neighbor(volume, nI3.y, i3, slot3);
            if (nI4.y != UINT32_MAX)
                patch_neighbor(volume, nI4.y, i4, slot4);
            if (nI5.y != UINT32_MAX)
                patch_neighbor(volume, nI5.y, i5, slot5);
            if (nI6.y != UINT32_MAX)
                patch_neighbor(volume, nI6.y, i6, slot6);
            if (nI7.y != UINT32_MAX)
                patch_neighbor(volume, nI7.y, i7, slot7);

            // Keep track of this diamond
            Diamond diamond;
            diamond.size = 8;
            diamond.heapID[0] = volume.heapIDArray[i0] / 2;
            diamond.heapID[1] = volume.heapIDArray[i1] / 2;
            diamond.heapID[2] = volume.heapIDArray[i2] / 2;
            diamond.heapID[3] = volume.heapIDArray[i3] / 2;
            diamond.heapID[4] = volume.heapIDArray[i4] / 2;
            diamond.heapID[5] = volume.heapIDArray[i5] / 2;
            diamond.heapID[6] = volume.heapIDArray[i6] / 2;
            diamond.heapID[7] = volume.heapIDArray[i7] / 2;
            volume.diamonds.push_back(diamond);
        }
        // 6 elements to process here
        else if (currentType == 1 || currentType == 2)
        {
            // Retrieve the 4 elements
            uint32_t i0 = targetElement;

            uint32_t i1 = volume.neighborsArray[i0].z;
            bool rev1 = volume.neighborsArray[i1].z == i0;

            uint32_t i2 = rev1 ? volume.neighborsArray[i1].w : volume.neighborsArray[i1].z;
            bool rev2 = volume.neighborsArray[i2].z == i1;

            uint32_t i3 = rev2 ? volume.neighborsArray[i2].w : volume.neighborsArray[i2].z;
            bool rev3 = volume.neighborsArray[i3].z == i2;

            uint32_t i4 = rev3 ? volume.neighborsArray[i3].w : volume.neighborsArray[i3].z;
            bool rev4 = volume.neighborsArray[i4].z == i3;

            uint32_t i5 = rev4 ? volume.neighborsArray[i4].w : volume.neighborsArray[i4].z;
            bool rev5 = volume.neighborsArray[i5].z == i4;

            // Just making sure we did the round trip
            assert(volume.neighborsArray[i0].w == i5);

            // Make sure all the types are the right ones
            assert(volume.typeArray[i0] == 1 || volume.typeArray[i0] == 2);
            assert(volume.typeArray[i1] == 1 || volume.typeArray[i1] == 2);
            assert(volume.typeArray[i2] == 1 || volume.typeArray[i2] == 2);
            assert(volume.typeArray[i3] == 1 || volume.typeArray[i3] == 2);
            assert(volume.typeArray[i4] == 1 || volume.typeArray[i4] == 2);
            assert(volume.typeArray[i5] == 1 || volume.typeArray[i5] == 2);

            // Allocate all the slots
            uint32_t slot0 = allocate_new_elements(volume, 6);
            uint32_t slot1 = slot0 + 1;
            uint32_t slot2 = slot1 + 1;
            uint32_t slot3 = slot2 + 1;
            uint32_t slot4 = slot3 + 1;
            uint32_t slot5 = slot4 + 1;

            volume.modifArray[i0] = (volume.modifArray[i0] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i1] = (volume.modifArray[i1] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i2] = (volume.modifArray[i2] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i3] = (volume.modifArray[i3] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i4] = (volume.modifArray[i4] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i5] = (volume.modifArray[i5] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;

            // A son is not more dividable it's father
            volume.modifArray[slot0] = volume.modifArray[i0];
            volume.modifArray[slot1] = volume.modifArray[i1];
            volume.modifArray[slot2] = volume.modifArray[i2];
            volume.modifArray[slot3] = volume.modifArray[i3];
            volume.modifArray[slot4] = volume.modifArray[i4];
            volume.modifArray[slot5] = volume.modifArray[i5];

            // Let's evaluate all the heap IDs
            volume.heapIDArray[i0] = volume.heapIDArray[i0] * 2;
            volume.heapIDArray[slot0] = volume.heapIDArray[i0] + 1;

            volume.heapIDArray[i1] = volume.heapIDArray[i1] * 2;
            volume.heapIDArray[slot1] = volume.heapIDArray[i1] + 1;

            volume.heapIDArray[i2] = volume.heapIDArray[i2] * 2;
            volume.heapIDArray[slot2] = volume.heapIDArray[i2] + 1;

            volume.heapIDArray[i3] = volume.heapIDArray[i3] * 2;
            volume.heapIDArray[slot3] = volume.heapIDArray[i3] + 1;

            volume.heapIDArray[i4] = volume.heapIDArray[i4] * 2;
            volume.heapIDArray[slot4] = volume.heapIDArray[i4] + 1;

            volume.heapIDArray[i5] = volume.heapIDArray[i5] * 2;
            volume.heapIDArray[slot5] = volume.heapIDArray[i5] + 1;

            // Now let's do the pointer dance
            uint4 nI0 = volume.neighborsArray[i0];
            uint4 nI1 = volume.neighborsArray[i1];
            uint4 nI2 = volume.neighborsArray[i2];
            uint4 nI3 = volume.neighborsArray[i3];
            uint4 nI4 = volume.neighborsArray[i4];
            uint4 nI5 = volume.neighborsArray[i5];

            // Set all the connections inside the loop
            // Scheme
            // Normal +1 0 -1 / -1 0 1 -- 0 -1 1 / 0 1 -1
            // Rev - 1 0 1 / 1 0 -1 -- 0 1 -1 / 0 -1 1
            if (volume.typeArray[i0] == 1)
            {
                volume.neighborsArray[i0] = { slot1, slot0, slot5, nI0.x };
                volume.neighborsArray[slot0] = { i5, i0, i1, nI0.y };
            }
            else
            {
                volume.neighborsArray[i0] = { slot0, slot5, slot1, nI0.x };
                volume.neighborsArray[slot0] = { i0, i1, i5, nI0.y };
            }

            if (rev1)
            {
                if (volume.typeArray[i1] == 1)
                {
                    volume.neighborsArray[i1] = { slot0, slot1, slot2, nI1.x };
                    volume.neighborsArray[slot1] = { i2, i1, i0, nI1.y };
                }
                else
                {
                    volume.neighborsArray[i1] = { slot1, slot2, slot0, nI1.x };
                    volume.neighborsArray[slot1] = { i1, i0, i2, nI1.y };
                }
            }
            else
            {
                if (volume.typeArray[i1] == 1)
                {
                    volume.neighborsArray[i1] = { slot2, slot1, slot0, nI1.x };
                    volume.neighborsArray[slot1] = { i2, i1, i0, nI1.y };
                }
                else
                {
                    volume.neighborsArray[i1] = { slot1, slot0, slot2, nI1.x };
                    volume.neighborsArray[slot1] = { i1, i0, i2, nI1.y };
                }
            }

            if (volume.typeArray[i2] == 1)
            {
                volume.neighborsArray[i2] = { slot3, slot2, slot1, nI2.x };
                volume.neighborsArray[slot2] = { i1, i2, i3, nI2.y };
            }
            else
            {
                volume.neighborsArray[i2] = { slot2, slot1, slot3, nI2.x };
                volume.neighborsArray[slot2] = { i2, i3, i1, nI2.y };
            }

            if (rev3)
            {
                if (volume.typeArray[i3] == 1)
                {
                    volume.neighborsArray[i3] = { slot2, slot3, slot4, nI3.x };
                    volume.neighborsArray[slot3] = { i4, i3, i2, nI3.y };
                }
                else
                {
                    volume.neighborsArray[i3] = { slot3, slot4, slot2, nI3.x };
                    volume.neighborsArray[slot3] = { i3, i2, i4, nI3.y };
                }
            }
            else
            {
                if (volume.typeArray[i3] == 1)
                {
                    volume.neighborsArray[i3] = { slot4, slot3, slot2, nI3.x };
                    volume.neighborsArray[slot3] = { i4, i3, i2, nI3.y };
                }
                else
                {
                    volume.neighborsArray[i3] = { slot3, slot2, slot4, nI3.x };
                    volume.neighborsArray[slot3] = { i3, i2, i4, nI3.y };
                }
            }

            if (volume.typeArray[i4] == 1)
            {
                volume.neighborsArray[i4] = { slot5, slot4, slot3, nI4.x };
                volume.neighborsArray[slot4] = { i3, i4, i5, nI4.y };
            }
            else
            {
                volume.neighborsArray[i4] = { slot4, slot3, slot5, nI4.x };
                volume.neighborsArray[slot4] = { i4, i5, i3, nI4.y };
            }

            if (rev5)
            {
                if (volume.typeArray[i5] == 1)
                {
                    volume.neighborsArray[i5] = { slot4, slot5, slot0, nI5.x };
                    volume.neighborsArray[slot5] = { i0, i5, i4, nI5.y };
                }
                else
                {
                    volume.neighborsArray[i5] = { slot5, slot0, slot4, nI5.x };
                    volume.neighborsArray[slot5] = { i5, i4, i0, nI5.y };
                }
            }
            else
            {
                if (volume.typeArray[i5] == 1)
                {
                    volume.neighborsArray[i5] = { slot0, slot5, slot4, nI5.x };
                    volume.neighborsArray[slot5] = { i0, i5, i4, nI5.y };
                }
                else
                {
                    volume.neighborsArray[i5] = { slot5, slot4, slot0, nI5.x };
                    volume.neighborsArray[slot5] = { i5, i4, i0, nI5.y };
                }
            }

            // Update the types
            volume.typeArray[i0] = 3;
            volume.typeArray[slot0] = 3;

            volume.typeArray[i1] = 3;
            volume.typeArray[slot1] = 3;

            volume.typeArray[i2] = 3;
            volume.typeArray[slot2] = 3;

            volume.typeArray[i3] = 3;
            volume.typeArray[slot3] = 3;

            volume.typeArray[i4] = 3;
            volume.typeArray[slot4] = 3;

            volume.typeArray[i5] = 3;
            volume.typeArray[slot5] = 3;

            // Patch the neighbors
            if (nI0.y != UINT32_MAX)
                patch_neighbor(volume, nI0.y, i0, slot0);
            if (nI1.y != UINT32_MAX)
                patch_neighbor(volume, nI1.y, i1, slot1);
            if (nI2.y != UINT32_MAX)
                patch_neighbor(volume, nI2.y, i2, slot2);
            if (nI3.y != UINT32_MAX)
                patch_neighbor(volume, nI3.y, i3, slot3);
            if (nI4.y != UINT32_MAX)
                patch_neighbor(volume, nI4.y, i4, slot4);
            if (nI5.y != UINT32_MAX)
                patch_neighbor(volume, nI5.y, i5, slot5);

            // Keep track of this diamond
            Diamond diamond;
            diamond.size = 6;
            diamond.heapID[0] = volume.heapIDArray[i0] / 2;
            diamond.heapID[1] = volume.heapIDArray[i1] / 2;
            diamond.heapID[2] = volume.heapIDArray[i2] / 2;
            diamond.heapID[3] = volume.heapIDArray[i3] / 2;
            diamond.heapID[4] = volume.heapIDArray[i4] / 2;
            diamond.heapID[5] = volume.heapIDArray[i5] / 2;
            volume.diamonds.push_back(diamond);
        }
        // 4 elements to process here
        else if (currentType == 3)
        {
            // Retrieve the 4 elements
            uint32_t i0 = targetElement;
            uint32_t i1 = volume.neighborsArray[targetElement].z;
            uint32_t i2 = volume.neighborsArray[i1].w;
            uint32_t i3 = volume.neighborsArray[i2].z;

            // Make sure all the types are the right ones
            assert(volume.typeArray[i0] == 3);
            assert(volume.typeArray[i1] == 3);
            assert(volume.typeArray[i2] == 3);
            assert(volume.typeArray[i3] == 3);

            // Allocate all the slots
            uint32_t slot0 = allocate_new_elements(volume, 4);
            uint32_t slot1 = slot0 + 1;
            uint32_t slot2 = slot1 + 1;
            uint32_t slot3 = slot2 + 1;

            volume.modifArray[i0] = (volume.modifArray[i0] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i1] = (volume.modifArray[i1] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i2] = (volume.modifArray[i2] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
            volume.modifArray[i3] = (volume.modifArray[i3] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;

            volume.modifArray[slot0] = volume.modifArray[i0];
            volume.modifArray[slot1] = volume.modifArray[i1];
            volume.modifArray[slot2] = volume.modifArray[i2];
            volume.modifArray[slot3] = volume.modifArray[i3];

            // Let's evaluate all the heap IDs
            volume.heapIDArray[i0] = volume.heapIDArray[i0] * 2;
            volume.heapIDArray[slot0] = volume.heapIDArray[i0] + 1;
            volume.heapIDArray[i1] = volume.heapIDArray[i1] * 2;
            volume.heapIDArray[slot1] = volume.heapIDArray[i1] + 1;
            volume.heapIDArray[i2] = volume.heapIDArray[i2] * 2;
            volume.heapIDArray[slot2] = volume.heapIDArray[i2] + 1;
            volume.heapIDArray[i3] = volume.heapIDArray[i3] * 2;
            volume.heapIDArray[slot3] = volume.heapIDArray[i3] + 1;

            // Update the types
            volume.typeArray[i0] = 0;
            volume.typeArray[slot0] = 0;
            volume.typeArray[i1] = 0;
            volume.typeArray[slot1] = 0;
            volume.typeArray[i2] = 0;
            volume.typeArray[slot2] = 0;
            volume.typeArray[i3] = 0;
            volume.typeArray[slot3] = 0;

            // Now let's do the pointer dance
            uint4 nI0 = volume.neighborsArray[i0];
            uint4 nI1 = volume.neighborsArray[i1];
            uint4 nI2 = volume.neighborsArray[i2];
            uint4 nI3 = volume.neighborsArray[i3];

            // Set all the connections inside the loop
            volume.neighborsArray[i0] = { slot1, slot0, slot3, nI0.x };
            volume.neighborsArray[slot0] = { i0, i1, i3, nI0.y };

            volume.neighborsArray[i1] = { slot0, slot1, slot2 , nI1.x };
            volume.neighborsArray[slot1] = { i1, i0, i2, nI1.y };

            volume.neighborsArray[i2] = { slot3, slot2, slot1, nI2.x };
            volume.neighborsArray[slot2] = { i2, i3, i1, nI2.y };

            volume.neighborsArray[i3] = { slot2, slot3, slot0, nI3.x };
            volume.neighborsArray[slot3] = { i3, i2, i0, nI3.y };

            // Patch the neighbors
            if (nI0.y != UINT32_MAX)
                patch_neighbor(volume, nI0.y, i0, slot0);
            if (nI1.y != UINT32_MAX)
                patch_neighbor(volume, nI1.y, i1, slot1);
            if (nI2.y != UINT32_MAX)
                patch_neighbor(volume, nI2.y, i2, slot2);
            if (nI3.y != UINT32_MAX)
                patch_neighbor(volume, nI3.y, i3, slot3);

            // Keep track of this diamond
            Diamond diamond;
            diamond.size = 4;
            diamond.heapID[0] = volume.heapIDArray[i0] / 2;
            diamond.heapID[1] = volume.heapIDArray[i1] / 2;
            diamond.heapID[2] = volume.heapIDArray[i2] / 2;
            diamond.heapID[3] = volume.heapIDArray[i3] / 2;
            volume.diamonds.push_back(diamond);
        }
    }

    void diamond_split_incomplete(LEBVolume& volume, uint32_t targetElement, uint32_t rightElements, uint32_t leftElements)
    {
        // Get the type of the current element
        uint8_t currentType = volume.typeArray[targetElement];

        // Define our diamonds
        uint32_t diamondSize = 1 + rightElements + leftElements;
        uint32_t elementIndex = leftElements;

        // Split the diamonds
        // 8 elements to process here
        if (currentType == 0)
        {
            // Buffer that holds all the valid indices of the diamond
            uint32_t indices[8] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };
            uint32_t slots[8] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };

            // Our central element
            indices[elementIndex] = targetElement;
            slots[elementIndex] = allocate_new_elements(volume, 1);

            // Set all the left elements
            bool firstTwin = true;
            for (uint32_t eleIdx = 0; eleIdx < leftElements; ++eleIdx)
            {
                // Previous index
                uint32_t prevIndex = indices[elementIndex - eleIdx];

                // Get the following index
                indices[elementIndex - eleIdx - 1] = firstTwin ? volume.neighborsArray[prevIndex].w : volume.neighborsArray[prevIndex].z;

                // Allocate the slot
                slots[elementIndex - eleIdx - 1] = allocate_new_elements(volume, 1);

                // Reverse the twin
                firstTwin = !firstTwin;
            }

            // Set all the left elements
            firstTwin = true;
            for (uint32_t eleIdx = 0; eleIdx < rightElements; ++eleIdx)
            {
                // Previous index
                uint32_t prevIndex = indices[elementIndex + eleIdx];

                // Get the following index
                indices[elementIndex + eleIdx + 1] = firstTwin ? volume.neighborsArray[prevIndex].z : volume.neighborsArray[prevIndex].w;

                // Allocate the slot
                slots[elementIndex + eleIdx + 1] = allocate_new_elements(volume, 1);

                // Reverse the twin
                firstTwin = !firstTwin;
            }

            // Update the data
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];

                // Update heapIDs
                volume.heapIDArray[ind] = volume.heapIDArray[ind] * 2;
                volume.heapIDArray[slot] = volume.heapIDArray[ind] + 1;

                // Update the types
                volume.typeArray[ind] = 1;
                volume.typeArray[slot] = 2;

                // Flag modification
                volume.modifArray[ind] = (volume.modifArray[ind] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
                volume.modifArray[slot] = volume.modifArray[ind];
            }

            uint4 neighborArray[8];
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                uint32_t ind = indices[eleIdx];
                neighborArray[eleIdx] = volume.neighborsArray[ind];
            }

            // Schemes
            // UP (1 0 -1 / 0 1 -1)
            // DOWN (-1 0 1 / 0 -1 1)

            firstTwin = elementIndex % 2 == 0;
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];
                uint4 neighbors = neighborArray[eleIdx];

                // Grab the index and the allocated slot
                uint32_t nextIdx = indices[(eleIdx + 1) % 8];
                uint32_t nextSlot = slots[(eleIdx + 1) % 8];

                // Grab the index and the allocated slot
                uint32_t prevIdx = indices[(eleIdx - 1 + 8) % 8];
                uint32_t prevSLot = slots[(eleIdx - 1 + 8) % 8];

                // Set all the connections inside the loop
                if (firstTwin)
                {
                    volume.neighborsArray[ind] = { nextSlot, slot, prevSLot, neighbors.x };
                    volume.neighborsArray[slot] = { ind, nextIdx, prevIdx, neighbors.y };
                }
                else
                {
                    volume.neighborsArray[ind] = { prevSLot, slot, nextSlot , neighbors.x };
                    volume.neighborsArray[slot] = { ind, prevIdx, nextIdx, neighbors.y };
                }

                // Flag modification
                volume.modifArray[ind] = (volume.modifArray[ind] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
                volume.modifArray[slot] = volume.modifArray[ind];

                firstTwin = !firstTwin;
            }

            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];
                uint4 neighbors = neighborArray[eleIdx];
                if (neighbors.y != UINT32_MAX)
                    patch_neighbor(volume, neighbors.y, ind, slot);
            }
            /*
            // Keep track of this diamond
            Diamond diamond;
            diamond.size = 8;
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
                diamond.heapID[eleIdx] = volume.heapIDArray[indices[eleIdx]] / 2;
            for (uint32_t eleIdx = diamondSize; eleIdx < 8; ++eleIdx)
                diamond.heapID[eleIdx] = volume.heapIDArray[indices[0]] / 2;
            volume.diamonds.push_back(diamond);
            */
        }
        // 6 elements to process here
        else if (currentType == 1 || currentType == 2)
        {
            assert_fail();
        }
        // 4 elements to process here
        else if (currentType == 3)
        {
            // Buffer that holds all the valid indices of the diamond
            uint32_t indices[4] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };
            uint32_t slots[4] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX };

            // Our central element
            indices[elementIndex] = targetElement;
            slots[elementIndex] = allocate_new_elements(volume, 1);

            // Set all the left elements
            bool firstTwin = true;
            for (uint32_t eleIdx = 0; eleIdx < leftElements; ++eleIdx)
            {
                // Previous index
                uint32_t prevIndex = indices[elementIndex - eleIdx];

                // Get the following index
                indices[elementIndex - eleIdx - 1] = firstTwin ? volume.neighborsArray[prevIndex].w : volume.neighborsArray[prevIndex].z;

                // Allocate the slot
                slots[elementIndex - eleIdx - 1] = allocate_new_elements(volume, 1);

                // Reverse the twin
                firstTwin = !firstTwin;
            }

            // Set all the left elements
            firstTwin = true;
            for (uint32_t eleIdx = 0; eleIdx < rightElements; ++eleIdx)
            {
                // Previous index
                uint32_t prevIndex = indices[elementIndex + eleIdx];

                // Get the following index
                indices[elementIndex + eleIdx + 1] = firstTwin ? volume.neighborsArray[prevIndex].z : volume.neighborsArray[prevIndex].w;

                // Allocate the slot
                slots[elementIndex + eleIdx + 1] = allocate_new_elements(volume, 1);

                // Reverse the twin
                firstTwin = !firstTwin;
            }

            // Update the data
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];

                // Update heapIDs
                volume.heapIDArray[ind] = volume.heapIDArray[ind] * 2;
                volume.heapIDArray[slot] = volume.heapIDArray[ind] + 1;

                // Update the types
                volume.typeArray[ind] = 0;
                volume.typeArray[slot] = 0;

                // Flag modification
                volume.modifArray[ind] = (volume.modifArray[ind] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
                volume.modifArray[slot] = volume.modifArray[ind];
            }

            uint4 neighborArray[4];
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                uint32_t ind = indices[eleIdx];
                neighborArray[eleIdx] = volume.neighborsArray[ind];
            }

            firstTwin = elementIndex % 2 == 0;
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];
                uint4 neighbors = neighborArray[eleIdx];

                // Grab the index and the allocated slot
                uint32_t nextIdx = indices[(eleIdx + 1) % 4];
                uint32_t nextSlot = slots[(eleIdx + 1) % 4];

                // Grab the index and the allocated slot
                uint32_t prevIdx = indices[(eleIdx - 1 + 4) % 4];
                uint32_t prevSLot = slots[(eleIdx - 1 + 4) % 4];

                // Set all the connections inside the loop
                if (firstTwin)
                {
                    volume.neighborsArray[ind] = { nextSlot, slot, prevSLot, neighbors.x };
                    volume.neighborsArray[slot] = { ind, nextIdx, prevIdx, neighbors.y };
                }
                else
                {
                    volume.neighborsArray[ind] = { prevSLot, slot, nextSlot , neighbors.x };
                    volume.neighborsArray[slot] = { ind, prevIdx, nextIdx, neighbors.y };
                }

                // Flag modification
                volume.modifArray[ind] = (volume.modifArray[ind] & ~(ELEMENT_REQUESTED)) | ELEMENT_INVALID_CACHE;
                volume.modifArray[slot] = volume.modifArray[ind];
                firstTwin = !firstTwin;
            }

            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
            {
                // Grab the index and the allocated slot
                uint32_t ind = indices[eleIdx];
                uint32_t slot = slots[eleIdx];
                uint4 neighbors = neighborArray[eleIdx];
                if (neighbors.y != UINT32_MAX)
                    patch_neighbor(volume, neighbors.y, ind, slot);
            }

            // Keep track of this diamond
            Diamond diamond;
            diamond.size = 4;
            for (uint32_t eleIdx = 0; eleIdx < diamondSize; ++eleIdx)
                diamond.heapID[eleIdx] = volume.heapIDArray[indices[eleIdx]] / 2;
            for (uint32_t eleIdx = diamondSize; eleIdx < 4; ++eleIdx)
                diamond.heapID[eleIdx] = volume.heapIDArray[indices[0]] / 2;
            volume.diamonds.push_back(diamond);
        }
    }

    void recursive_split_element(LEBVolume& volume, uint32_t targetElement)
    {
        // First we need to make sure that everyone in the diamond is of the same type
        uint32_t rightElements = 0, leftElements = 0;
        bool completeDiamond = check_diamond(volume, targetElement, rightElements, leftElements);

        // Then we divide the diamond
        if (completeDiamond)
            diamond_split(volume, targetElement);
        else
            diamond_split_incomplete(volume, targetElement, rightElements, leftElements);
    }

    // Number of elements we allow
    const uint32_t g_NumSamples = 64;
    const uint32_t g_MaxNumSamples = 64;

    // Random values for the subdivion
    const float g_randValues[1024 * 3] = { 0.666396, 0.485715, 0.195109, 0.661663, 0.514085, 0.355028, 0.66598, 0.391414, 0.975544, 0.0694461, 0.692437, 0.893124, 0.882394, 0.0370496, 0.0150706, 0.364045, 0.0413516, 0.758285, 0.81706, 0.4531, 0.269463, 0.270766, 0.495198, 0.814423, 0.327479, 0.251672, 0.0402451, 0.681452, 0.976203, 0.323839, 0.462165, 0.285781, 0.832938, 0.141999, 0.360107, 0.164971, 0.471935, 0.262178, 0.737941, 0.899734, 0.852847, 0.119048, 0.203527, 0.702858, 0.578565, 0.611803, 0.836235, 0.685489, 0.202552, 0.811043, 0.27055, 0.868279, 0.164832, 0.896315, 0.130212, 0.996571, 0.296417, 0.491936, 0.460485, 0.89773, 0.109105, 0.919792, 0.432518, 0.387055, 0.567689, 0.501255, 0.718226, 0.669991, 0.206115, 0.157569, 0.928191, 0.954122, 0.580545, 0.514911, 0.898392, 0.872936, 0.0800286, 0.0925329, 0.898228, 0.165493, 0.190434, 0.496224, 0.0639763, 0.473503, 0.673996, 0.341668, 0.570015, 0.796298, 0.277851, 0.143967, 0.673157, 0.634842, 0.792124, 0.6255, 0.923486, 0.475615, 0.975397, 0.451066, 0.643816, 0.232277, 0.681069, 0.784651, 0.901268, 0.776224, 0.719079, 0.89203, 0.219328, 0.166517, 0.440035, 0.347464, 0.107837, 0.812212, 0.148275, 0.168295, 0.810396, 0.850702, 0.497764, 0.544742, 0.0577473, 0.36018, 0.698042, 0.852507, 0.761512, 0.325126, 0.704778, 0.282526, 0.935548, 0.316265, 0.637223, 0.396185, 0.535368, 0.118932, 0.238118, 0.207914, 0.659671, 0.26996, 0.812464, 0.944776, 0.772601, 0.0898776, 0.741193, 0.695768, 0.0439553, 0.683205, 0.698059, 0.984097, 0.525741, 0.934414, 0.895894, 0.209638, 0.891699, 0.474109, 0.160519, 0.973979, 0.179318, 0.179579, 0.405087, 0.281556, 0.970604, 0.125398, 0.458481, 0.191075, 0.115431, 0.403793, 0.108846, 0.600758, 0.11978, 0.812652, 0.586782, 0.0933691, 0.324114, 0.610999, 0.298, 0.421073, 0.560231, 0.806034, 0.571064, 0.586234, 0.20422, 0.413569, 0.13865, 0.779376, 0.0313641, 0.25836, 0.484006, 0.58996, 0.0230047, 0.153414, 0.0210534, 0.755971, 0.507842, 0.851419, 0.237829, 0.605265, 0.184749, 0.823932, 0.4017, 0.748893, 0.543449, 0.0496821, 0.48822, 0.395583, 0.507799, 0.883067, 0.897199, 0.665166, 0.262074, 0.64148, 0.193708, 0.772541, 0.861913, 0.0385729, 0.0675766, 0.854178, 0.0571387, 0.502576, 0.308278, 0.592821, 0.767629, 0.873629, 0.364513, 0.217204, 0.536221, 0.0821511, 0.126073, 0.475672, 0.434544, 0.426055, 0.65778, 0.954145, 0.5245, 0.00783304, 0.878455, 0.544893, 0.439541, 0.0831566, 0.969489, 0.847266, 0.76126, 0.275173, 0.797044, 0.995286, 0.0484789, 0.590237, 0.33955, 0.965418, 0.162206, 0.673138, 0.794971, 0.635449, 0.269653, 0.112241, 0.861192, 0.69866, 0.328373, 0.776557, 0.913631, 0.609744, 0.103809, 0.614762, 0.604002, 0.323343, 0.995878, 0.306864, 0.765351, 0.923268, 0.558757, 0.434542, 0.274857, 0.710676, 0.44535, 0.234239, 0.544487, 0.0705754, 0.848324, 0.904511, 0.910053, 0.318321, 0.516022, 0.652178, 0.0506757, 0.938703, 0.611004, 0.659273, 0.893375, 0.637459, 0.803721, 0.326604, 0.0389826, 0.0933859, 0.183971, 0.434383, 0.150411, 0.935827, 0.564455, 0.96128, 0.34953, 0.784825, 0.0802959, 0.710968, 0.932218, 0.969086, 0.690569, 0.625287, 0.0550805, 0.890509, 0.190378, 0.946349, 0.681064, 0.360764, 0.237352, 0.973658, 0.697726, 0.305782, 0.324638, 0.364612, 0.238713, 0.436569, 0.862501, 0.884646, 0.172541, 0.6756, 0.122092, 0.837792, 0.0965888, 0.249312, 0.591201, 0.988083, 0.384176, 0.547849, 0.371777, 0.280722, 0.274448, 0.177713, 0.179005, 0.816997, 0.646529, 0.0452048, 0.21406, 0.0182051, 0.564534, 0.0740849, 0.553413, 0.680363, 0.402797, 0.683183, 0.0670838, 0.835017, 0.0208894, 0.166071, 0.143357, 0.413812, 0.749521, 0.875315, 0.0827605, 0.454154, 0.0319409, 0.472759, 0.71724, 0.963104, 0.0705429, 0.117288, 0.189334, 0.652795, 0.882932, 0.649354, 0.777302, 0.961731, 0.39768, 0.575632, 0.881988, 0.478331, 0.320157, 0.970557, 0.484058, 0.62625, 0.86132, 0.988307, 0.771462, 0.397058, 0.753716, 0.322685, 0.3864, 0.815118, 0.328172, 0.110505, 0.79139, 0.663224, 0.0683096, 0.0759124, 0.244139, 0.776009, 0.679452, 0.372239, 0.105332, 0.68599, 0.0174056, 0.165109, 0.847084, 0.558897, 0.0408033, 0.899199, 0.424165, 0.177906, 0.245618, 0.557445, 0.088583, 0.298874, 0.348967, 0.538157, 0.81724, 0.796627, 0.832346, 0.0869249, 0.381563, 0.485296, 0.182202, 0.678146, 0.713458, 0.743813, 0.110968, 0.708849, 0.7559, 0.226391, 0.701201, 0.762268, 0.799015, 0.365068, 0.890594, 0.166143, 0.258006, 0.909398, 0.804183, 0.554821, 0.813739, 0.185814, 0.344466, 0.308048, 0.961302, 0.534008, 0.93553, 0.177885, 0.167676, 0.711611, 0.0368899, 0.311028, 0.23795, 0.56093, 0.617757, 0.863796, 0.512577, 0.898007, 0.896587, 0.111117, 0.990981, 0.338226, 0.361498, 0.47811, 0.686439, 0.839181, 0.88719, 0.860302, 0.0587999, 0.674182, 0.686596, 0.771039, 0.481188, 0.999042, 0.36616, 0.923661, 0.179411, 0.101677, 0.528365, 0.390921, 0.449509, 0.323476, 0.710548, 0.377266, 0.186437, 0.799125, 0.601361, 0.0801128, 0.490193, 0.397962, 0.659473, 0.569986, 0.354471, 0.73855, 0.616664, 0.609106, 0.283259, 0.923368, 0.848954, 0.980244, 0.559221, 0.149817, 0.124826, 0.187271, 0.842779, 0.00555294, 0.398005, 0.965216, 0.407942, 0.49433, 0.347851, 0.207319, 0.604114, 0.0896606, 0.254015, 0.883376, 0.381271, 0.782667, 0.361209, 0.625733, 0.178469, 0.392361, 0.516468, 0.245163, 0.613085, 0.971844, 0.962657, 0.361924, 0.0874644, 0.162287, 0.819196, 0.477573, 0.920944, 0.812945, 0.252979, 0.324209, 0.555707, 0.871243, 0.434183, 0.203123, 0.564943, 0.736916, 0.366864, 0.197151, 0.748387, 0.565236, 0.749576, 0.061349, 0.124559, 0.541136, 0.323849, 0.0541247, 0.996086, 0.805451, 0.951828, 0.506366, 0.93909, 0.580107, 0.440123, 0.921349, 0.79317, 0.948663, 0.918593, 0.627641, 0.125918, 0.638338, 0.961865, 0.649217, 0.806301, 0.532841, 0.736785, 0.450208, 0.0823454, 0.612754, 0.43022, 0.387674, 0.374223, 0.310026, 0.802116, 0.9249, 0.432292, 0.796707, 0.135921, 0.0940459, 0.350019, 0.835674, 0.676961, 0.123069, 0.781924, 0.963558, 0.724924, 0.592938, 0.450867, 0.144454, 0.441745, 0.475956, 0.229067, 0.441192, 0.332063, 0.211342, 0.782902, 0.440898, 0.231482, 0.754886, 0.673177, 0.225404, 0.142334, 0.645758, 0.451443, 0.151091, 0.548133, 0.254393, 0.460802, 0.887988, 0.432293, 0.699782, 0.638984, 0.709364, 0.768653, 0.296816, 0.890815, 0.0648696, 0.941304, 0.418125, 0.196189, 0.884008, 0.944363, 0.702718, 0.507255, 0.90103, 0.257407, 0.542536, 0.864303, 0.158169, 0.894912, 0.610593, 0.00790805, 0.546729, 0.839954, 0.0105106, 0.489619, 0.474168, 0.786319, 0.491382, 0.253244, 0.935752, 0.545181, 0.145104, 0.570077, 0.718744, 0.584135, 0.075085, 0.0939665, 0.0277568, 0.909202, 0.521546, 0.691384, 0.861972, 0.269578, 0.72116, 0.759926, 0.722819, 0.389668, 0.877073, 0.904852, 0.754639, 0.340692, 0.499991, 0.733426, 0.93366, 0.512836, 0.256422, 0.180404, 0.826302, 0.501777, 0.872289, 0.3363, 0.298828, 0.319895, 0.495119, 0.326322, 0.222987, 0.707107, 0.632508, 0.187255, 0.0980026, 0.205053, 0.712317, 0.72974, 0.573087, 0.923238, 0.147725, 0.891213, 0.867337, 0.368101, 0.658093, 0.981077, 0.659026, 0.133285, 0.402358, 0.462514, 0.664331, 0.219589, 0.813638, 0.820271, 0.00167687, 0.324581, 0.734324, 0.830967, 0.300019, 0.785986, 0.506024, 0.399712, 0.139965, 0.514801, 0.365496, 0.531691, 0.401448, 0.717625, 0.466117, 0.932241, 0.382057, 0.533557, 0.814113, 0.390356, 0.78012, 0.65825, 0.619443, 0.824433, 0.646767, 0.926646, 0.914738, 0.742666, 0.144883, 0.0511084, 0.00693319, 0.385808, 0.560856, 0.475167, 0.119809, 0.80243, 0.62318, 0.219808, 0.298397, 0.293924, 0.735979, 0.781232, 0.318389, 0.39793, 0.601491, 0.151159, 0.63728, 0.879869, 0.854269, 0.963373, 0.0431295, 0.417527, 0.994009, 0.217353, 0.997144, 0.972361, 0.993753, 0.743614, 0.840375, 0.764585, 0.174332, 0.719222, 0.534486, 0.765977, 0.638689, 0.529911, 0.556963, 0.910533, 0.824742, 0.201656, 0.235935, 0.436008, 0.481958, 0.946703, 0.84981, 0.0533484, 0.696604, 0.979596, 0.691331, 0.766516, 0.524852, 0.186858, 0.736631, 0.805154, 0.502172, 0.470859, 0.418607, 0.767529, 0.998048, 0.226615, 0.8426, 0.867223, 0.878943, 0.265232, 0.712679, 0.0195937, 0.362996, 0.278862, 0.60657, 0.11215, 0.549613, 0.994353, 0.136419, 0.275753, 0.424496, 0.383441, 0.84764, 0.867018, 0.032162, 0.238283, 0.266878, 0.820178, 0.820856, 0.0288315, 0.0617696, 0.357487, 0.551206, 0.069567, 0.863614, 0.951642, 0.739426, 0.0551659, 0.811677, 0.423379, 0.734743, 0.830842, 0.381587, 0.424437, 0.168273, 0.392103, 0.784205, 0.0451125, 0.220886, 0.893363, 0.724288, 0.537121, 0.15271, 0.619892, 0.344307, 0.160654, 0.43691, 0.702294, 0.726452, 0.286146, 0.916057, 0.413562, 0.390575, 0.0935033, 0.938836, 0.447241, 0.186815, 0.722776, 0.0459348, 0.24931, 0.615789, 0.427087, 0.288472, 0.233141, 0.903279, 0.169429, 0.704024, 0.33347, 0.960174, 0.740266, 0.109423, 0.0346369, 0.15612, 0.351215, 0.122886, 0.59686, 0.0355899, 0.943857, 0.86413, 0.00298592, 0.622175, 0.133059, 0.0358369, 0.945253, 0.645829, 0.156068, 0.613313, 0.754409, 0.764766, 0.938395, 0.992607, 0.974598, 0.45341, 0.859568, 0.0712337, 0.305217, 0.817706, 0.86376, 0.0885215, 0.141952, 0.0275724, 0.55475, 0.676978, 0.575662, 0.839179, 0.0118721, 0.0498657, 0.125251, 0.0373655, 0.144041, 0.234368, 0.287728, 0.198038, 0.948221, 0.25102, 0.699914, 0.249854, 0.553812, 0.532184, 0.316732, 0.818236, 0.0369471, 0.192283, 0.106654, 0.969392, 0.37972, 0.00272891, 0.433531, 0.0380754, 0.364391, 0.659316, 0.806852, 0.915025, 0.187207, 0.505748, 0.359521, 0.0351119, 0.764707, 0.634712, 0.573952, 0.864235, 0.83106, 0.85716, 0.163975, 0.521621, 0.869956, 0.399588, 0.110652, 0.344316, 0.825521, 0.793489, 0.753249, 0.109696, 0.973649, 0.0460847, 0.0815014, 0.694369, 0.183057, 0.775863, 0.222668, 0.0325375, 0.647284, 0.585687, 0.346847, 0.0198755, 0.840825, 0.184879, 0.156875, 0.374326, 0.621638, 0.130635, 0.771302, 0.635746, 0.409468, 0.578243, 0.739846, 0.66229, 0.00499842, 0.230546, 0.249524, 0.555134, 0.405169, 0.200745, 0.731202, 0.191816, 0.201103, 0.997381, 0.585518, 0.495469, 0.567137, 0.20525, 0.0325028, 0.634426, 0.464567, 0.44226, 0.765492, 0.0926129, 0.744242, 0.415532, 0.195853, 0.876934, 0.934584, 0.811244, 0.433559, 0.72804, 0.618947, 0.764534, 0.113559, 0.117901, 0.279798, 0.956954, 0.90274, 0.694036, 0.0271714, 0.757859, 0.316101, 0.360745, 0.139018, 0.764896, 0.797468, 0.806064, 0.803901, 0.241956, 0.944621, 0.602715, 0.0573628, 0.0989708, 0.743615, 0.0649431, 0.546407, 0.425598, 0.705804, 0.615781, 0.299919, 0.392264, 0.311926, 0.986864, 0.972351, 0.208722, 0.298259, 0.962501, 0.810056, 0.406207, 0.94038, 0.938549, 0.223691, 0.151711, 0.330003, 0.961159, 0.0155194, 0.135821, 0.588792, 0.0633473, 0.890277, 0.538588, 0.422573, 0.108548, 0.639227, 0.0656679, 0.776613, 0.0790063, 0.764435, 0.422237, 0.0706512, 0.812852, 0.196854, 0.950452, 0.453003, 0.840857, 0.0261752, 0.65038, 0.649631, 0.950432, 0.20789, 0.268159, 0.330578, 0.941558, 0.707515, 0.8457, 0.864949, 0.705654, 0.534007, 0.338908, 0.762793, 0.422337, 0.104718, 0.979755, 0.310024, 0.0621276, 0.811398, 0.65657, 0.778174, 0.391138, 0.61753, 0.138366, 0.466426, 0.956765, 0.749003, 0.457684, 0.674738, 0.840426, 0.310346, 0.62979, 0.343729, 0.713201, 0.577874, 0.432719, 0.846503, 0.437289, 0.810778, 0.234074, 0.987428, 0.225426, 0.83797, 0.910231, 0.698368, 0.618298, 0.0553145, 0.675258, 0.575982, 0.638173, 0.947858, 0.980611, 0.0650766, 0.721592, 0.0542087, 0.930349, 0.375394, 0.185091, 0.884752, 0.734836, 0.558489, 0.734436, 0.72751, 0.823721, 0.308647, 0.377333, 0.65432, 0.653709, 0.969998, 0.76187, 0.18257, 0.611112, 0.189444, 0.427172, 0.0616899, 0.795626, 0.384288, 0.923163, 0.279896, 0.467512, 0.122645, 0.394974, 0.513886, 0.614539, 0.0562266, 0.786606, 0.356473, 0.918663, 0.395751, 0.143317, 0.436091, 0.506897, 0.921399, 0.327789, 0.591813, 0.522002, 0.39695, 0.867394, 0.65212, 0.914263, 0.757454, 0.114994, 0.47102, 0.178026, 0.596279, 0.829618, 0.21817, 0.0453678, 0.197392, 0.0512079, 0.952486, 0.131038, 0.931336, 0.875565, 0.984337, 0.912926, 0.351183, 0.823882, 0.825012, 0.566588, 0.51042, 0.604312, 0.36171, 0.42078, 0.451995, 0.534203, 0.282415, 0.51924, 0.906024, 0.576883, 0.358379, 0.549386, 0.838483, 0.865289, 0.0992419, 0.660054, 0.209056, 0.599612, 0.237071, 0.160778, 0.499084, 0.337661, 0.753185, 0.283378, 0.64177, 0.390448, 0.221328, 0.869758, 0.671464, 0.135675, 0.159433, 0.152145, 0.411877, 0.237886, 0.915525, 0.615473, 0.879785, 0.221662, 0.727344, 0.773264, 0.0559387, 0.125295, 0.43638, 0.193114, 0.22124, 0.327946, 0.814097, 0.595464, 0.759628, 0.729825, 0.528293, 0.738152, 0.988166, 0.34706, 0.951464, 0.122535, 0.0930978, 0.846, 0.654617, 0.429645, 0.895112, 0.486997, 0.100804, 0.699719, 0.640653, 0.069817, 0.333462, 0.249737, 0.223692, 0.555309, 0.919699, 0.584206, 0.0726064, 0.473555, 0.325471, 0.122485, 0.830329, 0.0807839, 0.448428, 0.656175, 0.248706, 0.348662, 0.591473, 0.404566, 0.889813, 0.736385, 0.777952, 0.0383756, 0.162721, 0.652143, 0.164584, 0.695189, 0.961552, 0.924667, 0.829374, 0.29782, 0.514809, 0.29296, 0.940923, 0.188287, 0.371527, 0.0190084, 0.245494, 0.944152, 0.500135, 0.679016, 0.538321, 0.959014, 0.525588, 0.487403, 0.786487, 0.398144, 0.649196, 0.940313, 0.13031, 0.115526, 0.573504, 0.508164, 0.284478, 0.281278, 0.0327656, 0.858668, 0.847122, 0.749112, 0.858192, 0.298048, 0.984682, 0.880844, 0.0134418, 0.508255, 0.999236, 0.353257, 0.60998, 0.918952, 0.284751, 0.575558, 0.275943, 0.760955, 0.839323, 0.245663, 0.414991, 0.246424, 0.0474618, 0.706107, 0.826252, 0.99888, 0.258884, 0.194819, 0.203329, 0.91943, 0.311826, 0.352774, 0.648992, 0.566962, 0.376389, 0.281214, 0.743735, 0.708925, 0.757605, 0.0444994, 0.46357, 0.671458, 0.0850286, 0.666989, 0.606732, 0.988156, 0.490724, 0.890289, 0.24129, 0.800961, 0.439736, 0.0456747, 0.309139, 0.161556, 0.168071, 0.575651, 0.699742, 0.54386, 0.251361, 0.0805019, 0.358797, 0.131005, 0.317818, 0.026843, 0.145346, 0.405259, 0.839073, 0.286568, 0.092722, 0.398757, 0.59825, 0.754566, 0.435142, 0.596951, 0.147184, 0.23741, 0.873554, 0.315849, 0.40519, 0.133996, 0.320854, 0.683446, 0.735099, 0.707358, 0.605526, 0.0129357, 0.325551, 0.404118, 0.380278, 0.433321, 0.553361, 0.620582, 0.898018, 0.593091, 0.158371, 0.524877, 0.477245, 0.9976, 0.727507, 0.678649, 0.63815, 0.509352, 0.241473, 0.662996, 0.832001, 0.653643, 0.608581, 0.94908, 0.921827, 0.716509, 0.18834, 0.605652, 0.581619, 0.44952, 0.252118, 0.393937, 0.303138, 0.0535739, 0.906304, 0.432722, 0.432395, 0.531122, 0.434723, 0.139411, 0.232138, 0.935866, 0.622635, 0.95871, 0.563265, 0.533407, 0.922442, 0.576395, 0.187066, 0.120819, 0.369259, 0.180811, 0.924993, 0.557831, 0.738537, 0.74214, 0.351274, 0.771961, 0.694013, 0.691323, 0.0873022, 0.787679, 0.447403, 0.891443, 0.537331, 0.190018, 0.632635, 0.723, 0.618978, 0.998985, 0.994344, 0.655155, 0.0428336, 0.356375, 0.238811, 0.723176, 0.120832, 0.731114, 0.296517, 0.556594, 0.0652835, 0.0756491, 0.221861, 0.318691, 0.316914, 0.112735, 0.851453, 0.816673, 0.89049, 0.0541566, 0.515387, 0.708721, 0.941516, 0.666221, 0.332829, 0.107915, 0.110036, 0.105364, 0.35427, 0.615433, 0.311126, 0.575871, 0.0662295, 0.588713, 0.794003, 0.626261, 0.762, 0.747017, 0.852056, 0.479205, 0.552184, 0.0510724, 0.438967, 0.949708, 0.686758, 0.212961, 0.808046, 0.862208, 0.228734, 0.327491, 0.552249, 0.586013, 0.668978, 0.722499, 0.673048, 0.0161658, 0.415662, 0.987837, 0.143772, 0.465253, 0.227688, 0.506399, 0.909402, 0.372334, 0.88401, 0.485272, 0.925853, 0.0336529, 0.41654, 0.537766, 0.881427, 0.455471, 0.246115, 0.83802, 0.115372, 0.272062, 0.620325, 0.29129, 0.754776, 0.103496, 0.157671, 0.251741, 0.691927, 0.170262, 0.328078, 0.543228, 0.470822, 0.540126, 0.993994, 0.551243, 0.900252, 0.323328, 0.280577, 0.687265, 0.375221, 0.515514, 0.909804, 0.546349, 0.297815, 0.727319, 0.701979, 0.101613, 0.389321, 0.59405, 0.925107, 0.250573, 0.809992, 0.281339, 0.207916, 0.390438, 0.0888577, 0.991518, 0.738867, 0.0491308, 0.923809, 0.931872, 0.148058, 0.675823, 0.637445, 0.00889777, 0.855722, 0.852714, 0.159796, 0.442635, 0.77048, 0.107203, 0.789966, 0.000783537, 0.978468, 0.751512, 0.00386218, 0.0569624, 0.66797, 0.771371, 0.873201, 0.0420061, 0.387716, 0.905371, 0.8557, 0.97673, 0.110398, 0.554184, 0.418237, 0.0163355, 0.245771, 0.883778, 0.550305, 0.586041, 0.826307, 0.478204, 0.900118, 0.509355, 0.786718, 0.138361, 0.538416, 0.0579795, 0.92507, 0.854924, 0.47624, 0.642796, 0.249576, 0.707613, 0.163501, 0.54596, 0.271685, 0.894636, 0.398247, 0.850182, 0.760428, 0.691582, 0.554453, 0.295657, 0.227315, 0.613388, 0.622922, 0.443319, 0.614141, 0.651801, 0.518931, 0.662318, 0.727308, 0.308628, 0.620876, 0.712675, 0.628183, 0.635119, 0.376443, 0.146629, 0.184917, 0.629291, 0.00408074, 0.904207, 0.938971, 0.492644, 0.502524, 0.772153, 0.581557, 0.25535, 0.56576, 0.690355, 0.343688, 0.564136, 0.669863, 0.0321877, 0.87133, 0.225761, 0.606911, 0.661251, 0.738705, 0.606617, 0.0785045, 0.767994, 0.864076, 0.716466, 0.768006, 0.264587, 0.629722, 0.734937, 0.633064, 0.16013, 0.515267, 0.292636, 0.852904, 0.465444, 0.829899, 0.327915, 0.471927, 0.277148, 0.579296, 0.383082, 0.161342, 0.259925, 0.595907, 0.461246, 0.699814, 0.9853, 0.321437, 0.3077, 0.023122, 0.437778, 0.115904, 0.0206309, 0.724726, 0.659549, 0.876128, 0.735535, 0.989658, 0.935301, 0.365851, 0.55215, 0.37662, 0.175965, 0.266196, 0.572286, 0.505852, 0.870353, 0.185792, 0.662481, 0.375529, 0.939451, 0.263325, 0.663276, 0.831637, 0.127307, 0.107482, 0.445304, 0.454268, 0.315111, 0.0921151, 0.188407, 0.495789, 0.69916, 0.218723, 0.659315, 0.955586, 0.156281, 0.294236, 0.720383, 0.679805, 0.742932, 0.933772, 0.986052, 0.43827, 0.762681, 0.0157433, 0.484975, 0.28251, 0.554015, 0.947579, 0.656361, 0.810301, 0.947137, 0.322961, 0.484951, 0.59217, 0.810351, 0.410453, 0.165713, 0.283234, 0.410025, 0.756927, 0.447891, 0.670059, 0.162346, 0.70124, 0.713993, 0.769441, 0.552209, 0.345015, 0.251984, 0.278653, 0.488788, 0.708673, 0.851739, 0.942807, 0.973008, 0.923455, 0.437415, 0.377766, 0.451267, 0.368133, 0.00562299, 0.570673, 0.775375, 0.395132, 0.183796, 0.118967, 0.707671, 0.862295, 0.180472, 0.967361, 0.565207, 0.768455, 0.22301, 0.995828, 0.0418567, 0.975624, 0.889794, 0.0879353, 0.942034, 0.569619, 0.953872, 0.979931, 0.544575, 0.815084, 0.231879, 0.862153, 0.587586, 0.363841, 0.326651, 0.904059, 0.688253, 0.862035, 0.122161, 0.303916, 0.974072, 0.88438, 0.0747532, 0.756998, 0.161644, 0.757315, 0.0504771, 0.303464, 0.547171, 0.867305, 0.419363, 0.40212, 0.522645, 0.94471, 0.133789, 0.597038, 0.353182, 0.693711, 0.993959, 0.631215, 0.143502, 0.184232, 0.735016, 0.63225, 0.510379, 0.522777, 0.550331, 0.946103, 0.633196, 0.176046, 0.514358, 0.543132, 0.203254, 0.791961, 0.68809, 0.088005, 0.968699, 0.547594, 0.564182, 0.372223, 0.563046, 0.908034, 0.417323, 0.573513, 0.868606, 0.730446, 0.713655, 0.259062, 0.924951, 0.881239, 0.253672, 0.157761, 0.806186, 0.424733, 0.894901, 0.80805, 0.534738, 0.573853, 0.869008, 0.0133925, 0.265329, 0.921865, 0.298829, 0.0907778, 0.359574, 0.549647, 0.261876, 0.00341059, 0.0728397, 0.301762, 0.386211, 0.078449, 0.494075, 0.252302, 0.780088, 0.977462, 0.730104, 0.319065, 0.289137, 0.817385, 0.314961, 0.117345, 0.265852, 0.276528, 0.930753, 0.0194516, 0.60053, 0.544746, 0.218718, 0.267926, 0.80917, 0.2511, 0.449482, 0.0295149, 0.0036135, 0.869008, 0.539032, 0.813979, 0.561438, 0.788413, 0.194563, 0.410917, 0.746242, 0.0463897, 0.442352, 0.549155, 0.708268, 0.79752, 0.100802, 0.787347, 0.929167, 0.0271108, 0.136149, 0.0835563, 0.884191, 0.86128, 0.233209, 0.0107651, 0.507445, 0.937075, 0.948931, 0.823581, 0.442387, 0.227269, 0.834799, 0.866631, 0.0351804, 0.480412, 0.939101, 0.363265, 0.477917, 0.724766, 0.0580422, 0.267703, 0.807691, 0.443448, 0.0741712, 0.406616, 0.573313, 0.796578, 0.562945, 0.171831, 0.064469, 0.746527, 0.615753, 0.347731, 0.448074, 0.955458, 0.620173, 0.721773, 0.602318, 0.963236, 0.932757, 0.272343, 0.0623209, 0.670386, 0.114621, 0.342902, 0.389315, 0.778104, 0.698476, 0.964488, 0.436376, 0.600276, 0.0291703, 0.665683, 0.747293, 0.764485, 0.756598, 0.739772, 0.0690698, 0.595107, 0.214909, 0.472442, 0.666573, 0.848058, 0.81098, 0.182024, 0.493561, 0.752228, 0.580174, 0.651437, 0.360041, 0.885364, 0.84198, 0.849274, 0.638822, 0.135896, 0.206682, 0.00968726, 0.0593081, 0.401103, 0.0698183, 0.838873, 0.865469, 0.421719, 0.858114, 0.6378, 0.582638, 0.182929, 0.330311, 0.45109, 0.592268, 0.102484, 0.428269, 0.121532, 0.914348, 0.739148, 0.740084, 0.300259, 0.188741, 0.408297, 0.855445, 0.806004, 0.310597, 0.323113, 0.170458, 0.912651, 0.36209, 0.00513822, 0.232988, 0.957556, 0.0549995, 0.236583, 0.987661, 0.566377, 0.15795, 0.840312, 0.421717, 0.431665, 0.415938, 0.11833, 0.885262, 0.85526, 0.960609, 0.751188, 0.575263, 0.440809, 0.0625919, 0.0849359, 0.545125, 0.82277, 0.310333, 0.196094, 0.679125, 0.948405, 0.97103, 0.722191, 0.942181, 0.319416, 0.610713, 0.161431, 0.529953, 0.178969, 0.559109, 0.258655, 0.110526, 0.735978, 0.932287, 0.215041, 0.208841, 0.697747, 0.730437, 0.137258, 0.749819, 0.686742, 0.94817, 0.676275, 0.0378051, 0.744359, 0.616289, 0.957741, 0.292157, 0.830258, 0.737018, 0.55991, 0.344963, 0.646344, 0.812765, 0.658878, 0.715642, 0.289592, 0.252104, 0.495215, 0.955687, 0.881394, 0.602265, 0.170774, 0.648984, 0.706133, 0.492167, 0.689682, 0.190965, 0.947836, 0.0671655, 0.682582, 0.021863, 0.580932, 0.997623, 0.631444, 0.0470748, 0.657462, 0.951397, 0.947864, 0.869473, 0.331071, 0.0764137, 0.134259, 0.853109, 0.441956, 0.229734, 0.166804, 0.722162, 0.544085, 0.277936, 0.418565, 0.291805, 0.544056, 0.00706886, 0.294136, 0.83441, 0.114848, 0.545585, 0.939194, 0.614981, 0.975705, 0.682284, 0.350332, 0.330951, 0.574065, 0.863566, 0.524181, 0.753671, 0.663489, 0.276993, 0.840139, 0.409545, 0.202242, 0.637711, 0.780402, 0.136603, 0.743674, 0.322901, 0.266817, 0.430988, 0.837451, 0.029732, 0.386327, 0.14895, 0.592856, 0.754524, 0.188711, 0.869899, 0.353125, 0.204006, 0.982342, 0.230605, 0.850453, 0.328672, 0.163424, 0.557863, 0.599441, 0.0897752, 0.788608, 0.709872, 0.72203, 0.0566912, 0.266239, 0.655237, 0.0918161, 0.865955, 0.122665, 0.828624, 0.175924, 0.850178, 0.776361, 0.594582, 0.125319, 0.686567, 0.647152, 0.201226, 0.904889, 0.788243, 0.996603, 0.526667, 0.739371, 0.0984627, 0.433356, 0.585019, 0.951808, 0.299641, 0.693481, 0.571284, 0.375782, 0.883717, 0.895938, 0.839964, 0.816588, 0.933501, 0.202387, 0.308711, 0.788198, 0.379973, 0.555083, 0.471212, 0.227123, 0.906361, 0.77032, 0.689023, 0.307341, 0.0692508, 0.626954, 0.384941, 0.975229, 0.453644, 0.144167, 0.911252, 0.330288, 0.821161, 0.222794, 0.0466522, 0.139298, 0.9615, 0.419031, 0.00720708, 0.405898, 0.991909, 0.256333, 0.5015, 0.125564, 0.0498163, 0.924334, 0.76439, 0.0280978, 0.846779, 0.427556, 0.732286, 0.822311, 0.873722, 0.0142984, 0.759156, 0.204959, 0.0332132, 0.69748, 0.585244, 0.891592, 0.291032, 0.142791, 0.975641, 0.579178, 0.729296, 0.44119, 0.685034, 0.737183, 0.913442, 0.331368, 0.634874, 0.677823, 0.290696, 0.0435776, 0.697042, 0.783703, 0.221469, 0.278971, 0.595424, 0.302517, 0.490463, 0.521158, 0.45559, 0.535484, 0.561956, 0.0477893, 0.624324, 0.647343, 0.413147, 0.49548, 0.719845, 0.302179, 0.190032, 0.855333, 0.350458, 0.964997, 0.891389, 0.15696, 0.746908, 0.591542, 0.351737, 0.859808, 0.490949, 0.434491, 0.282087, 0.419584, 0.70942, 0.445091, 0.761825, 0.0637301, 0.743266, 0.791832, 0.327517, 0.567756, 0.876661, 0.09192, 0.729235, 0.741625, 0.133645, 0.924025, 0.812086, 0.756016, 0.793117, 0.216468, 0.608275, 0.0726016, 0.841651, 0.751123, 0.733054, 0.731982, 0.00170104, 0.149926, 0.6599, 0.541068, 0.307496, 0.679416, 0.958734, 0.393006, 0.397038, 0.460954, 0.140122, 0.361618, 0.89384, 0.951004, 0.173619, 0.542184, 0.817414, 0.603218, 0.486209, 0.109397, 0.52401, 0.377548, 0.597248, 0.642081, 0.998852, 0.626801, 0.791873, 0.507178, 0.821587, 0.787757, 0.432112, 0.543764, 0.224466, 0.434974, 0.931771, 0.417491, 0.844757, 0.898997, 0.0779591, 0.928855, 0.533002, 0.195309, 0.751724, 0.216368, 0.9946, 0.959465, 0.543688, 0.193075, 0.115915, 0.226794, 0.391832, 0.391646, 0.976734, 0.823625, 0.882266, 0.64828, 0.202905, 0.325872, 0.547297, 0.361168, 0.479941, 0.435426, 0.697423, 0.317234, 0.536454, 0.458844, 0.404218, 0.578745, 0.301369, 0.0937386, 0.343097, 0.190392, 0.848572, 0.412811, 0.844857, 0.387668, 0.873679, 0.175084, 0.785906, 0.248463, 0.292919, 0.748881, 0.889884, 0.782019, 0.628666, 0.893273, 0.644152, 0.867571, 0.344721, 0.290376, 0.563481, 0.57505, 0.967899, 0.0777754, 0.98924, 0.282413, 0.45939, 0.894027, 0.991951, 0.786938, 0.92862, 0.0782513, 0.113949, 0.316015, 0.476689, 0.292015, 0.0785141, 0.299947, 0.267081, 0.13012, 0.605259, 0.169863, 0.703897, 0.422347, 0.472591, 0.437836, 0.0639048, 0.646554, 0.430616, 0.289813, 0.817608, 0.633937, 0.680945, 0.814558, 0.438514, 0.409767, 0.265028, 0.0622799, 0.918171, 0.165328, 0.626525, 0.818939, 0.0661035, 0.24218, 0.653476, 0.558096, 0.562947, 0.14382, 0.847284, 0.558643, 0.266182, 0.528058, 0.453878, 0.975277, 0.191283, 0.0160192, 0.765373, 0.189053, 0.182231, 0.616205, 0.142784, 0.83137, 0.921717, 0.444885, 0.0409254, 0.544704, 0.391706, 0.573491, 0.18928, 0.722742, 0.610539, 0.728206, 0.772781, 0.501013, 0.916285, 0.873418, 0.395835, 0.460096, 0.626107, 0.496815, 0.221905, 0.784807, 0.326967, 0.654897, 0.907004, 0.714254, 0.0212391, 0.938704, 0.933965, 0.459868, 0.131048, 0.310894, 0.373571, 0.51211, 0.605577, 0.71674, 0.795699, 0.58346, 0.0521716, 0.99831, 0.830604, 0.725135, 0.950702, 0.640542, 0.713226, 0.0820053, 0.102529, 0.501349, 0.751006, 0.0510981, 0.614075, 0.196892, 0.553744, 0.844273, 0.188721, 0.328352, 0.512575, 0.185507, 0.000347872, 0.773494, 0.3191, 0.772719, 0.617572, 0.471139, 0.211561, 0.856124, 0.45306, 0.930481, 0.957733, 0.113227, 0.198419, 0.713261, 0.811051, 0.595474, 0.917834, 0.872196, 0.28512, 0.12633, 0.678198, 0.571998, 0.0674686, 0.696006, 0.148784, 0.572372, 0.0678842, 0.559304, 0.32725, 0.322916, 0.181689, 0.554838, 0.0926046, 0.944078, 0.602847, 0.381864, 0.181601, 0.146619, 0.505172, 0.985273, 0.0958826, 0.523289, 0.893749, 0.867673, 0.321633, 0.942154, 0.334201, 0.294935, 0.967413, 0.533135, 0.310605, 0.870357, 0.0284603, 0.6507, 0.686425, 0.768334, 0.26381, 0.0535408, 0.50388, 0.30435, 0.194052, 0.742663, 0.940618, 0.380453, 0.71784, 0.804033, 0.852417, 0.0194744, 0.712641, 0.418425, 0.891299, 0.516178, 0.936014, 0.683078, 0.725715, 0.16573, 0.393438, 0.855652, 0.131019, 0.504019, 0.990876, 0.994457, 0.302595, 0.542732, 0.0653849, 0.529859, 0.811532, 0.893634, 0.0673627, 0.166714, 0.0408592, 0.659634, 0.841196, 0.410221, 0.173624, 0.277623, 0.789564, 0.29478, 0.699638, 0.462536, 0.241927, 0.962051, 0.558386, 0.620936, 0.755911, 0.679307, 0.381169, 0.207028, 0.308464, 0.71093, 0.848575, 0.279458, 0.552044, 0.822755, 0.123608, 0.776023, 0.582035, 0.640352, 0.47445, 0.969937, 0.920046, 0.576912, 0.656781, 0.178677, 0.351285, 0.623039, 0.717107, 0.722454, 0.377238, 0.767855, 0.767785, 0.545204, 0.570867, 0.803338, 0.965248, 0.946177, 0.906283, 0.342871, 0.220019, 0.670193, 0.348047, 0.710134, 0.877795, 0.0193134, 0.379052, 0.588049, 0.470245, 0.431968, 0.519762, 0.930581, 0.147833, 0.745054, 0.608032, 0.290519, 0.945471, 0.467578, 0.219623, 0.457163, 0.981925, 0.22468, 0.364442, 0.778962, 0.106442, 0.0783381, 0.0153325, 0.340636, 0.0228824, 0.357168, 0.789118, 0.748818, 0.33107, 0.50591, 0.373315, 0.0169078, 0.586467, 0.762393, 0.190515, 0.934148, 0.387233, 0.160021, 0.216224, 0.678906, 0.136081, 0.342266, 0.422382, 0.975316, 0.797463, 0.716709, 0.443029, 0.376028, 0.877483, 0.753574, 0.319675, 0.288028, 0.695979, 0.0556148, 0.318934, 0.741867, 0.765351, 0.675663, 0.00680175, 0.154415, 0.836966, 0.798916, 0.176471, 0.775005, 0.796686, 0.381875, 0.924537, 0.297915, 0.205283, 0.689324, 0.655697, 0.8989, 0.937357, 0.179556, 0.661333, 0.997421, 0.127502, 0.678079, 0.643556, 0.0612656, 0.662091, 0.270001, 0.140914, 0.035904, 0.662001, 0.361986, 0.808576, 0.117415, 0.945084, 0.208141, 0.890475, 0.107378, 0.625419, 0.628165, 0.0169539, 0.296408, 0.100916, 0.265459, 0.0603498, 0.103627, 0.776113, 0.412191, 0.672967, 0.291956, 0.444092, 0.269312, 0.595114, 0.279969, 0.795031, 0.189775, 0.418428, 0.432704, 0.598494, 0.161718, 0.442602, 0.326105, 0.272677, 0.456476, 0.604828, 0.886392, 0.729876, 0.96707, 0.557813, 0.350259, 0.785917, 0.679159, 0.0229446, 0.691126, 0.423104, 0.993845, 0.854738, 0.667378, 0.259261, 0.369494, 0.197514, 0.94241, 0.543865, 0.905066, 0.824915, 0.665314, 0.052219, 0.373378, 0.109951, 0.60305, 0.715821, 0.853874, 0.520281, 0.290417, 0.433727, 0.8519, 0.518314, 0.764909, 0.232293, 0.0571275, 0.648624, 0.562963, 0.493722, 0.548211, 0.706569, 0.00155597, 0.207435, 0.480793, 0.841402, 0.719926, 0.145239, 0.908137, 0.665312, 0.193517, 0.990913, 0.888218, 0.46325, 0.720634, 0.54499, 0.728794, 0.898786, 0.611636, 0.34638, 0.935292, 0.647509, 0.411208, 0.292857, 0.637155, 0.326005, 0.892605, 0.404363, 0.372108, 0.393376, 0.944951, 0.35468, 0.0593461, 0.822156, 0.541171, 0.28758, 0.64391, 0.12182, 0.518222, 0.42566, 0.680637, 0.97527, 0.323638, 0.56021, 0.228998, 0.710711, 0.349062, 0.148541, 0.210056, 0.478101, 0.578317, 0.216788, 0.486165, 0.658206, 0.662026, 0.908037, 0.98715, 0.838024, 0.568959, 0.743494, 0.524797, 0.744701, 0.722963, 0.244239, 0.0648788, 0.88736, 0.0897641, 0.226914, 0.574529, 0.868164, 0.351639, 0.509631, 0.0699152, 0.916199, 0.680566, 0.368406, 0.702499, 0.874512, 0.807511, 0.161434, 0.437769, 0.569645, 0.850769, 0.397157, 0.938351, 0.205146, 0.262722, 0.958848, 0.313171, 0.142064, 0.925256, 0.278077, 0.361354, 0.350345, 0.577558, 0.322651, 0.296246, 0.939055, 0.964924, 0.878522, 0.722196, 0.938747, 0.735876, 0.171967, 0.132087, 0.0355267, 0.460198, 0.495563, 0.147435, 0.844658, 0.937956, 0.602661, 0.346767, 0.24191, 0.322006, 0.670864, 0.555147, 0.457824, 0.0223895, 0.673232, 0.625485, 0.624696, 0.731564, 0.590655, 0.0522749, 0.335502, 0.456354, 0.153838, 0.110774, 0.145965, 0.671008, 0.150859, 0.986993, 0.80309, 0.983623, 0.317613, 0.00183099, 0.379467, 0.555531, 0.24451, 0.332586, 0.505003, 0.702158, 0.874322, 0.564248, 0.630886, 0.426869, 0.351913, 0.0307013, 0.740623, 0.173375, 0.778473, 0.656191, 0.714253, 0.886929, 0.11353, 0.998478, 0.619928, 0.777644, 0.927962, 0.882533, 0.27228, 0.928427, 0.33492, 0.729187, 0.708209, 0.729694, 0.338904, 0.093702, 0.105706, 0.549513, 0.305526, 0.199841, 0.589937, 0.198297, 0.397914, 0.451971, 0.23406, 0.465951, 0.475751, 0.724563, 0.443994, 0.0475258, 0.938473, 0.254631, 0.296429, 0.463507, 0.792339, 0.977912, 0.85942, 0.259848, 0.138758, 0.107548, 0.248493, 0.262774, 0.490237, 0.492322, 0.526341, 0.674807, 0.983034, 0.139498, 0.792716, 0.173147, 0.310905, 0.0863226, 0.288894, 0.863713, 0.687205, 0.713781, 0.128086, 0.518425, 0.363241, 0.760319, 0.910962, 0.680351, 0.894097, 0.60106, 0.0656308, 0.668669, 0.581561, 0.000210384, 0.865886, 0.084972, 0.852482, 0.917757, 0.601443, 0.437893, 0.405563, 0.229098, 0.606033, 0.651045, 0.0373092, 0.241581, 0.648452, 0.476619, 0.90549 };
    float g_barycentrics[64 * 4] = {};

    void precompute_barycentrics()
    {
        // Pre-evalute the barycentrics
        for (uint32_t sampleIdx = 0; sampleIdx < g_MaxNumSamples; ++sampleIdx)
        {
            // Generate three random numbers
            float r1 = g_randValues[3 * sampleIdx];
            float r2 = g_randValues[3 * sampleIdx + 1];
            float r3 = g_randValues[3 * sampleIdx + 2];

            // Sort the random numbers
            if (r1 > r2) std::swap(r1, r2);
            if (r2 > r3) std::swap(r2, r3);
            if (r1 > r2) std::swap(r1, r2);

            // Compute barycentric coordinates
            float u = r1;
            float v = r2 - r1;
            float w = r3 - r2;
            float t = 1.0f - r3;

            // Compute the point in the tetrahedron
            g_barycentrics[4 * sampleIdx] = u;
            g_barycentrics[4 * sampleIdx + 1] = t;
            g_barycentrics[4 * sampleIdx + 2] = v;
            g_barycentrics[4 * sampleIdx + 3] = w;
        }
    }

    float evaluate_grid_value(const GridVolume& gridVolume, float3 position)
    {
        // Evalute the normalized positon
        float3 normPos = position + float3({ 0.5, 0.5, 0.5 });

        // Evalute the coords
        int64_t coordX = int64_t(normPos.x * gridVolume.resolution.x);
        int64_t coordY = int64_t(normPos.y * gridVolume.resolution.y);
        int64_t coordZ = int64_t(normPos.z * gridVolume.resolution.z);

        // If inside the volume return 
        if (coordX >= 0 && coordX < (int)gridVolume.resolution.x
            && coordY >= 0 && coordY < (int)gridVolume.resolution.y
            && coordZ >= 0 && coordZ < (int)gridVolume.resolution.z)
            return gridVolume.densityArray[coordX + coordY * gridVolume.resolution.x + gridVolume.resolution.x * gridVolume.resolution.y * coordZ];
        else
            return 0.0;
    }

    float3 point_in_tetrahedron(const Tetrahedron& tetra, uint32_t sampleIdx)
    {
        return tetra.p[0] * g_barycentrics[4 * sampleIdx] + tetra.p[1] * g_barycentrics[4 * sampleIdx + 1] + tetra.p[2] * g_barycentrics[4 * sampleIdx + 2] + tetra.p[3] * g_barycentrics[4 * sampleIdx + 3];
    }

    float2 eval_screen_cords(const float4x4& vp, const float3& position)
    {
        float4 p0P = mul_transpose(vp, float4({ position.x, position.y, position.z, 1.0f }));
        p0P.x = p0P.x / p0P.w;
        p0P.y = p0P.y / p0P.w;
        p0P.x = p0P.x * 0.5f + 0.5f;
        p0P.y = p0P.y * 0.5f + 0.5f;
        return { p0P.x, p0P.y };
    }

    void world_to_screen(const float4x4& vp, const AABB& aabb, float2& minSS, float2& maxSS)
    {
        // Initialize the screen-space bounds to extreme values
        minSS = float2(FLT_MAX, FLT_MAX);
        maxSS = float2(-FLT_MAX, -FLT_MAX);

        // Generate the 8 corners of the AABB in world space
        float3 corners[8] = {
            float3(aabb.min.x, aabb.min.y, aabb.min.z),
            float3(aabb.min.x, aabb.min.y, aabb.max.z),
            float3(aabb.min.x, aabb.max.y, aabb.min.z),
            float3(aabb.min.x, aabb.max.y, aabb.max.z),
            float3(aabb.max.x, aabb.min.y, aabb.min.z),
            float3(aabb.max.x, aabb.min.y, aabb.max.z),
            float3(aabb.max.x, aabb.max.y, aabb.min.z),
            float3(aabb.max.x, aabb.max.y, aabb.max.z)
        };

        // Transform each corner from world space to clip space
        for (int i = 0; i < 8; i++)
        {
            // Convert NDC to screen space
            const float2& cornerSS = eval_screen_cords(vp, corners[i]);

            // Update screen-space bounds
            minSS = min(minSS, cornerSS);
            maxSS = max(maxSS, cornerSS);
        }
    }

    bool should_subdivide_element(const LEBVolume& volume, uint32_t eleIdx, uint32_t depth, const GridVolume& gridVolume, const HeuristicCache& gridCache, const FittingParameters& fittingParams, const Frustum& frustum)
    {
        // Get the camera relative position
        const Tetrahedron& tetra = volume.tetraCacheArray[eleIdx];

        // Cull by frustrum if required
        if (fittingParams.frustumCull)
        {
            Tetrahedron tetraCR;
            tetraCR.p[0] = tetra.p[0] * gridVolume.scale - fittingParams.cameraPosition;
            tetraCR.p[1] = tetra.p[1] * gridVolume.scale - fittingParams.cameraPosition;
            tetraCR.p[2] = tetra.p[2] * gridVolume.scale - fittingParams.cameraPosition;
            tetraCR.p[3] = tetra.p[3] * gridVolume.scale - fittingParams.cameraPosition;

            // Evaluate the AABB
            AABB box;
            box.min = min(box.min, tetraCR.p[0]);
            box.min = min(box.min, tetraCR.p[1]);
            box.min = min(box.min, tetraCR.p[2]);
            box.min = min(box.min, tetraCR.p[3]);

            box.max = max(box.max, tetraCR.p[0]);
            box.max = max(box.max, tetraCR.p[1]);
            box.max = max(box.max, tetraCR.p[2]);
            box.max = max(box.max, tetraCR.p[3]);

            // Do we pursue?
            if (!frustum_aabb_intersect(frustum, box))
                return false;

            // Cull by pixel size if required
            if (fittingParams.pixelCull)
            {
                // Project the AABB to screen
                float2 minSS, maxSS;
                world_to_screen(fittingParams.viewProjectionMatrix, box, minSS, maxSS);

                // Does the AABB fit in a pixel?
                float2 extent = (maxSS - minSS);
                float sX = extent.x * fittingParams.screenSize.x;
                float sY = extent.y * fittingParams.screenSize.y;
                if (sX < fittingParams.pixelSize && sY < fittingParams.pixelSize)
                    return false;
            }
        }

        // Read one level lower than required
        float3 center = (tetra.p[0] + tetra.p[1] + tetra.p[2] + tetra.p[3]) * 0.25;
        const float4& stats = heuristic_cache::sample_cache(gridCache, center, depth);
        float range = stats.w - stats.z;
        float v = (range) / std::max(stats.x, fittingParams.minThreshold);
        return (v > fittingParams.ratioThreshold);
    }

    float mean_density_element(const GridVolume& gridVolume, uint32_t, const Tetrahedron& tetra)
    {
        // Mean value
        float mean = 0.0;
        for (uint32_t v = 0; v < g_MaxNumSamples; ++v)
        {
            float3 p = point_in_tetrahedron(tetra, v);
            mean += evaluate_grid_value(gridVolume, p);
        }
        return (mean / g_MaxNumSamples);
    }

    uint32_t fit_volume_to_grid(LEBVolume& lebVolume, const GridVolume& gridVolume, const HeuristicCache& heuristicCache, const FittingParameters& parameters)
    {
        // Extract the frustum from the view proj
        Frustum frustum;
        extract_planes_from_view_projection_matrix(parameters.viewProjectionMatrix, frustum);

        // Pre-compute the sample barycentrics
        precompute_barycentrics();
        Leb3DCache lebCache;

        // Compute the right max depth
        uint32_t maxDepth = uint32_t(log2f((float)gridVolume.resolution.x) * 3.0 + 6.0) - 2;

        // All the initial elements should be initialized with the right state
        for (uint32_t eleIdx = 0; eleIdx < lebVolume.totalNumElements; ++eleIdx)
            lebVolume.modifArray[eleIdx] = ELEMENT_INCLUDED | ELEMENT_INVALID_CACHE;

        // Subdivide the tetrahedrons untill it's good
        while (true)
        {
            // We loop through the elements, classify them in a parallel fashion, then do the subdivion in a monothread fashion
            const uint32_t elementCount = lebVolume.totalNumElements;

            #pragma omp parallel for num_threads(32)
            for (int32_t eleIdx = 0; eleIdx < (int32_t)elementCount; ++eleIdx)
            {
                // Get it's depth
                if ((lebVolume.modifArray[eleIdx] & (ELEMENT_INCLUDED)) == 0)
                    continue;

                // If the element was flagged as modified and is still included, update the caches
                if ((lebVolume.modifArray[eleIdx] & (ELEMENT_INVALID_CACHE)) == ( ELEMENT_INVALID_CACHE))
                {
                    // Grab heap ID
                    const uint64_t heapID = lebVolume.heapIDArray[eleIdx];

                    // Re-evaluate the depth
                    lebVolume.depthArray[eleIdx] = (uint8_t)find_msb_64(heapID);

                    // Update the posision
                    leb_volume::evaluate_tetrahedron(lebVolume.heapIDArray[eleIdx], lebVolume.minimalDepth, lebVolume.basePoints, lebVolume.baseTypes, lebCache, lebVolume.tetraCacheArray[eleIdx]);

                    // Not modified anymore
                    lebVolume.modifArray[eleIdx] &= ~(ELEMENT_INVALID_CACHE);
                }

                // Get it's depth
                if (lebVolume.depthArray[eleIdx] >= maxDepth)
                    continue;

                // Request or exclude
                if (should_subdivide_element(lebVolume, eleIdx, lebVolume.depthArray[eleIdx], gridVolume, heuristicCache, parameters, frustum))
                    lebVolume.modifArray[eleIdx] |= ELEMENT_REQUESTED;
                else
                    lebVolume.modifArray[eleIdx] &= ~(ELEMENT_INCLUDED);
            }

            // Non parallel retourine
            uint32_t splitTriggered = 0;
            for (uint32_t eleIdx = 0; eleIdx < elementCount; ++eleIdx)
            {
                if ((lebVolume.modifArray[eleIdx] & ELEMENT_REQUESTED) == ELEMENT_REQUESTED)
                {
                    recursive_split_element(lebVolume, eleIdx);
                    splitTriggered++;
                }
            }

            // How many splits during this round
            if (splitTriggered == 0)
                break;
        }

        // Final number of primitives
        printf("    Total num Primitives %u\n", lebVolume.totalNumElements);

        return maxDepth;
    }
}