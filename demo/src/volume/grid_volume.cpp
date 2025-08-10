// Internal includes
#include "volume/grid_volume.h"
#include "tools/stream.h"

namespace grid_volume
{
    // Export a packed mesh to disk
    void export_grid_volume(const GridVolume& gridVolume, const char* path)
    {
        // Vector that will hold our packed mesh 
        std::vector<char> binaryFile;

        // Pack the structure in a buffer
        pack_bytes(binaryFile, gridVolume.scale);
        pack_bytes(binaryFile, gridVolume.resolution);
        pack_vector_bytes(binaryFile, gridVolume.densityArray);

        // Write to disk
        FILE* pFile;
        pFile = fopen(path, "wb");
        fwrite(binaryFile.data(), sizeof(char), binaryFile.size(), pFile);
        fclose(pFile);
    }

    // Export a packed mesh to disk
    void import_grid_volume(const char* path, GridVolume& gridVolume)
    {
        // Vector that will hold our packed mesh 
        std::vector<char> binaryFile;

        // Read from disk
        FILE* pFile;
        pFile = fopen(path, "rb");
        _fseeki64(pFile, 0L, SEEK_END);
        __int64 fileSize = _ftelli64(pFile);
        binaryFile.resize(fileSize);
        _fseeki64(pFile, 0L, SEEK_SET);
        rewind(pFile);
        fread(binaryFile.data(), sizeof(char), fileSize, pFile);
        fclose(pFile);

        // Pack the structure in a buffer
        const char* binaryPtr = binaryFile.data();
        unpack_bytes(binaryPtr, gridVolume.scale);
        unpack_bytes(binaryPtr, gridVolume.resolution);
        unpack_vector_bytes(binaryPtr, gridVolume.densityArray);
    }
}