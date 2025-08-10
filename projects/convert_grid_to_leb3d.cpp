// Project includes
#include "volume/grid_volume.h"
#include "volume/leb_volume.h"
#include "volume/leb_volume_gpu.h"
#include "volume/volume_generation.h"
#include "math/operators.h"
#include "tools/security.h"

// System includes
#define NOMINMAX
#include <Windows.h>
#include <string>
#include <iostream>

int CALLBACK main(HINSTANCE, HINSTANCE, PWSTR, int)
{
    // Check the parameter count
    assert_msg(__argc == 2, "Not enough parameters to the call. One parameter expected <project_dir>.");

    // Project directory
    const std::string& projectDir = __argv[1];

    // Volume that holds our intial structure
    LEBVolume lebVolume;
    leb_volume::create_type0_cube(lebVolume);
    std::cout << "Base LEB structured built." << std::endl;

    // Import the grid
    GridVolume gridVolume;
    grid_volume::import_grid_volume((projectDir + "/volumes/wdas_cloud_grid.bin").c_str(), gridVolume);
    std::cout << "Grid volume imported." << std::endl;

    // Cache
    HeuristicCache heuristicCache;
    heuristic_cache::build_heuristic_cache(gridVolume, heuristicCache);
    std::cout << "Heuristic cache built." << std::endl;

    // Subdivide the volume
    FittingParameters fittingParams = { false, false };
    uint32_t maxDepth = leb_volume::fit_volume_to_grid(lebVolume, gridVolume, heuristicCache, fittingParams);
    std::cout << "LEB3D volume generated." << std::endl;

    // Export the volume
    LEBVolumeGPU lebVolumeGPU;
    uint64_t compressedSize = leb_volume::convert_to_leb_volume_to_gpu(lebVolume, gridVolume, fittingParams, maxDepth, lebVolumeGPU);
    std::cout << "LEB3D converted for the GPU." << std::endl;

    // Display the compressed size
    std::cout << "LEB3D compressed size " << compressedSize << " bytes." << std::endl;

    // Export to disk
    leb_volume::export_leb_volume_gpu(lebVolumeGPU, (projectDir + "/volumes/wdas_cloud_leb.bin").c_str());
    std::cout << "LEB3D GPU exported." << std::endl;
    return 0;
}