// Internal includes
#include "render_pipeline/volume_pipeline.h"

// External includes
#define NOMINMAX
#include <Windows.h>
#include <string>

int CALLBACK main(HINSTANCE hInstance, HINSTANCE, PWSTR, int)
{
    // Path of the exe
    const std::string& exePath(__argv[0]);
    uint32_t loc = (uint32_t)exePath.find_last_of('\\');

    // Evaluate the project directory
    const std::string& exeDir = exePath.substr(0, loc);
    const std::string& projectDir = __argv[1];

    // Create and init the pipeline
    const std::string& gridVolume = projectDir + "/volumes/wdas_cloud_grid.bin";
    const std::string& lebVolume = projectDir + "/volumes/wdas_cloud_leb.bin";

    // Volume pipeline
    VolumePipeline pipeline;
    pipeline.initialize(hInstance, projectDir.c_str(), exeDir.c_str(), gridVolume.c_str(), lebVolume.c_str());

    // Trigger the render loop
    pipeline.render_loop();

    // release the pipeline
    pipeline.release();
	return 0;
}