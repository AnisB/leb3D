#pragma once

// Internal incldues
#include "rendering/frustum.h"
#include "tools/profiling_helper.h"
#include "render_pipeline/camera_controller.h"
#include "render_pipeline/rendering_mode.h"
#include "render_pipeline/leb_renderer.h"
#include "render_pipeline/grid_renderer.h"
#include "render_pipeline/frustum_renderer.h"
#include "render_pipeline/sky.h"

// External includes includes
#define NOMINMAX
#include <Windows.h>
#include <string>

class VolumePipeline
{
public:
    // Cst & Dst
    VolumePipeline();
    ~VolumePipeline();

    // Initialization and release
    void initialize(HINSTANCE hInstance, const char* projectDirectory, const char* exeDirectory, const char* gridVolume, const char* lebVolume);
    void release();

    // Runtime loop
    void render_loop();

private:
    // Init
    void reload_shaders();

    // Rendering
    void prepare_rendering(CommandBuffer cmd);
    void render_ui(CommandBuffer cmd, RenderTexture rTexture);
    void update_constant_buffers(CommandBuffer cmd);

    // render pipelines
    void render_pipeline(CommandBuffer cmd);

    // Update
    void update(double deltaTime);

    // Controls
    void process_key_event(uint32_t keyCode, bool state);

private:
    // Graphics Backend
    GraphicsDevice m_Device = 0;
    RenderWindow m_Window = 0;
    CommandQueue m_CmdQueue = 0;
    CommandBuffer m_CmdBuffer = 0;
    SwapChain m_SwapChain = 0;

    // Project directory
    std::string m_ProjectDir = "";
    std::string m_ExeDir = "";

    // Global Rendering Resources
    RenderTexture m_ColorTexture = 0;
    RenderTexture m_DepthBuffer = 0;
    RenderTexture m_HistoryTexture = 0;
    ConstantBuffer m_GlobalCB = 0;
    ComputeShader m_AccumulateFrameCS = 0;
    ComputeShader m_TonemapFrameCS = 0;

    // Components
    LEBRenderer m_LEBRenderer = LEBRenderer();
    GridRenderer m_GridRenderer = GridRenderer();
    FrustumRenderer m_FrustumRenderer = FrustumRenderer();
    Sky m_Sky = Sky();
    CameraController m_CameraController = CameraController();
    ProfilingHelper m_ProfilingHelper = ProfilingHelper();

    // Global rendering properties
    uint32_t m_FrameIndex = 0;
    double m_Time = 0.0;
    float4 m_ScreenSize = { 0.0, 0.0, 0.0, 0.0 };
    uint2 m_ScreenSizeI = { 0, 0 };

    // UI controls
    bool m_DisplayUI = false;
    RenderingMode m_RenderingMode = RenderingMode::Count;
    bool m_LEBPath = false;
    bool m_RenderFrustum = false;
    float m_DensityMultipler = 0.75;
    float m_Albedo = 0.75;
    float m_SunIntensity = 1.0;
    float m_SkyIntensity = 1.0;
    float m_SunElevation = 1.0;
    float m_SunRotation = 1.0;
};