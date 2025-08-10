// Internal includes
#include "graphics/dx12_backend.h"
#include "render_pipeline/constant_buffers.h"
#include "render_pipeline/volume_pipeline.h"
#include "imgui/imgui.h"
#include "math/operators.h"
#include "tools/security.h"
#include "tools/shader_utils.h"

// External includes
#include <algorithm>
#include <chrono>
#include <math.h>
#include <iostream>

VolumePipeline::VolumePipeline()
{
}

VolumePipeline::~VolumePipeline()
{
}

void VolumePipeline::initialize(HINSTANCE hInstance, const char* projectDirectory, const char* exeDirectory, const char* gridVolume, const char* lebVolume)
{
    // Keep the project dir
    m_ProjectDir = projectDirectory;
    m_ExeDir = exeDirectory;

    // Keep track of the device
    m_Device = d3d12::device::create_graphics_device(DevicePickStrategy::VendorID, (uint32_t)GPUVendor::Nvidia);

    // Generic graphics api stuff
    m_Window = d3d12::window::create_window((uint64_t)hInstance, 1024, 1024, "Volume Renderer");
    m_CmdQueue = d3d12::command_queue::create_command_queue(m_Device);
    m_CmdBuffer = d3d12::command_buffer::create_command_buffer(m_Device);
    m_SwapChain = d3d12::swap_chain::create_swap_chain(m_Window, m_Device, m_CmdQueue, TextureFormat::R16G16B16A16_Float);

    // Evaluate the sizes
    d3d12::window::viewport_size(m_Window, m_ScreenSizeI);
    m_ScreenSize = float4({ (float)m_ScreenSizeI.x, (float)m_ScreenSizeI.y, 1.0f / m_ScreenSizeI.x, 1.0f / m_ScreenSizeI.y });

    // Initialize imgui
    d3d12::imgui::initialize_imgui(m_Device, m_Window, TextureFormat::R16G16B16A16_Float);

    // Profiler
    m_ProfilingHelper.initialize(m_Device, 1);

    // Constant buffers
    m_GlobalCB = d3d12::resources::create_constant_buffer(m_Device, sizeof(GlobalCB), ConstantBufferType::Mixed);

    // Depth buffer
    {
        TextureDescriptor descriptor;
        descriptor.type = TextureType::Tex2D;
        descriptor.width = (uint32_t)m_ScreenSize.x;
        descriptor.height = (uint32_t)m_ScreenSize.y;
        descriptor.depth = 1;
        descriptor.mipCount = 1;
        descriptor.isUAV = false;
        descriptor.format = TextureFormat::Depth32Stencil8;
        descriptor.clearColor = float4({ 1.0f, 0.0f, 0.0f, 0.0f });
        descriptor.debugName = "Depth Buffer D32S8";
        m_DepthBuffer = d3d12::resources::create_render_texture(m_Device, descriptor);
    }

    // Color texture
    {
        TextureDescriptor descriptor;
        descriptor.type = TextureType::Tex2D;
        descriptor.width = (uint32_t)m_ScreenSize.x;
        descriptor.height = (uint32_t)m_ScreenSize.y;
        descriptor.depth = 1;
        descriptor.mipCount = 1;
        descriptor.isUAV = true;
        descriptor.clearColor = float4({ 0.0f, 0.0f, 0.0f, 0.0f });
        descriptor.debugName = "ColorTexture";

        // history texture
        descriptor.format = TextureFormat::R16G16B16A16_Float;
        m_ColorTexture = d3d12::resources::create_render_texture(m_Device, descriptor);

        descriptor.format = TextureFormat::R32G32B32A32_Float;
        m_HistoryTexture = d3d12::resources::create_render_texture(m_Device, descriptor);
    }

    // Rendering properties
    m_FrameIndex = 0;
    m_Time = 0.0;

    // UI settings
    m_DisplayUI = true;
    m_RenderingMode = RenderingMode::ForwardPT;
    m_LEBPath = true;
    m_RenderFrustum = false;

    m_Albedo = 0.8f;
    m_SunIntensity = 4.0f;
    m_SkyIntensity = 2.5f;
    m_SunElevation = 0.4f;
    m_SunRotation = 0.7f;
    m_DensityMultipler = 1.0f;

    // Initialize the camera controller
    m_CameraController.initialize(m_Window, (uint32_t)m_ScreenSize.x, (uint32_t)m_ScreenSize.y, 30.0f * DEG_TO_RAD);
    Camera& camera = m_CameraController.get_camera();
    camera.position = float3({ 1.7, 0.03f, -0.857f });
    camera.angles = float3({ 1.1, -0.06, 0.0f });
    camera.nearFar = { 0.001f, 20.0f };

    // Modules
    m_LEBRenderer.initialize(m_Device, m_ScreenSizeI);
    m_GridRenderer.initialize(m_Device);
    m_Sky.initialize(m_Device);

    // Location of the shader library
    std::string volumesLibrary = m_ProjectDir;
    volumesLibrary += "\\volumes\\";

    // Load the geometry
    m_GridRenderer.load_geometry(gridVolume);
    m_LEBRenderer.load_geometry(lebVolume);
    m_FrustumRenderer.initialize(m_Device, m_LEBRenderer.volume());

    // Load all the shaders
    reload_shaders();

    // Upload geometry
    m_LEBRenderer.upload_geometry(m_CmdQueue, m_CmdBuffer);
    m_GridRenderer.upload_geometry(m_CmdQueue, m_CmdBuffer);
}

void VolumePipeline::release()
{
    // Modules
    m_LEBRenderer.release();
    m_GridRenderer.release();
    m_FrustumRenderer.release();
    m_Sky.release();
    m_ProfilingHelper.release();

    // Imgui cleanup
    d3d12::imgui::release_imgui();

    // Other rendering resources
    d3d12::compute_shader::destroy_compute_shader(m_TonemapFrameCS);
    d3d12::compute_shader::destroy_compute_shader(m_AccumulateFrameCS);
    d3d12::resources::destroy_constant_buffer(m_GlobalCB);
    d3d12::resources::destroy_render_texture(m_HistoryTexture);
    d3d12::resources::destroy_render_texture(m_DepthBuffer);
    d3d12::resources::destroy_render_texture(m_ColorTexture);

    // Generic graphics api cleanup
    d3d12::swap_chain::destroy_swap_chain(m_SwapChain);
    d3d12::command_buffer::destroy_command_buffer(m_CmdBuffer);
    d3d12::command_queue::destroy_command_queue(m_CmdQueue);
    d3d12::window::destroy_window(m_Window);
    d3d12::device::destroy_graphics_device(m_Device);

}

void VolumePipeline::reload_shaders()
{
    // Location of the shader library
    std::string shaderLibrary = m_ProjectDir;
    shaderLibrary += "\\shaders";

    {
        // Create the compute shaders
        ComputeShaderDescriptor csd;
        csd.includeDirectories.push_back(shaderLibrary);
        csd.filename = shaderLibrary + "\\AccumulateFrame.compute";

        csd.kernelname = "AccumulateFrame";
        compile_and_replace_compute_shader(m_Device, csd, m_AccumulateFrameCS);
    }

    {
        // Create the compute shaders
        ComputeShaderDescriptor csd;
        csd.includeDirectories.push_back(shaderLibrary);
        csd.filename = shaderLibrary + "\\TonemapFrame.compute";

        csd.kernelname = "TonemapFrame";
        compile_and_replace_compute_shader(m_Device, csd, m_TonemapFrameCS);
    }

    // Modules
    m_LEBRenderer.reload_shaders(shaderLibrary);
    m_GridRenderer.reload_shaders(shaderLibrary);
    m_FrustumRenderer.reload_shader(shaderLibrary);
    m_Sky.reload_shaders(shaderLibrary);
}

void VolumePipeline::render_ui(CommandBuffer cmd, RenderTexture rTexture)
{
    if (!m_DisplayUI)
        return;
    
    // Any changes to record?
    bool changes = false;
    d3d12::command_buffer::start_section(cmd, "Render UI");
    {
        d3d12::command_buffer::start_section(cmd, "Render UI");
        {
            // Start enqueing commands
            d3d12::imgui::start_frame();

            // Debug Params
            ImGui::SetNextWindowSize(ImVec2(350.0f, 300.0f));
            ImGui::Begin("Debug parameters");
            {
                // Reset of checked
                changes |= ImGui::Checkbox("LEB", &m_LEBPath);
                changes |= ImGui::Checkbox("Frustum", &m_RenderFrustum);
                changes |= ImGui::SliderFloat("Albedo", &m_Albedo, 0.0, 1.0);
                changes |= ImGui::SliderFloat("Sun Intensity", &m_SunIntensity, 0.0, 10.0);
                changes |= ImGui::SliderFloat("Sky Intensity", &m_SkyIntensity, 0.0, 10.0);
                changes |= ImGui::SliderFloat("Sun Elevation", &m_SunElevation, 0.001, 1.0);
                changes |= ImGui::SliderFloat("Sun Rotation", &m_SunRotation, 0.001, 1.0);
                changes |= ImGui::SliderFloat("Density Multiplier", &m_DensityMultipler, 0.01f, 3.0f);
                
                // Create a dropdown menu
                const char* rendering_mode_labels[] = { "DebugView", "Density", "ForwardPT" };
                const char* current_item = rendering_mode_labels[(uint32_t)m_RenderingMode];
                if (ImGui::BeginCombo("Mode", current_item)) // "Dropdown" is the label of the dropdown
                {
                    for (int i = 0; i < (uint32_t)RenderingMode::Count; i++)
                    {
                        bool is_selected = (current_item == rendering_mode_labels[i]); // Check if the item is selected
                        if (ImGui::Selectable(rendering_mode_labels[i], is_selected))
                        {
                            m_RenderingMode = (RenderingMode)i;
                            m_FrameIndex = 0;
                        }

                        // Set the initial focus when opening the combo (optional)
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                // Display the duration
                uint64_t duration = m_ProfilingHelper.get_scope_last_duration(0);
                std::string renderDuration = "Duration: ";
                renderDuration += std::to_string(duration);
                ImGui::Text(renderDuration.c_str());
            }
            ImGui::End();

            // End enqueing commands
            d3d12::imgui::end_frame();

            // Enqueue into the command buffer
            d3d12::imgui::draw_frame(cmd, rTexture);
        }
        d3d12::command_buffer::end_section(cmd);
    }
    d3d12::command_buffer::end_section(cmd);

    // Any changes?
    if (changes)
        m_FrameIndex = 0;
}

void VolumePipeline::update_constant_buffers(CommandBuffer cmdB)
{
    d3d12::command_buffer::start_section(cmdB, "Update Constant Buffers");
    {
        // Grab the camera
        const Camera& camera = m_CameraController.get_camera();

        // Global constant buffer
        GlobalCB globalCB;

        // Camera properties
        globalCB._ViewProjectionMatrix = camera.viewProjection;
        globalCB._InvViewProjectionMatrix = camera.invViewProjection;
        globalCB._CameraPosition = camera.position;
        globalCB._ScreenSize = m_ScreenSize;
        globalCB._FrameIndex = m_FrameIndex;
        globalCB._FrameAccumulationFactors = m_FrameIndex == 0 ? float2({ 1.0, 0.0 }) : float2({ 1.0f / m_FrameIndex, (m_FrameIndex - 1) / (float)m_FrameIndex });

        // Volume properties
        globalCB._DensityMultiplier = m_DensityMultipler;
        globalCB._VolumeAlbedo = m_Albedo;

        // Light properties
        globalCB._SunIntensity = m_SunIntensity;
        globalCB._SkyIntensity = m_SkyIntensity;
        float angle = (1.0f - m_SunElevation) * (float)HALF_PI;
        float angle2 = (1.0f - m_SunRotation) * (float)TWO_PI;
        globalCB._SunDirection = float3({ sinf(angle2) * sinf(angle), cosf(angle), cosf(angle2) * sinf(angle)});

        d3d12::resources::set_constant_buffer(m_GlobalCB, (const char*)&globalCB, sizeof(GlobalCB));
        d3d12::command_buffer::upload_constant_buffer(cmdB, m_GlobalCB);

        // Modules
        m_LEBRenderer.upload_constant_buffers(cmdB, camera.position);
        m_GridRenderer.upload_constant_buffers(cmdB);
        m_FrustumRenderer.upload_constant_buffers(cmdB);
    }
    d3d12::command_buffer::end_section(cmdB);
}

void VolumePipeline::process_key_event(uint32_t keyCode, bool state)
{
    switch (keyCode)
    {
    case 0x74: // F5
        if (state)
            reload_shaders();
        break;
    case 0x7A: // F11
        if (state)
            m_DisplayUI = !m_DisplayUI;
        break;
    }

    // Propagate to the camera controller
    m_CameraController.process_key_event(keyCode, state);
}

void VolumePipeline::prepare_rendering(CommandBuffer cmd)
{
    // Reset the command buffer
    d3d12::command_buffer::reset(cmd);

    // Update the constant buffer
    update_constant_buffers(cmd);
    m_Sky.pre_render(cmd);

    // Close and flush the command buffer
    d3d12::command_buffer::close(cmd);
    d3d12::command_queue::execute_command_buffer(m_CmdQueue, cmd);
    d3d12::command_queue::flush(m_CmdQueue);
}

void VolumePipeline::render_pipeline(CommandBuffer cmd)
{
    // Grab the camera
    const Camera& camera = m_CameraController.get_camera();

    // Reset the command buffer
    d3d12::command_buffer::reset(cmd);

    // Update the constant buffers
    update_constant_buffers(cmd);

    // Clear the render target texture
    {
        d3d12::command_buffer::start_section(cmd, "Clear RTs");
        d3d12::command_buffer::clear_render_texture(cmd, m_ColorTexture, float4({ 0.0, 0.0, 0.0, 0.0 }));
        if (m_FrameIndex == 0)
            d3d12::command_buffer::clear_render_texture(cmd, m_HistoryTexture, float4({ 0.0, 0.0, 0.0, 0.0 }));
        d3d12::command_buffer::clear_depth_stencil_texture(cmd, m_DepthBuffer, 1.0f, 0);

        d3d12::command_buffer::end_section(cmd);
    }

    // Render the frustum inside the volume
    if (m_RenderFrustum)
    {
        d3d12::command_buffer::set_render_texture(cmd, m_ColorTexture, m_DepthBuffer);
        d3d12::command_buffer::set_viewport(cmd, 0, 0, m_ScreenSizeI.x, m_ScreenSizeI.y);
        m_FrustumRenderer.render_under(cmd, m_GlobalCB);
    }

    // Render the volume
    m_ProfilingHelper.start_profiling(m_CmdBuffer, 0);
    if (m_LEBPath)
        m_LEBRenderer.render_volume(m_CmdBuffer, m_GlobalCB, m_ColorTexture, m_DepthBuffer, m_RenderingMode, m_Sky, camera);
    else
        m_GridRenderer.render_volume(m_CmdBuffer, m_GlobalCB, m_ColorTexture, m_DepthBuffer, m_RenderingMode, m_Sky, camera);
    m_ProfilingHelper.end_profiling(m_CmdBuffer, 0);

    // Accumulate
    if (m_RenderingMode == RenderingMode::ForwardPT)
    {
        d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_AccumulateFrameCS, "_GlobalCB", m_GlobalCB);
        d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_AccumulateFrameCS, "_SampleTexture", m_ColorTexture);
        d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_AccumulateFrameCS, "_HistoryTexture", m_HistoryTexture);
        d3d12::command_buffer::dispatch(cmd, m_AccumulateFrameCS, ((uint32_t)m_ScreenSize.x + 7) / 8, ((uint32_t)m_ScreenSize.y + 7) / 8, 1);
    }
    else if (m_RenderingMode == RenderingMode::DebugView)
    {
        d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_TonemapFrameCS, "_GlobalCB", m_GlobalCB);
        d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_TonemapFrameCS, "_ColorTexture", m_ColorTexture);
        d3d12::command_buffer::dispatch(cmd, m_TonemapFrameCS, ((uint32_t)m_ScreenSize.x + 7) / 8, ((uint32_t)m_ScreenSize.y + 7) / 8, 1);
    }

    // The the render viewport
    d3d12::command_buffer::set_render_texture(cmd, m_ColorTexture, m_DepthBuffer);
    d3d12::command_buffer::set_viewport(cmd, 0, 0, m_ScreenSizeI.x, m_ScreenSizeI.y);
    
    // Render the frustum outside the volume
    if (m_RenderFrustum)
        m_FrustumRenderer.render_above(cmd, m_GlobalCB);

    // Render the UI
    render_ui(cmd, m_ColorTexture);

    // Grab the current swap chain render target
    RenderTexture colorBuffer = d3d12::swap_chain::get_current_render_texture(m_SwapChain);

    // Copy our texture to the swap chain RT
    d3d12::command_buffer::copy_render_texture(cmd, m_ColorTexture, colorBuffer);

    // Set the render target in present mode
    d3d12::command_buffer::transition_to_present(cmd, colorBuffer);

    // Close the command buffer
    d3d12::command_buffer::close(cmd);

    // Execute the command buffer in the command queue
    d3d12::command_queue::execute_command_buffer(m_CmdQueue, cmd);

    // Present
    d3d12::swap_chain::present(m_SwapChain);

    // Flush the queue
    d3d12::command_queue::flush(m_CmdQueue);

    // Process the scopes
    m_ProfilingHelper.process_scopes(m_CmdQueue);
}

void VolumePipeline::render_loop()
{
    // Show the window
    d3d12::window::show(m_Window);

    // All required initializations before the render loop
    prepare_rendering(m_CmdBuffer);

    // Render loop
    bool activeLoop = true;
    m_FrameIndex = 0;
    m_Time = 0.0;
    while (activeLoop)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Handle the messages
        d3d12::window::handle_messages(m_Window);
        uint2 windowCenter = d3d12::window::window_center(m_Window);

        bool resetCursorToCenter = false;
        // Process the events
        EventData eventData;
        while (event_collector::peek_event(eventData))
        {
            switch (eventData.type)
            {
            case FrameEvent::Raw:
                d3d12::imgui::handle_input(m_Window, eventData);
                break;
            case FrameEvent::MouseMovement:
                resetCursorToCenter |= m_CameraController.process_mouse_movement({ (int)eventData.data0, (int)eventData.data1 }, windowCenter, m_ScreenSize);
                break;
            case FrameEvent::MouseWheel:
                m_CameraController.process_mouse_wheel((int)eventData.data0);
                break;
            case FrameEvent::MouseButton:
                resetCursorToCenter |= m_CameraController.process_mouse_button((MouseButton)eventData.data0, eventData.data1 != 0);
                break;
            case FrameEvent::KeyDown:
                process_key_event(eventData.data0, true);
                break;
            case FrameEvent::KeyUp:
                process_key_event(eventData.data0, false);
                break;
            case FrameEvent::Close:
            case FrameEvent::Destroy:
                activeLoop = false;
                break;
            }
        }

        // Reset the cursor to the center if requested
        if (resetCursorToCenter)
        {
            m_FrameIndex = 0;
            d3d12::window::set_cursor_pos(m_Window, windowCenter);
        }

        // Draw if needed
        double deltaTime = 0.0f;
        if (event_collector::active_draw_request())
        {
            render_pipeline(m_CmdBuffer);
            m_FrameIndex++;
            event_collector::draw_done();
        }

        // Evaluate the time
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        deltaTime = duration.count() / 1e6;

        // Add to the time
        m_Time += deltaTime;

        // Update the scene
        update(deltaTime);
    }
}

void VolumePipeline::update(double deltaTime)
{
    // Update the controller
    m_CameraController.update(deltaTime);
}
