// Internal includes
#include "graphics/dx12_backend.h"
#include "render_pipeline/camera_controller.h"
#include "imgui/imgui.h"
#include "math/operators.h"

// External includes
#include <algorithm>
#include <sstream>
#include <fstream>

CameraController::CameraController()
{
}

CameraController::~CameraController()
{
}

void CameraController::initialize(RenderWindow renderWindow, uint32_t width, uint32_t height, float fov)
{
    // Keep track of the window
    m_Window = renderWindow;

    // Camera properties
    m_Camera.fov = fov;
    m_Camera.aspectRatio = width / (float)height;
    m_Camera.position = {0.0, 0.0, 0.0};
    m_Camera.angles = { 0.5, 0.0, 0.0 };
    m_Camera.scaleOffset = { 0.0f, 0.0f, 0.0f };
    m_Speed = 0.1;

    // Init control
    m_ActiveInteraction = false;
}

void CameraController::process_key_event(uint32_t keyCode, bool state)
{
    switch (keyCode)
    {
        case 0x44: // D
            m_ControllerStates[(uint32_t)NavigationButtons::Right] = state;
            break;
        case 0x51: // Q
            m_ControllerStates[(uint32_t)NavigationButtons::Left] = state;
            break;
        case 0x5A: // Z
            m_ControllerStates[(uint32_t)NavigationButtons::Forward] = state;
            break;
        case 0x53: // S
            m_ControllerStates[(uint32_t)NavigationButtons::Backward] = state;
            break;
        case 0x41: // A
            m_ControllerStates[(uint32_t)NavigationButtons::Up] = state;
            break;
        case 0x45: // E
            m_ControllerStates[(uint32_t)NavigationButtons::Down] = state;
            break;
        case 0x10: // Shift
            m_ControllerStates[(uint32_t)NavigationButtons::Shift] = state;
            break;
    }
}

bool CameraController:: process_mouse_button(MouseButton button, bool)
{
    if (button == MouseButton::Right)
    {
        d3d12::window::set_cursor_visibility(m_Window, m_ActiveInteraction);
        m_ActiveInteraction = !m_ActiveInteraction;
        return true;
    }
    return false;
}

bool CameraController::process_mouse_movement(int2 mouse, uint2 windowCenter, float4 screenSize)
{
    if (m_ActiveInteraction)
    {
        int2 mouv = int2({ mouse.x - (int)windowCenter.x, mouse.y - (int)windowCenter.y });
        m_Camera.angles.x -= mouv.x / screenSize.x * 5.0f;
        m_Camera.angles.y -= mouv.y / screenSize.y * 5.0f;
        return true;
    }
    return false;
}

void CameraController::process_mouse_wheel(int wheel)
{
    if (m_ActiveInteraction)
    {
        if (wheel > 0)
            m_Speed *= 2.0;
        else
            m_Speed /= 2.0;
    }
}

void CameraController::update(double deltaTime)
{
    if (m_ActiveInteraction)
    {
        float3 forwardDir = float3({ m_Camera.view.m[2], m_Camera.view.m[6], m_Camera.view.m[10] });
        float3 rightDir = float3({ m_Camera.view.m[0], m_Camera.view.m[4], m_Camera.view.m[8] });
        float speed = m_Speed * (float)deltaTime;

        float3 displacement = float3({ 0.0, 0.0, 0.0 });
        if (m_ControllerStates[(uint32_t)NavigationButtons::Forward])
        {
            displacement.x += forwardDir.x * speed;
            displacement.y += forwardDir.y * speed;
            displacement.z += forwardDir.z * speed;
        }

        if (m_ControllerStates[(uint32_t)NavigationButtons::Backward])
        {
            displacement.x -= forwardDir.x * speed;
            displacement.y -= forwardDir.y * speed;
            displacement.z -= forwardDir.z * speed;
        }

        if (m_ControllerStates[(uint32_t)NavigationButtons::Left])
        {
            displacement.x -= rightDir.x * speed;
            displacement.y -= rightDir.y * speed;
            displacement.z -= rightDir.z * speed;
        }

        if (m_ControllerStates[(uint32_t)NavigationButtons::Right])
        {
            displacement.x += rightDir.x * speed;
            displacement.y += rightDir.y * speed;
            displacement.z += rightDir.z * speed;
        }

        m_Camera.position = m_Camera.position + displacement;
    }

    // Position has been updated, update the matrices
    evaluate_camera_matrices();
}

void CameraController::evaluate_camera_matrices()
{
    // Evaluate the projection matrix
    m_Camera.projection = projection_matrix(m_Camera.fov, m_Camera.nearFar.x, m_Camera.nearFar.y, m_Camera.aspectRatio);

    // Update the view matrix
    const float4x4 rotation_z = rotation_matrix_z(m_Camera.angles.z);
    const float4x4 rotation_y = rotation_matrix_y(m_Camera.angles.x);
    const float4x4 rotation_x = rotation_matrix_x(m_Camera.angles.y);
    m_Camera.view = mul(rotation_z, mul(rotation_x, rotation_y));


    // Update the compound matrices
    m_Camera.viewProjection = mul(m_Camera.projection, m_Camera.view);

    // Compute the inverse matrices
    m_Camera.invViewProjection = inverse(m_Camera.viewProjection);
}