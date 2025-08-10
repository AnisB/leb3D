#pragma once

// SDK includes
#include "render_pipeline/camera.h"
#include "graphics/types.h"
#include "graphics/event_collector.h"

// System includes
#include <string>

// Buttons that control the camera movements
enum class NavigationButtons
{
    Forward = 0,
    Backward,
    Left,
    Right,
    Up,
    Down,
    Shift,
    Count
};

class CameraController
{
public:
    CameraController();
    ~CameraController();

    // Initialize the controller
    virtual void initialize(RenderWindow renderWindow, uint32_t width, uint32_t height, float fov);

    // Process key event
    virtual void process_key_event(uint32_t keyCode, bool state);

    // Process a mouse mouvement
    virtual bool process_mouse_movement(int2 mouse, uint2 windowCenter, float4 screenSize);
    virtual void process_mouse_wheel(int wheel);
    virtual bool process_mouse_button(MouseButton button, bool state);

    // Apply the delta time
    virtual void update(double deltaTime);
    virtual void evaluate_camera_matrices();

    // Get camera
    const Camera& get_camera() const { return m_Camera; }
    Camera& get_camera() { return m_Camera; }

protected:
    // Render window
    RenderWindow m_Window = 0;

    // The camera that the controller is handeling
    Camera m_Camera = Camera();
    
    // Button controls
    bool m_ControllerStates[(uint32_t)NavigationButtons::Count] = { false, false, false, false, false, false, false };

    // Flag that defines if we can interact with the camera
    bool m_ActiveInteraction = false;

    // Speed
    float m_Speed = 0.0f;
};
