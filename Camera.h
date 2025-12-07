#ifndef _H_CAMERA_
#define H_CAMERA_
#include "matrices.h"
#include "Geometry3D.h"

class Camera
{
protected:
    // projection matrix variables
    // field of view
    float m_nFov;
    // aspect ratio
    float m_nAspect;
    float m_nNear;
    float m_nFar;
    float m_nWidth;
    float m_nHeight;
    // world transform
    mat4 m_matWorld;
    // inverse world
    mat4 m_matProj;
    // 0: perspective, 1: ortho, 2: user
    int m_nProjectionMode;

public:
    Camera();
    inline virtual ~Camera() {}
    // matrix functions.
    mat4 GetWorldMatrix();
    mat4 GetViewMatrix();
    mat4 GetProjectionMatrix();
    void SetProjection(const mat4 &projection);
    void SetWorld(const mat4 &view);
    // helper functions.
    float GetAspect();
    bool IsOrthographic();
    bool IsPerspective();
    bool IsOrthoNormal();
    void OrthoNormalize();
    // rebuild projection matrix
    void Resize(int width, int height);
    void Perspective(float fov, float aspect, float zNear, float zFar);
    void Orthographic(float width, float height, float zNear, float zFar);

    // Get frustum from a view matrix.
    Frustum GetFrustum();
};

class OrbitCamera : public Camera
{
protected:
    // thing camera is looking at.
    vec3 target;
    // how fast the camera moves.
    vec2 panSpeed;
    // how far camera is from the target.
    float zoomDistance;
    vec2 zoomDistanceLimit;
    float zoomSpeed;
    // rotation around y and x axis.
    vec2 rotationSpeed;
    vec2 yRotationLimit;
    vec2 currentRotation;

    float ClampAngle(float angle, float min, float max);
public:
    OrbitCamera();
    inline virtual ~OrbitCamera() {}

    void Rotate(const vec2& deltaRot, float deltaTime);
    void Zoom(float deltaZoom, float deltaTime);
    void Pan(const vec2& deltaPan, float deltaTime);
    
    void Update(float dt);
};

#endif