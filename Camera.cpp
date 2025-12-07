#include "Camera.h"
#include "Compare.h"

Camera::Camera() {
    m_nFov = 60.0f;
    m_nAspect = 1.3f;
    m_nNear = 0.01f;
    m_nFar = 1000.0f;
    m_nWidth = 1.0;
    m_nHeight = 1.0f;

    m_matWorld = mat4();
    m_matProj = Projection(m_nFov, m_nAspect, m_nNear, m_nFar);
    m_nProjectionMode = 0;
}

mat4 Camera::GetWorldMatrix() {
    return m_matWorld;
}

mat4 Camera::GetViewMatrix() {
    if (!IsOrthoNormal()) {
        OrthoNormalize();
    }
    // transpose to invert rotation matrix.
    mat4 inverse = Transpose(m_matWorld);
    inverse._41 = inverse._14 = 0.0f;
    inverse._42 = inverse._24 = 0.0f;
    inverse._43 = inverse._34 = 0.0f;

    // right up forward vectors from world matrix
    vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
    vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
    vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

    // position vector
    vec3 position = vec3(m_matWorld._41, m_matWorld._42, m_matWorld._43);

    inverse._41 = -Dot(right, position);
    inverse._42 = -Dot(up, position);
    inverse._43 = -Dot(forward, position);

    return inverse;
}

mat4 Camera::GetProjectionMatrix() {
    return m_matProj;
}

float Camera::GetAspect() {
    return m_nAspect;
}

bool Camera::IsOrthographic() {
    return m_nProjectionMode == 1;
}

bool Camera::IsPerspective() {
    return m_nProjectionMode == 0;
}

bool Camera::IsOrthoNormal() {
    vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
    vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
    vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

    // check unit length
    if (!CMP(Dot(right, right), 1.0f) || 
        !CMP(Dot(up, up), 1.0f) || 
        !CMP(Dot(forward, forward), 1.0f)) {
            return false;
        }

    // check perpendicular
    if (!CMP(Dot(forward, up), 0.0f) ||
        !CMP(Dot(forward, right), 0.0f) || 
        !CMP(Dot(right, up), 0.0f)) {
            return false;
        }
    
    return true;
}

void Camera::OrthoNormalize() {
    vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
    vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
    vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

    // compute perpendicular basis vector
    vec3 f = Normalized(forward);
    vec3 r = Normalized(Cross(up, f));
    vec3 u = Cross(f, r);

    // construct new world matrix with the new basis vector.
    m_matWorld = mat4(
        r.x, r.y, r.z, 0.0f,
        u.x, u.y, u.z, 0.0f,
        f.x, f.y, f.z, 0.0f,
        m_matWorld._41, m_matWorld._42, m_matWorld._43, 0.0f
    );
}

void Camera::Resize(int width, int height) {
    m_nAspect = (float) width / (float) height;
    // rebuild projection matrix
    if (m_nProjectionMode == 0) {
        m_matProj = Projection(m_nFov, m_nAspect, m_nNear, m_nFar);
    } else if (m_nProjectionMode == 1) {
        m_nWidth = (float) width;
        m_nHeight = (float) height;
        float halfW = m_nWidth * 0.5f;
        float halfH = m_nHeight * 0.5f;
        m_matProj = Ortho(-halfW, halfW, halfH, -halfH, m_nNear, m_nFar);
    }
    // projectionMode == 2 is user defined
}

void Camera::Perspective(float fov, float aspect, float zNear, float zFar) {
    m_nFov = fov;
    m_nAspect = aspect;
    m_nNear = zNear;
    m_nFar = zFar;
    m_matProj = Projection(fov, aspect, zNear, zFar);
    m_nProjectionMode = 0;
}

void Camera::Orthographic(float width, float height, float zNear, float zFar) {
    m_nWidth = width;
    m_nHeight = height;
    m_nNear = zNear;
    m_nFar = zFar;
    float halfW = width * 0.5f;
    float halfH = height * 0.5f;
    m_matProj = Ortho(-halfW, halfW, halfH, -halfH, zNear, zFar);
    m_nProjectionMode = 1;
}

void Camera::SetProjection(const mat4& projection) {
    m_matProj = projection;
    m_nProjectionMode = 2;
}

void Camera::SetWorld(const mat4& view) {
    m_matWorld = view;
}

OrbitCamera::OrbitCamera() {
    target = vec3(0, 0, 0);
    zoomDistance = 10.f;
    zoomSpeed = 200.0f;
    rotationSpeed = vec2(250.0f, 150.0f);
    yRotationLimit = vec2(-20.f, 80.f);
    zoomDistanceLimit = vec2(3.0f, 15.0f);
    currentRotation = vec2(0, 0);
    panSpeed = vec2(180.0f, 180.0f);
}

void OrbitCamera::Rotate(const vec2& deltaRot, float deltaTime) {
    // update rotation
    currentRotation.x += deltaRot.x * rotationSpeed.x * zoomDistance * deltaTime;
    currentRotation.y += deltaRot.y * rotationSpeed.y * zoomDistance * deltaTime;
    // clamp angle
    currentRotation.x = ClampAngle(currentRotation.x, -360, 360);
    currentRotation.y = ClampAngle(currentRotation.y, yRotationLimit.x, yRotationLimit.y);
}

void OrbitCamera::Zoom(float deltaZoom, float deltaTime) {
    zoomDistance = zoomDistance + deltaZoom * zoomSpeed * deltaTime;
    if (zoomDistance < zoomDistanceLimit.x) {
        zoomDistance = zoomDistanceLimit.x;
    }
    if (zoomDistance > zoomDistanceLimit.y) {
        zoomDistance =  zoomDistanceLimit.y;
    }
}

void OrbitCamera::Pan(const vec2& deltaPan, float deltaTime) {
    vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
    // pan on the x axis in local space.
    float xPanMag = deltaPan.x * panSpeed.x * deltaTime;
    target = target - (right * xPanMag);
    // pan on the y axis in global space.
    float yPanMag = deltaPan.y * panSpeed.y * deltaTime;
    target = target + (vec3(0, 1, 0) * yPanMag);
}

void OrbitCamera::Update(float dt) {
    vec3 rotation = vec3(currentRotation.y, currentRotation.x, 0);
    mat3 orient = Rotation3x3(rotation.x, rotation.y, rotation.z);
    vec3 direction = MultiplyVector(vec3(0.0, 0.0, -zoomDistance), orient);
    vec3 position = direction + target;
    // world matrix inverse from view matrix
    m_matWorld = Inverse(LookAt(position, target, vec3(0, 1 ,0)));
}

float OrbitCamera::ClampAngle(float angle, float min, float max) {
    // keep angle between -360 and 360 degrees.
    while (angle < -360) {
        angle += 360;
    }
    while (angle > 360) {
        angle -= -360;
    }
    if (angle < min) {
        angle = min;
    }
    if (angle > max) {
        angle = max;
    }
    return angle;
}

Frustum Camera::GetFrustum() {
    Frustum result;
    mat4 vp = GetViewMatrix() * GetProjectionMatrix();
    vec3 col1(vp._11, vp._21, vp._31);
    vec3 col2(vp._12, vp._22, vp._32);
    vec3 col3(vp._13, vp._23, vp._33);
    vec3 col4(vp._14, vp._24, vp._34);
    // normal of planes
    result.left.normal = col4 + col1;
    result.right.normal = col4 - col1;
    result.bottom.normal = col4 + col2;
    result.top.normal = col4 - col2;
    result._near.normal = col3;
    result._far.normal = col4 - col3;
    // distance of planes.
    result.left.distance = vp._44 + vp._41;
    result.right.distance = vp._44 - vp._41;
    result.bottom.distance = vp._44 + vp._42;
    result.top.distance = vp._44 - vp._42;
    result._near.distance = vp._43;
    result._far.distance = vp._44 - vp._43;
    // normalize
    for (int i = 0; i < 6; ++i) {
        float mag = 1.0f / Magnitude(result.planes[i].normal);
        result.planes[i].normal = result.planes[i].normal * mag;
        result.planes[i].distance *= mag;
    }
    
    return result;
}