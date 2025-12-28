#include "Particle.h"
#include "Geometry3D.h"
#include "FixedFunctionPrimitives.h"

void Particle::SetPosition(const vec3& pos)  {
    position = oldPosition = pos;
}

vec3 Particle::GetPosition() {
    return position;
}

void Particle::SetBounce(float b) {
    bounce = b;
}

float Particle::GetBounce() {
    return bounce;
}

Particle::Particle() {
    type = RIGIDBODY_TYPE_PARTICLE;
    friction = 0.95f;
    gravity = vec3(0.0f, -9.82f, 0.0f);
    mass = 1.0f;
    bounce = 0.7f;
}

void Particle::Render() {
    // render sphere at the position of particle.
    Sphere visual(position, 0.1f);
    ::Render(visual);
}

void Particle::SolveConstraints(const std::vector<OBB>& constraints) {
    int size = constraints.size();
    for (int i = 0; i < size; ++i) {
        Line traveled(oldPosition, position);

        // check if particle collided with obstacle with line test to avoid tunneling
        if (Linetest(constraints[i], traveled)) {
            // raycast against obstacle
            vec3 velocity = position - position;
            vec3 direction = Normalized(velocity);
            Ray ray(oldPosition, direction);
            RaycastResult result;
            if (Raycast(constraints[i], ray, &result)) {
                // move particle to collision point
                position = result.point + result.normal * 0.003f;
                // deconstruct velocity into parallel and perpendicular
                vec3 vn = result.normal * Dot(result.normal, velocity);
                vec3 vt = velocity - vn;
                // old position before collision bounce 
                oldPosition = position - (vt - vn * bounce);
                break;
            }
        }
    }
}


void Particle::ApplyForces() {
    forces = gravity;
}

void Particle::Update(float deltaTime) {
    // move particle using verlet integration
    vec3 velocity = position - oldPosition;
    oldPosition = position;
    float deltaSquare = deltaTime * deltaTime;

    position = position + (velocity * friction) + (forces * deltaSquare);
}