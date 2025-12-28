#include "RigidbodyVolume.h"
#include "FixedFunctionPrimitives.h"
#include "Compare.h"

void RigidbodyVolume::ApplyForces() {
    forces = GRAVITY_CONST * mass;
}

void RigidbodyVolume::AddLinearImpulse(const vec3& impulse) {
    // impulse has immediate effect on velocity
    velocity = velocity + impulse;
}

float RigidbodyVolume::InvMass() {
    if (mass == 0.0f) { return 0.0f; }
    return 1.0f / mass;
}

void RigidbodyVolume::SynchCollisionVolumes() {
    sphere.position = position;
    box.position = position;
}

void RigidbodyVolume::Render() {
    SynchCollisionVolumes();
    if (type == RIGIDBODY_TYPE_SPHERE) {
        ::Render(sphere);
    }
    else if (type == RIGIDBODY_TYPE_BOX) {
        ::Render(box);
    }
}

void RigidbodyVolume::Update(float dt) {
    // air friction
    const float damping = 0.98f;
    vec3 acceleration = forces * InvMass();
    velocity = velocity + acceleration * dt;
    velocity = velocity * damping;
    position = position + velocity * dt;
    SynchCollisionVolumes();
}

CollisionManifold FindCollisionFeatures(RigidbodyVolume& ra, RigidbodyVolume& rb) {
    CollisionManifold result;
    ResetCollisionManifold(&result);

    if (ra.type == RIGIDBODY_TYPE_SPHERE) {
        if (rb.type == RIGIDBODY_TYPE_SPHERE) {
            result = FindCollisionFeatures(ra.sphere, rb.sphere);
        }
        else if (rb.type == RIGIDBODY_TYPE_BOX) {
            result = FindCollisionFeatures(rb.box, ra.sphere);
            // invert normal
            result.normal = result.normal * -1.0f;
        }
    } 
    else if (ra.type == RIGIDBODY_TYPE_BOX) {
        if (rb.type == RIGIDBODY_TYPE_BOX) {
            result = FindCollisionFeatures(ra.box, rb.box);
        }
        else if (rb.type == RIGIDBODY_TYPE_SPHERE) {
            result = FindCollisionFeatures(ra.box, rb.sphere);
        }
    }

    return result;
}

void ApplyImpulse(RigidbodyVolume& A, RigidbodyVolume& B, const CollisionManifold& M, int c) {
    float invMass1 = A.InvMass();
    float invMass2 = B.InvMass();
    float invMassSum = invMass1 + invMass2;
    if (invMassSum == 0.0f) { return; }
    // relative velocity
    vec3 relativeVel = B.velocity - A.velocity;
    vec3 relativeNorm = M.normal;
    Normalize(relativeNorm);
    if (Dot(relativeVel, relativeNorm) > 0.0f) {
        // moving away from each other
        return;
    }
    // magnitude of impulse
    float e = fminf(A.cor, B.cor);
    float numerator = (-(1.0f + e) * Dot(relativeVel, relativeNorm));
    float j = numerator / invMassSum;
    if (M.contacts.size() > 0.0f && j != 0.0f) {
        j /= (float) M.contacts.size();
    }
    vec3 impulse = relativeNorm * j;
    // apply impulse to velocity
    A.velocity = A.velocity - impulse * invMass1;
    B.velocity = B.velocity + impulse * invMass2;
    // friction
    // tangential
    vec3 t = relativeVel - (relativeNorm * Dot(relativeVel, relativeNorm));
    if (CMP(MagnitudeSq(t), 0.0f)) {
        return;
    }
    Normalize(t);
    // magnitude of friction
    numerator = -Dot(relativeVel, t);
    float jt = numerator / invMassSum;
    if (M.contacts.size() > 0.0f && jt != 0.0f) {
        jt /= (float) M.contacts.size();
    }
    if (CMP(jt, 0.0f)) {
        return;
    }
    // clamp friction
    float friction = sqrtf(A.friction * B.friction);
    if (jt > j * friction) {
        jt = j * friction;
    } else if (jt < -j * friction) {
        jt = -j * friction;
    }
    vec3 tangentImpulse = t * jt;

    A.velocity = A.velocity - tangentImpulse * invMass1;
    B.velocity = B.velocity + tangentImpulse * invMass2;
}