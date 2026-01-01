#include "PhysicSystem.h"
#include "RigidbodyVolume.h"
#include "FixedFunctionPrimitives.h"
#include "glad/glad.h"

PhysicsSystem::PhysicsSystem() {
    LinearProjectionPercent = 0.45f;
    PenetrationSlack = 0.01f;
    ImpluseIteration = 5;

    colliders1.reserve(100);
    colliders2.reserve(100);
    results.reserve(100);
}

void PhysicsSystem::AddRigidbody(Rigidbody* body) {
    bodies.push_back(body);
}

void PhysicsSystem::AddConstraint(const OBB& obb) {
    constraints.push_back(obb);
}

void PhysicsSystem::ClearRigidbodys() {
    bodies.clear();
}

void PhysicsSystem::ClearConstraints() {
    constraints.clear();
}

void PhysicsSystem::Render() {
    // rigid body colors
    static const float rigidbodyDiffuse[]{ 200.0f / 255.0f, 0.0f, 0.0f, 0.0f };
	static const float rigidbodyAmbient[]{ 200.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f, 0.0f };

	static const float groundDiffuse[]{ 0.0f, 0.0f, 200.0f / 255.0f, 0.0f };
	static const float groundAmbient[]{ 50.0f / 255.0f, 50.0f / 255.0f, 200.0f / 255.0f, 0.0f };

	static const float constraintDiffuse[]{ 0.0f, 200.0f / 255.0f, 0.0f, 0.0f };
	static const float constraintAmbient[]{ 50.0f / 255.0f, 200.0f / 255.0f, 50.0f / 255.0f, 0.0f };
	
	static const float zero[] = { 0.0f, 0.0f, 0.0f, 0.0f };

    glColor3f(rigidbodyDiffuse[0], rigidbodyDiffuse[1], rigidbodyDiffuse[2]);
    glLightfv(GL_LIGHT0, GL_AMBIENT, rigidbodyAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, rigidbodyDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

    // render rigid bodies
    for (int i = 0, size = bodies.size(); i < size; ++i) {
        bodies[i]->Render();
    }

    // constraint color
    glColor3f(constraintDiffuse[0], constraintDiffuse[1], constraintDiffuse[2]);
    glLightfv(GL_LIGHT0, GL_AMBIENT, constraintAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, constraintDiffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, zero);

    // render constraints
    for (int i = 0; i < constraints.size(); ++i) {
        ::Render(constraints[i]);
    }
}

void PhysicsSystem::Update(float deltaTime) {
    // reset collision pairs.
    colliders1.clear();
    colliders2.clear();
    results.clear();

    // find pair of colliding bodies
    for (int i = 0, size = bodies.size(); i < size; ++i) {
        for (int j = i; j < size; ++j) {
            if (i == j) {
                continue;
            }
            CollisionManifold result;
            ResetCollisionManifold(&result);
            if (bodies[i]->HasVolume() && bodies[j]->HasVolume()) {
                RigidbodyVolume* m1 = (RigidbodyVolume*)bodies[i];
                RigidbodyVolume* m2 = (RigidbodyVolume*)bodies[j];
                result = FindCollisionFeatures(*m1, *m2);
            }
            if (result.colliding) {
                colliders1.push_back(bodies[i]);
                colliders2.push_back(bodies[j]);
                results.push_back(result);
            }
        }
    }

    // sum forces on rigid bodies
    for (int i = 0, size = bodies.size(); i < size; ++i) {
        bodies[i]->ApplyForces();
    }

    // apply impulse
    for (int k = 0; k < ImpluseIteration; ++k) {
        for (int i = 0; i < results.size(); ++i) {
            int jSize = results[i].contacts.size();
            for (int j = 0; j < jSize; ++j) {
                if (colliders1[i]->HasVolume() && colliders2[i]->HasVolume()) {
                    RigidbodyVolume* m1 = (RigidbodyVolume*)colliders1[i];
                    RigidbodyVolume* m2 = (RigidbodyVolume*)colliders2[i];
                    ApplyImpulse(*m1, *m2, results[i], j);
                }
            }
        }
    }

    // update rigid bodies position
    for (int i = 0, size = bodies.size(); i < size; ++i) {
        bodies[i]->Update(deltaTime);
    }

    // linear projection to fix sinking issues
    for (int i = 0, size = results.size(); i < size; ++i) {
        if (!colliders1[i]->HasVolume() && !colliders2[i]->HasVolume()) {
            continue;
        }
        RigidbodyVolume* m1 = (RigidbodyVolume*)colliders1[i];
        RigidbodyVolume* m2 = (RigidbodyVolume*)colliders2[i];
        float totalMass = m1->InvMass() + m2->InvMass();
        if (totalMass == 0.0f) {
            continue;
        }
        float depth = fmaxf(results[i].depth - PenetrationSlack, 0.0f);
        float scalar = depth / totalMass;
        vec3 correction = results[i].normal * scalar * LinearProjectionPercent;
        m1->position = m1->position - correction * m1->InvMass();
        m2->position = m2->position + correction * m2->InvMass();
        m1->SynchCollisionVolumes();
        m2->SynchCollisionVolumes();
    }

    // keep rigid bodies from moving through constraints.
    for (int i = 0, size = bodies.size(); i < size; ++i) {
        bodies[i]->SolveConstraints(constraints);
    }
}