#include "Cloth.h"
#include "glad/glad.h"
#include "FixedFunctionPrimitives.h"

void Cloth::Initialize(int gridSize, float distance, const vec3& position) {
    float k = -1.0f;
    float b = 0.0f;
    clothSize = gridSize;
    verts.clear();
    structural.clear();
    shear.clear();
    bend.clear();
    verts.resize(gridSize * gridSize);
    // half width and depth
    float hs = (float)(gridSize - 1) * 0.5f;

    // at least 9 particles for stable cloth
    if (gridSize < 3) {
        gridSize = 3;
    }

    // create particles for the cloth vertices
    for (int x = 0; x < gridSize; ++x) {
        for (int z = 0; z < gridSize; ++z) {
            int i = z * gridSize + x;
            // position of particle
            float x_pos = ((float)x + position.x - hs) * distance;
            float z_pos = ((float)z + position.z - hs) * distance;
            verts[i].SetPosition(vec3(x_pos, position.y, z_pos));
            verts[i].SetMass(1.0f);
            verts[i].SetBounce(0.0f);
            verts[i].SetFriction(0.9f);
        }
    }

    // structual spring left-rigth
    for (int x = 0; x < gridSize; ++x) {
        for (int z = 0; z < gridSize - 1; ++z) {
            int i = z * gridSize + x;
            int j = (z + 1) * gridSize + x;
            
            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            structural.push_back(spring);
        }
    }

    // structual spring up-down
    for (int x = 0; x < gridSize - 1; ++x) {
        for (int z = 0; z < gridSize; ++z) {
            int i = z * gridSize + x;
            int j = z * gridSize + (x + 1);

            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            structural.push_back(spring);
        }
    }

    // shear spring left-right
    for (int x = 0; x < gridSize - 1; ++x) {
        for (int z = 0; z < gridSize - 1; ++z) {
            int i = z * gridSize + x;
            int j = (z + 1) * gridSize + (x + 1);
            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            shear.push_back(spring);
        }
    }

    for (int x = 1; x < gridSize; ++x) {
        for (int z = 0; z < gridSize - 1; ++z) {
            int i = z * gridSize + x;
            int j = (z + 1) * gridSize + (x - 1);
            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            shear.push_back(spring);
        }
    }

    // bend spring
    for (int x = 0; x < gridSize; ++x) {
        for (int z = 0; z < gridSize - 2; ++z) {
            int i = z * gridSize + x;
            int j = (z + 2) * gridSize + x;
            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            bend.push_back(spring);
        }
    }

    // bend spring
    for (int x = 0; x < gridSize - 2; ++x) {
        for (int z = 0; z < gridSize; ++z) {
            int i = z * gridSize + x;
            int j = z * gridSize + (x + 2);
            vec3 iPos = verts[i].GetPosition();
            vec3 jPos = verts[j].GetPosition();
            float rest = Magnitude(iPos - jPos);
            Spring spring(k, b, rest);
            spring.SetParticles(&verts[i], &verts[j]);
            bend.push_back(spring);
        }
    }
}

void Cloth::SetStructuralSprings(float k, float b) {
    for (int i = 0; i < structural.size(); ++i) {
        structural[i].SetConstants(k, b);
    }
}

void Cloth::SetShearSprings(float k, float b) {
    for (int i = 0; i < shear.size(); ++i) {
        shear[i].SetConstants(k, b);
    }
}

void Cloth::SetBendSprings(float k, float b) {
    for (int i = 0; i < bend.size(); ++i) {
        bend[i].SetConstants(k, b);
    }
}

void Cloth::SetParticleMass(float mass) {
    for (int i = 0; i < verts.size(); ++i) {
        verts[i].SetMass(mass);
    }
}

void Cloth::ApplyForces() {
    for (int i = 0; i < verts.size(); ++i) {
        verts[i].ApplyForces();
    }
}

void Cloth::Update(float dt) {
    for (int i = 0; i < verts.size(); ++i) {
        verts[i].Update(dt);
    }
}

void Cloth::ApplySpringForces(float dt) {
    // apply forces on all springs in cloth.
    for (int i = 0; i < structural.size(); ++i) {
        structural[i].ApplyForce(dt);
    }

    for (int i = 0; i < shear.size(); ++i) {
        shear[i].ApplyForce(dt);
    }

    for (int i = 0; i < bend.size(); ++i) {
        bend[i].ApplyForce(dt);
    }
}

void Cloth::SolveConstraints(const std::vector<OBB>& constraints) {
    // solve constraints for all particles in cloth
    for (int i = 0; i < verts.size(); ++i) {
        verts[i].SolveConstraints(constraints);
    }
}

void Cloth::Render() {
    for (int x = 0; x < clothSize - 1; ++x) {
        for (int z = 0; z < clothSize - 1; ++z) {
            // 4 particles as vertices
            int tl = z * clothSize + x;
            int bl = (z + 1) * clothSize + x;
            int tr = z * clothSize + (x + 1);
            int br = (z + 1) * clothSize + (x + 1);

            Triangle t1(
                verts[tl].GetPosition(),
                verts[br].GetPosition(),
                verts[bl].GetPosition()
            );
            Triangle t2(
                verts[tl].GetPosition(),
                verts[tr].GetPosition(),
                verts[br].GetPosition()
            );

            ::Render(t1);
            ::Render(t2);
        }
    }
}