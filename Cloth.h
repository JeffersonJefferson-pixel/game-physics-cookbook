#ifndef _H_CLOTH_
#define _H_CLOTH_

#include "Spring.h"
#include <vector>

class Cloth {
protected:
    std::vector<Particle> verts;
    std::vector<Spring> structural;
    std::vector<Spring> shear;
    std::vector<Spring> bend;
    float clothSize;
public:
    void Initialize(int gridSize, float distance, const vec3& position);
    void SetStructuralSprings(float k, float b);
    void SetShearSprings(float k, float b);
    void SetBendSprings(float k, float b);
    void SetParticleMass(float mass);

    void ApplyForces();
    void Update(float dt);
    void ApplySpringForces(float dt);
    void SolveConstraints(const std::vector<OBB>& constraints);
    void Render();
};

#endif