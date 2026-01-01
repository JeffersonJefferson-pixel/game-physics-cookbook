#ifndef _H_PHYSICS_SYSTEM_
#define _H_PHYSICS_SYSTEM_

#include "Rigidbody.h"

// collection of rigid bodies and constraints.
class PhysicsSystem
{
protected:
    std::vector<Rigidbody *> bodies;
    std::vector<OBB> constraints;
    std::vector<Rigidbody*> colliders1;
    std::vector<Rigidbody*> colliders2;
    std::vector<CollisionManifold> results;

public:
    // how much positional correction to apply
    // smaller value allow oibjects to penetrate more.
    float LinearProjectionPercent;
    // how much objects are allwoed to penetrate.
    // the larger the number, the less jitter.
    float PenetrationSlack;
    // more iteration, more accurate
    int ImpluseIteration;

    PhysicsSystem();

    void Update(float deltaTime);
    void Render();

    void AddRigidbody(Rigidbody *body);
    void AddConstraint(const OBB &constraint);

    void ClearRigidbodys();
    void ClearConstraints();
};

#endif