#ifndef _H_SPRING_
#define _H_SPRING_

#include "Particle.h"

class Spring {
protected:
    Particle* p1;
    Particle* p2;

    // higher k mean stiffer spring. ranges from -n to 0
    float k;
    // ranges from 0 to 1
    float b;
    float restingLength;

public:
    inline Spring(float _k, float _b, float len) : k(_k), b(_b), restingLength(len) {}
    Particle* GetP1();
    Particle* GetP2();
    void SetParticles(Particle* _p1, Particle* _p2);
    void SetConstants(float _k, float _b);
    void ApplyForce(float dt);
};

#endif