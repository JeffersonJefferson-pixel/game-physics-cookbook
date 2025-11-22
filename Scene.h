#ifndef _H_SCENE_
#define _H_SCENE_

#include "Geometry3D.h"
#include <vector>

class Scene {
protected:
    std::vector<Model*> objects;
public: 
    // add model to the scene.
    void AddModel(Model* model);
    // remoev model from the scene.
    void RemoveModel(Model* model);
    // update model that has moved since last frame.
    void UpdateModel(Model* model);
    std::vector<Model*> FindChildren(const Model* model);
    // cast ray into the scene and return the model that was hit.
    Model* Raycast(const Ray& ray);
    // resturn models in a sphere.
    std::vector<Model*> Query(const Sphere& sphere);
    // resturn models in an aabb.
    std::vector<Model*> Query(const AABB& aabb);
};

#endif