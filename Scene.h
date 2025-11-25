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

// octree
// acceleration structure for scene.

typedef struct OctreeNode {
    AABB bounds;
    OctreeNode* children;
    std::vector<Model*> models;
    inline OctreeNode() : children(0) {}
    inline ~OctreeNode() {
        if (children != 0) {
            delete[] children;
        }
    }
} OctreeNode;

void SplitTree(OctreeNode* node, int depth);
void Insert(OctreeNode* node, Model* model);
void Remove(OctreeNode* node, Model* model);
void Update(OctreeNode* node, Model* model);

Model* FindClosest(const std::vector<Model*>& set, const Ray& ray);
Model* Raycast(OctreeNode* node, const Ray& ray);
std::vector<Model*> Query(OctreeNode* node, const Sphere& sphere);
std::vector<Model*> Query(OctreeNode* node, const AABB& aabb);
#endif