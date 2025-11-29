#include "Scene.h"
#include <algorithm>
#include <stack>

void Scene::AddModel(Model* model) {
    // only add if model is not already in the scene.
    if (std::find(objects.begin(), objects.end(), model) != objects.end()) {
        return;
    }

    objects.push_back(model);
}

void Scene::RemoveModel(Model* model) {
    objects.erase(std::remove(objects.begin(), objects.end(), model), objects.end());
}

void Scene::UpdateModel(Model* model) {

}

std::vector<Model*> Scene::FindChildren(const Model* model) {
    std::vector<Model*> result;
    for (int i = 0, size = objects.size(); i < size; ++i) {
        if (objects[i] == 0 || objects[i] == model) {
            continue;
        }
        // store if model is child of target model.
        Model* iterator = objects[i]->parent;
        while (iterator != 0) {
            if (iterator == model) {
                result.push_back(objects[i]);
                break;
            }
            iterator = iterator->parent;
        }
    }
}

Model* Scene::Raycast(const Ray& ray) {
    // raycast against octree if exists.
    if (octree != 0) {
        return ::Raycast(octree, ray);
    }
    Model* result = 0;
    float result_t = -1;
    // ray cast against all models in the scene and store minimum.
    for (int i = 0, size = objects.size(); i < size; ++i) {
        float t = ModelRay(*objects[i], ray);
        if (result == 0 && t >= 0) {
            result = objects[i];
            result_t = t;
        } else if (result != 0 && t < result_t) {
            result = objects[i];
            result_t = t;
        }
    }
    return result;
}

std::vector<Model*> Scene::Query(const Sphere& sphere) {
    // query against octree if exits.
    if (octree != 0) {
        return ::Query(octree, sphere);
    }
    std::vector<Model*> result;
    // loop through models in scene.
    for (int i = 0, size = objects.size(); i < size; ++i) {
        // bound of model in world space
        OBB bounds = GetOBB(*objects[i]);
        // test intersection between bounds and given sphere.
        if (SphereOBB(sphere, bounds)) {
            result.push_back(objects[i]);
        }
    }
    return result;
}

std::vector<Model*> Scene::Query(const AABB& aabb) {
    if (octree != 0) {
        return ::Query(octree, aabb);
    }
    std::vector<Model*> result;
    // loop through models in scene.
    for (int i = 0, size = objects.size(); i < size; ++i) {
        // bound of model in world space
        OBB bounds = GetOBB(*objects[i]);
        // test intersection between bounds and given aabb.
        if (AABBOBB(aabb, bounds)) {
            result.push_back(objects[i]);
        }
    }
    return result;
}

void SplitTree(OctreeNode* node, int depth) {
    if (depth-- <= 0) {
        return;
    }
    // split to 8 bounding boxes.
    if (node->children == 0) {
        node->children = new OctreeNode[8];
        vec3 c = node->bounds.position;
        vec3 e = node->bounds.size * 0.5f;
        node->children[0].bounds = AABB(c + vec3(-e.x, +e.y, -e.z), e);
        node->children[1].bounds = AABB(c + vec3(+e.x, +e.y, -e.z), e);
        node->children[2].bounds = AABB(c + vec3(-e.x, +e.y, +e.z), e);
        node->children[3].bounds = AABB(c + vec3(+e.x, +e.y, +e.z), e);
        node->children[4].bounds = AABB(c + vec3(-e.x, -e.y, -e.z), e);
        node->children[5].bounds = AABB(c + vec3(+e.x, -e.y, -e.z), e);
        node->children[6].bounds = AABB(c + vec3(-e.x, -e.y, +e.z), e);
        node->children[7].bounds = AABB(c + vec3(+e.x, -e.y, +e.z), e);
    }
    // move models to children
    if (node->children != 0 && node->models.size() > 0) {
        for (int i = 0; i < 8; ++i) {
            for (int j = 0, size = node->models.size(); j < size; ++j) {
                OBB bounds = GetOBB(*node->models[j]);
                if (AABBOBB(node->children[i].bounds, bounds)) {
                    node->children[i].models.push_back(node->models[j]);
                }
            }
        }
        node->models.clear();
    }
    // recursively split children.
    for (int i = 0; i < 8; ++i) {
        SplitTree(&(node->children[i]), depth);
    }
}

void Insert(OctreeNode* node, Model* model) {
    OBB bounds = GetOBB(*model);
    // check if model intersect with node bound.
    if (AABBOBB(node->bounds, bounds)) {
        if (node->children == 0) {
            // add to model list if leaf node.
            node->models.push_back(model);
        } else {
            // recurse in children
            for (int i = 0; i < 8; ++i) {
                Insert(&(node->children[i]), model);
            }
        }
    }
}

void Remove(OctreeNode* node, Model* model) {
    // does not check for containment
    if (node->children == 0) {
        // find model to remove
        std::vector<Model*>::iterator it = std::find(node->models.begin(), node->models.end(), model);
        if (it != node->models.end()) {
            node->models.erase(it);
        } else {
            // recurse in children.
            for (int i = 0; i < 8; ++i) {
                Remove(&(node->children[i]), model);
            }
        }
    }
}

void Update(OctreeNode* node, Model* model) {
    // remove and re-insert
    Remove(node, model);
    Insert(node, model);
}

Model* FindClosest(const std::vector<Model*>& set, const Ray& ray) {
    if (set.size() == 0) {
        return 0;
    }
    Model* closest = 0;
    float closest_t = -1;
    // loop models and raycast
    for (int i = 0, size = set.size(); i < size; ++i) {
        float this_t = ModelRay(*set[i], ray);
        if (this_t < 0) {
            continue;
        }
        // get the raycast with smallest t
        if (closest_t < 0 || this_t < closest_t) {
            closest_t = this_t;
            closest = set[i];
        }
    }
    return closest;
}

Model* Raycast(OctreeNode* node, const Ray& ray) {
    // check intersection of ray and node bound
    float t = Raycast(node->bounds, ray);
    if (t >= 0) {
        if (node->children == 0) {
            // find closest among models in the leaf node.
            return FindClosest(node->models, ray);
        } else {
            // raycast against children node
            std::vector<Model*> results;
            for (int i = 0; i < 8; ++i) {
                Model* result = Raycast(&(node->children[i]), ray);
                if (result != 0) {
                    results.push_back(result);
                }
            }
            // find closest among models in children.
            return FindClosest(results, ray);
        }
    }
    return 0;
}

std::vector<Model*> Query(OctreeNode* node, const Sphere& sphere) {
    std::vector<Model*> result;
    // check sphere intersect with node bounds.
    if (SphereAABB(sphere, node->bounds)) {
        if (node->children == 0) {
            // leaf node.
            // loop through each models.
            for (int i = 0, size = node->models.size(); i < size; ++i) {
                OBB bounds = GetOBB(*(node->models[i]));
                // check sphere intersect with model bound.
                if (SphereOBB(sphere, bounds)) {
                    result.push_back(node->models[i]);
                }
            }
        } else {
            // recursively query node children.
            for (int i = 0; i < 8; ++i) {
                std::vector<Model*> child = Query(&(node->children[i]), sphere);
                if (child.size() > 0) {
                    result.insert(result.end(), child.begin(), child.end());
                }
            }
        }
    }
}

std::vector<Model*> Query(OctreeNode* node, const AABB& aabb) {
    std::vector<Model*> result;
    // check if aabb intersect with node bounds.
    if (AABBAABB(aabb, node->bounds)) {
        if (node->children == 0) {
            for (int i = 0, size = node->models.size(); i < size; ++i) {
                OBB bounds = GetOBB(*(node->models[i]));
                // check if aabb intersect with model bound
                if (AABBOBB(aabb, bounds)) {
                    result.push_back(node->models[i]);
                }
            }
        }
    } else {
        // recursively query children node
        for (int i = 0; i < 8; ++i) {
            std::vector<Model*> child = Query(&(node->children[i]), aabb);
            if (child.size() > 0) {
                result.insert(result.end(), child.begin(), child.end());
            }
        }
    }
    return result;
}

bool Scene::Accelerate(const vec3& position, float size) {
    if (octree != 0) {
        return false;
    }
    // compute minimum and maximum point for octree bounds.
    vec3 min(position.x - size, position.y - size, position.z - size);
    vec3 max(position.x + size, position.y + size, position.z + size);
    // construct octree
    octree = new OctreeNode();
    octree->bounds = FromMinMax(min, max);
    octree->children = 0;
    for (int i = 0, size = objects.size(); i < size; ++i) {
        octree->models.push_back(objects[i]);
    }
    // split 5 levels deep
    SplitTree(octree, 5);
    return true;
}