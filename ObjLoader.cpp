#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "ObjLoader.h"

bool LoadMesh(const char* szFilePath, Mesh* mesh) {
    if (szFilePath == 0 || mesh == 0) {
        return false;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, szFilePath)) {
        return false;
    }

    mesh->numTriangles = 0;
    mesh->triangles = 0;

    // loop shapes
    for (int s = 0; s < shapes.size(); s++) {
        int index_offset = 0;
        // loop faces
        for (int f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            // check triangle
            int fv = shapes[s].mesh.num_face_vertices[f];
            if (fv != 3) {
                return false;
            }
            mesh->numTriangles += 1;
        }
    }

    mesh->triangles = new Triangle[mesh->numTriangles];
    int vi = 0;

    for (int s = 0; s < shapes.size(); s++) {
        int index_offset = 0;
        for (int f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // loop vertices
            for (int v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                float vx = attrib.vertices[3 * idx.vertex_index + 0];
                float vy = attrib.vertices[3 * idx.vertex_index + 1];
                float vz = attrib.vertices[3 * idx.vertex_index + 2];

                mesh->vertices[vi++] = vec3(vx, vy, vz);
            }
            index_offset += fv;

            shapes[s].mesh.material_ids[f];
        }
    }

    return true;
}

void FreeMesh(Mesh* mesh) {
    if (mesh->triangles != 0) {
        delete[] mesh->triangles;
    }

    mesh->triangles = 0;
    mesh->numTriangles = 0;
}