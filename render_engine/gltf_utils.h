#ifndef CHIMERA_GLTF_UTILS_H
#define CHIMERA_GLTF_UTILS_H

#include <LiteMath.h>
#include "geom/cmesh.h"
#include "tiny_gltf.h"
#include "shaders/common.h"

void getNumVerticesAndIndicesFromGLTFMesh(const tinygltf::Model &a_model, const tinygltf::Mesh &a_mesh, uint32_t& numVertices, uint32_t& numIndices);
cmesh::SimpleMesh  simpleMeshFromGLTFMesh(const tinygltf::Model &a_model, const tinygltf::Mesh &a_mesh);
LiteMath::float4x4 transformMatrixFromGLTFNode(const tinygltf::Node &node);
MaterialData_pbrMR materialDataFromGLTF(const tinygltf::Material &gltfMat);

#endif// CHIMERA_GLTF_UTILS_H
