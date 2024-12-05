#pragma once
#include "mesh.h"

namespace cmesh4_demo
{
  //these settings do not affect the data structure (it is still cmesh4::SimpleMesh), 
  //they just determine how the mesh is created
  
  enum class VerticesType
  {
    SHARED, //one vertex can belong to multiple triangles (typical for indexed meshes)
    UNIQUE //one vertex can only belong to one triangle ("triangle soup")
  };
  enum class NormalsType
  {
    VERTEX, //default, normals in each vertex is an average of geomtry normals
    GEOMETRY //only for VerticesType::UNIQUE, all vertices in trianle will have the same normal
  };

  //all meshes are created with center in (0,0,0)
  //scale means different things, but reducing in all directions will always result in a smaller mesh

  cmesh4::SimpleMesh create_cube(LiteMath::float3 scale, bool six_materials, VerticesType a_verticesType = VerticesType::SHARED, NormalsType a_normalsType = NormalsType::VERTEX);
  cmesh4::SimpleMesh create_tetrahedron(LiteMath::float3 scale,VerticesType a_verticesType = VerticesType::SHARED, NormalsType a_normalsType = NormalsType::VERTEX);
  cmesh4::SimpleMesh create_cylinder(LiteMath::float3 scale, unsigned c_segments = 16, unsigned h_segments = 16, VerticesType a_verticesType = VerticesType::SHARED, NormalsType a_normalsType = NormalsType::VERTEX);
}