#pragma once
#include "LiteScene/cmesh4.h"
#include "embree4/rtcore.h"

//an embree-based accelerated structure for a mesh to facilitate work with it
class MeshBVH
{
public:
  MeshBVH() = default;
  ~MeshBVH();
  void init(const cmesh4::SimpleMesh &_mesh);

  //distance from point to the closest triangle in mesh, >0 if point is outside, <0 if point is inside
  //sign is correct only with watertight meshes with geometric normals oriented outside
  float get_signed_distance(LiteMath::float3 p);

  cmesh4::SimpleMesh mesh;
private:
  RTCDevice m_device =     nullptr;
  RTCScene  m_scene  =     nullptr;
  RTCGeometry m_geometry = nullptr;
};