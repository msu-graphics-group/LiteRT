#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <chrono>

#include "BVH2Common.h"

float2 BVHRT::box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir)
{
  float3 safe_dir = sign(dir) * max(float3(1e-9f), abs(dir));
  float3 tMin = (min_pos - origin) / safe_dir;
  float3 tMax = (max_pos - origin) / safe_dir;
  float3 t1 = min(tMin, tMax);
  float3 t2 = max(tMin, tMax);
  float tNear = std::max(t1.x, std::max(t1.y, t1.z));
  float tFar = std::min(t2.x, std::min(t2.y, t2.z));

  return float2(tNear, tFar);
}

float BVHRT::eval_dist_prim(unsigned prim_id, float3 p)
{
  SdfObject prim = m_SdfObjects[prim_id];
  float3 pos = to_float3(prim.transform * to_float4(p, 1));

  switch (prim.type)
  {
  case SDF_PRIM_SPHERE:
  {
    float r = m_SdfParameters[prim.params_offset + 0];
    // fprintf(stderr, "sphere %f %f %f - %f",pos.x, pos.y, pos.z, r);
    return length(pos) - r;
  }
  case SDF_PRIM_BOX:
  {
    float3 size(m_SdfParameters[prim.params_offset + 0],
                m_SdfParameters[prim.params_offset + 1],
                m_SdfParameters[prim.params_offset + 2]);
    // fprintf(stderr, "box %f %f %f - %f %f %f - %f %f %f",p.x, p.y, p.z, pos.x, pos.y, pos.z, size.x, size.y, size.z);
    float3 q = abs(pos) - size;
    return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
  }
  case SDF_PRIM_CYLINDER:
  {
    float h = m_SdfParameters[prim.params_offset + 0];
    float r = m_SdfParameters[prim.params_offset + 1];
    float2 d = abs(float2(sqrt(pos.x * pos.x + pos.z * pos.z), pos.y)) - float2(r, h);
    return min(max(d.x, d.y), 0.0f) + length(max(d, float2(0.0f)));
  }
  case SDF_PRIM_SIREN:
  {
    float tmp_mem[2 * NEURAL_SDF_MAX_LAYER_SIZE];

    NeuralProperties prop = m_SdfNeuralProperties[prim.neural_id];
    unsigned t_ofs1 = 0;
    unsigned t_ofs2 = NEURAL_SDF_MAX_LAYER_SIZE;

    tmp_mem[t_ofs1 + 0] = p.x;
    tmp_mem[t_ofs1 + 1] = p.y;
    tmp_mem[t_ofs1 + 2] = p.z;

    for (int l = 0; l < prop.layer_count; l++)
    {
      unsigned m_ofs = prop.layers[l].offset;
      unsigned b_ofs = prop.layers[l].offset + prop.layers[l].in_size * prop.layers[l].out_size;
      for (int i = 0; i < prop.layers[l].out_size; i++)
      {
        tmp_mem[t_ofs2 + i] = m_SdfParameters[b_ofs + i];
        for (int j = 0; j < prop.layers[l].in_size; j++)
          tmp_mem[t_ofs2 + i] += tmp_mem[t_ofs1 + j] * m_SdfParameters[m_ofs + i * prop.layers[l].in_size + j];
        if (l < prop.layer_count - 1)
          tmp_mem[t_ofs2 + i] = std::sin(SIREN_W0 * tmp_mem[t_ofs2 + i]);
      }

      t_ofs2 = t_ofs1;
      t_ofs1 = (t_ofs1 + NEURAL_SDF_MAX_LAYER_SIZE) % (2 * NEURAL_SDF_MAX_LAYER_SIZE);
    }

    return tmp_mem[t_ofs1];
  }
  default:
    //fprintf(stderr, "unknown type %u", prim.type);
    //assert(false);
    break;
  }
  return -1000;
}

float BVHRT::eval_dist_conjunction(unsigned conj_id, float3 p)
{
  SdfConjunction conj = m_SdfConjunctions[conj_id];
  float conj_d = -1e6;
  for (unsigned pid = conj.offset; pid < conj.offset + conj.size; pid++)
  {
    float prim_d = m_SdfObjects[pid].distance_mult * eval_dist_prim(pid, p) +
                   m_SdfObjects[pid].distance_add;
    conj_d = max(conj_d, m_SdfObjects[pid].complement == 1 ? -prim_d : prim_d);
  }
  return conj_d;
}

void BVHRT::IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_Hit *pHit)
{
  unsigned type = m_geomTypeByGeomId[geomId];
  switch (type)
  {
  case TYPE_MESH_TRIANGLE:
    IntersectAllTrianglesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_SDF_PRIMITIVE:
    IntersectAllSdfPrimitivesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_SDF_GRID:
    IntersectAllSdfGridsInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  default:
    break;
  }
}

SdfHit BVHRT::sdf_conjunction_sphere_tracing(unsigned conj_id, const float3 &min_pos, const float3 &max_pos,
                                             const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  hit.hit_pos = float4(0,0,0,-1);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = tNear_tFar.x;
  float tFar = tNear_tFar.y;
  if (t > tFar)
    return hit;
  
  int iter = 0;
  float d = eval_dist_conjunction(conj_id, pos + t * dir);
  while (iter < 1000 && d > EPS && t < tFar)
  {
    t += d + EPS;
    d = eval_dist_conjunction(conj_id, pos + t * dir);
    iter++;
  }

  if (d > EPS)
    return hit;

  float3 p0 = pos + t * dir;
  float3 norm = float3(1,0,0);
  if (need_norm)
  {
    const float h = 0.001;
    float ddx = (eval_dist_conjunction(conj_id, p0 + float3(h, 0, 0)) -
                 eval_dist_conjunction(conj_id, p0 + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (eval_dist_conjunction(conj_id, p0 + float3(0, h, 0)) -
                 eval_dist_conjunction(conj_id, p0 + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (eval_dist_conjunction(conj_id, p0 + float3(0, 0, h)) -
                 eval_dist_conjunction(conj_id, p0 + float3(0, 0, -h))) /
                (2 * h);

    norm = normalize(float3(ddx, ddy, ddz));
    // fprintf(stderr, "st %d (%f %f %f)\n", iter, surface_normal->x, surface_normal->y, surface_normal->z);
  }
  // fprintf(stderr, "st %d (%f %f %f)", iter, p0.x, p0.y, p0.z);
  hit.hit_pos = to_float4(p0, 1);
  hit.hit_norm = to_float4(norm, 1.0f);
  return hit;
}

void BVHRT::IntersectAllSdfPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                                float tNear, uint32_t instId, uint32_t geomId,
                                                uint32_t a_start, uint32_t a_count,
                                                CRT_Hit *pHit)
{
  //assert(a_count == 1);
  unsigned conjId = m_ConjIndices[m_geomOffsets[geomId].x + a_start];
  float l = length(ray_dir);
  float3 dir = ray_dir/l;

  SdfHit hit = sdf_conjunction_sphere_tracing(conjId, 
                                              to_float3(m_SdfConjunctions[conjId].min_pos), 
                                              to_float3(m_SdfConjunctions[conjId].max_pos),
                                              ray_pos, dir, true);
  if (hit.hit_pos.w > 0)
  {
    float t = length(to_float3(hit.hit_pos)-ray_pos)/l;
    if (t > tNear && t < pHit->t)
    {
      pHit->t         = t;
      pHit->primId    = conjId;
      pHit->instId    = instId;
      pHit->geomId    = geomId;  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = hit.hit_norm.x;
      pHit->coords[3] = hit.hit_norm.y;
    }
  }
}

float BVHRT::eval_distance_sdf_grid(unsigned grid_id, float3 pos)
{
  unsigned off = m_SdfGridOffsets[grid_id];
  uint3 size = m_SdfGridSizes[grid_id];

  //bbox for grid is a unit cube
  float3 grid_size_f = float3(size);
  float3 vox_f = grid_size_f*min(max((pos-float3(-1,-1,-1))/float3(2,2,2), float3(0.0f)), float3(1.0f-1e-5f));
  uint3 vox_u = uint3(vox_f);
  float3 dp = vox_f - float3(vox_u);

  //trilinear sampling
  float res = 0.0;
  if (vox_u.x < size.x-1 && vox_u.y < size.y-1 && vox_u.z < size.z-1)
  {
    for (int i=0;i<2;i++)
    {
      for (int j=0;j<2;j++)
      {
        for (int k=0;k<2;k++)
        {
          float qx = (1 - dp.x + i*(2*dp.x-1));
          float qy = (1 - dp.y + j*(2*dp.y-1));
          float qz = (1 - dp.z + k*(2*dp.z-1));   
          res += qx*qy*qz*m_SdfGridData[off + (vox_u.z + k)*size.x*size.y + (vox_u.y + j)*size.x + (vox_u.x + i)];   
        }      
      }
    }
  }
  else
  {
    res += m_SdfGridData[off + (vox_u.z)*size.x*size.y + (vox_u.y)*size.x + (vox_u.x)]; 
  }
  
  return res;
}

SdfHit BVHRT::sdf_grid_sphere_tracing(unsigned grid_id, const float3 &min_pos, const float3 &max_pos,
                                      const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  hit.hit_pos = float4(0,0,0,-1);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = tNear_tFar.x;
  float tFar = tNear_tFar.y;
  if (t > tFar)
    return hit;
  
  int iter = 0;
  float d = eval_distance_sdf_grid(grid_id, pos + t * dir);
  while (iter < 1000 && d > EPS && t < tFar)
  {
    t += d + EPS;
    d = eval_distance_sdf_grid(grid_id, pos + t * dir);
    iter++;
  }

  if (d > EPS)
    return hit;

  float3 p0 = pos + t * dir;
  float3 norm = float3(1,0,0);
  if (need_norm)
  {
    const float h = 0.001;
    float ddx = (eval_distance_sdf_grid(grid_id, p0 + float3(h, 0, 0)) -
                 eval_distance_sdf_grid(grid_id, p0 + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (eval_distance_sdf_grid(grid_id, p0 + float3(0, h, 0)) -
                 eval_distance_sdf_grid(grid_id, p0 + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (eval_distance_sdf_grid(grid_id, p0 + float3(0, 0, h)) -
                 eval_distance_sdf_grid(grid_id, p0 + float3(0, 0, -h))) /
                (2 * h);

    norm = normalize(float3(ddx, ddy, ddz));
    // fprintf(stderr, "st %d (%f %f %f)\n", iter, surface_normal->x, surface_normal->y, surface_normal->z);
  }
  // fprintf(stderr, "st %d (%f %f %f)", iter, p0.x, p0.y, p0.z);
  hit.hit_pos = to_float4(p0, 1);
  hit.hit_norm = to_float4(norm, 1.0f);
  return hit;
}

void BVHRT::IntersectAllSdfGridsInLeaf(const float3 ray_pos, const float3 ray_dir,
                                       float tNear, uint32_t instId, uint32_t geomId,
                                       uint32_t a_start, uint32_t a_count,
                                       CRT_Hit *pHit)
{
  //assert(a_count == 1);
  unsigned gridId = m_geomOffsets[geomId].x;
  float l = length(ray_dir);
  float3 dir = ray_dir/l;

  SdfHit hit = sdf_grid_sphere_tracing(gridId, float3(-1,-1,-1), float3( 1, 1, 1), ray_pos, dir, true);
  if (hit.hit_pos.w > 0)
  {
    float t = length(to_float3(hit.hit_pos)-ray_pos)/l;
    if (t > tNear && t < pHit->t)
    {
      pHit->t         = t;
      pHit->primId    = 0;
      pHit->instId    = instId;
      pHit->geomId    = geomId;  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = hit.hit_norm.x;
      pHit->coords[3] = hit.hit_norm.y;
    }
  }
}

void BVHRT::IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                            float tNear, uint32_t instId, uint32_t geomId,
                                            uint32_t a_start, uint32_t a_count,
                                            CRT_Hit *pHit)
{
  const uint2 a_geomOffsets = m_geomOffsets[geomId];

  for (uint32_t triId = a_start; triId < a_start + a_count; triId++)
  {
    const uint32_t A = m_indices[a_geomOffsets.x + triId*3 + 0];
    const uint32_t B = m_indices[a_geomOffsets.x + triId*3 + 1];
    const uint32_t C = m_indices[a_geomOffsets.x + triId*3 + 2];

    const float3 A_pos = to_float3(m_vertPos[a_geomOffsets.y + A]);
    const float3 B_pos = to_float3(m_vertPos[a_geomOffsets.y + B]);
    const float3 C_pos = to_float3(m_vertPos[a_geomOffsets.y + C]);

    const float3 edge1 = B_pos - A_pos;
    const float3 edge2 = C_pos - A_pos;
    const float3 pvec = cross(ray_dir, edge2);
    const float3 tvec = ray_pos - A_pos;
    const float3 qvec = cross(tvec, edge1);

    const float invDet = 1.0f / dot(edge1, pvec);
    const float v = dot(tvec, pvec) * invDet;
    const float u = dot(qvec, ray_dir) * invDet;
    const float t = dot(edge2, qvec) * invDet;

    if (v >= -1e-6f && u >= -1e-6f && (u + v <= 1.0f + 1e-6f) && t > tNear && t < pHit->t) 
    {
      float3 n = normalize(cross(edge1, edge2));

      pHit->t = t;
      pHit->primId = triId;
      pHit->instId = instId;
      pHit->geomId = geomId;
      pHit->coords[0] = u;
      pHit->coords[1] = v;
      pHit->coords[2] = n.x;
      pHit->coords[3] = n.y;
    }
  }
}

void BVHRT::BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear,
                                uint32_t instId, uint32_t geomId, uint32_t stack[STACK_SIZE], bool stopOnFirstHit,
                                CRT_Hit* pHit)
{
  const uint32_t bvhOffset = m_bvhOffsets[geomId];

  int top = 0;
  uint32_t leftNodeOffset = 0;

  const float3 rayDirInv = SafeInverse(ray_dir);
  while (top >= 0 && !(stopOnFirstHit && pHit->primId != uint32_t(-1)))
  {
    while (top >= 0 && ((leftNodeOffset & LEAF_BIT) == 0))
    {
      const BVHNodePair fatNode = m_allNodePairs[bvhOffset + leftNodeOffset];

      const uint32_t node0_leftOffset = fatNode.left.leftOffset;
      const uint32_t node1_leftOffset = fatNode.right.leftOffset;

      const float2 tm0 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.left.boxMin, fatNode.left.boxMax);
      const float2 tm1 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.right.boxMin, fatNode.right.boxMax);

      const bool hitChild0 = (tm0.x <= tm0.y) && (tm0.y >= tNear) && (tm0.x <= pHit->t);
      const bool hitChild1 = (tm1.x <= tm1.y) && (tm1.y >= tNear) && (tm1.x <= pHit->t);

      // traversal decision
      leftNodeOffset = hitChild0 ? node0_leftOffset : node1_leftOffset;

      if (hitChild0 && hitChild1)
      {
        leftNodeOffset = (tm0.x <= tm1.x) ? node0_leftOffset : node1_leftOffset; // GPU style branch
        stack[top]     = (tm0.x <= tm1.x) ? node1_leftOffset : node0_leftOffset; // GPU style branch
        top++;
      }

      if (!hitChild0 && !hitChild1) // both miss, stack.pop()
      {
        top--;
        leftNodeOffset = stack[std::max(top,0)];
      }

    } // end while (searchingForLeaf)

    // leaf node, intersect triangles
    //
    if (top >= 0 && leftNodeOffset != 0xFFFFFFFF)
    {
      const uint32_t start = EXTRACT_START(leftNodeOffset);
      const uint32_t count = EXTRACT_COUNT(leftNodeOffset);
      IntersectAllPrimitivesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, start, count, pHit);
    }

    // continue BVH traversal
    //
    top--;
    leftNodeOffset = stack[std::max(top,0)];

  } // end while (top >= 0)

}

CRT_Hit BVHRT::RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar)
{
  bool stopOnFirstHit = (dirAndFar.w <= 0.0f);
  if(stopOnFirstHit)
    dirAndFar.w *= -1.0f;

  uint32_t stack[STACK_SIZE];

  CRT_Hit hit;
  hit.t      = dirAndFar.w;
  hit.primId = uint32_t(-1);
  hit.instId = uint32_t(-1);
  hit.geomId = uint32_t(-1);

  const float3 rayDirInv = SafeInverse(to_float3(dirAndFar));
  uint32_t nodeIdx = 0;
  do
  {
    uint32_t travFlags  = 0;
    uint32_t leftOffset = 0;
    do
    {
      const BVHNode currNode = m_nodesTLAS[nodeIdx];
      const float2 boxHit    = RayBoxIntersection2(to_float3(posAndNear), rayDirInv, currNode.boxMin, currNode.boxMax);
      const bool intersects  = (boxHit.x <= boxHit.y) && (boxHit.y > posAndNear.w) && (boxHit.x < hit.t); // (tmin <= tmax) && (tmax > 0.f) && (tmin < curr_t)

      travFlags  = (currNode.leftOffset & LEAF_BIT) | uint32_t(intersects); // travFlags  = (((currNode.leftOffset & LEAF_BIT) == 0) ? 0 : LEAF_BIT) | (intersects ? 1 : 0);
      leftOffset = currNode.leftOffset;
      nodeIdx    = isLeafOrNotIntersect(travFlags) ? currNode.escapeIndex : leftOffset;

    } while (notLeafAndIntersect(travFlags) && nodeIdx != 0 && nodeIdx < 0xFFFFFFFE); 
     
    if(isLeafAndIntersect(travFlags)) 
    {
      const uint32_t instId = EXTRACT_START(leftOffset);
      const uint32_t geomId = m_geomIdByInstId[instId];
  
      // transform ray with matrix to local space
      //
      const float3 ray_pos = matmul4x3(m_instMatricesInv[instId], to_float3(posAndNear));
      const float3 ray_dir = matmul3x3(m_instMatricesInv[instId], to_float3(dirAndFar)); // DON'T NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!
  
      BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stack, stopOnFirstHit, &hit);
    }

  } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //

  if(hit.geomId < uint32_t(-1)) 
  {
    const uint2 geomOffsets = m_geomOffsets[hit.geomId];
    hit.primId = m_primIndices[geomOffsets.x/3 + hit.primId];
  }
  
  return hit;
}

bool BVHRT::RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar)
{
  dirAndFar.w *= -1.0f;
  CRT_Hit hit = RayQuery_NearestHit(posAndNear, dirAndFar);
  return (hit.geomId != uint32_t(-1));
}
