#pragma once

#include "../BVH/BVH2Common.h"
#include "DR_common.h"

namespace dr
{
  struct BVHDR : public BVHRT
  {
    CRT_HitDR RayQuery_NearestHitWithGrad(uint32_t ray_flags, float4 posAndNear, float4 dirAndFar,
                                          RayDiffPayload *relax_pt);
    void IntersectAllPrimitivesInLeafWithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                              float tNear, uint32_t instId, uint32_t geomId,
                                              uint32_t a_start, uint32_t a_count,
                                              RayDiffPayload *relax_pt, CRT_HitDR *pHit);
    void IntersectAllTrianglesInLeafWithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_HitDR *pHit);

    float Intersect(uint32_t ray_flags, const float3 ray_dir, float values[8], float d, 
                    float qNear, float qFar, float3 start_q,
                    RayDiffPayload *relax_pt);

    void dIntersect_dValues(uint32_t ray_flags, const float3 ray_dir, float values[8], float d,
                            float qNear, float qFar, float3 start_q, float out_dValues[8]);

    void OctreeBrickIntersectWithGrad(uint32_t type, uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                      float tNear, uint32_t instId, uint32_t geomId,
                                      uint32_t a_start, uint32_t a_count,
                                      RayDiffPayload *relax_pt,
                                      CRT_HitDR *pHit);

    void BVH2TraverseF32WithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir, float tNear,
                                 uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                 RayDiffPayload *relax_pt,
                                 CRT_HitDR *pHit);
  
    float load_distance_values_with_indices(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, 
                                            /*out*/ float values[8], /*out*/ uint32_t indices[8]);

  };
}