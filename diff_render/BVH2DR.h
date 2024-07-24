#pragma once

#include "../BVH/BVH2Common.h"
#include "DR_common.h"

namespace dr
{
  struct BVHDR : public BVHRT
  {
    CRT_HitDR RayQuery_NearestHitWithGrad(float4 posAndNear, float4 dirAndFar);
    void IntersectAllPrimitivesInLeafWithGrad(const float3 ray_pos, const float3 ray_dir,
                                              float tNear, uint32_t instId, uint32_t geomId,
                                              uint32_t a_start, uint32_t a_count,
                                              CRT_HitDR *pHit);
    void IntersectAllTrianglesInLeafWithGrad(const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_HitDR *pHit);

    void LocalSurfaceIntersectionWithGrad(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                          float values[8], uint32_t nodeId, uint32_t primId, float d, float qNear,
                                          float qFar, float2 fNearFar, float3 start_q,
                                          CRT_HitDR *pHit);

    void OctreeBrickIntersectWithGrad(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                      float tNear, uint32_t instId, uint32_t geomId,
                                      uint32_t a_start, uint32_t a_count,
                                      CRT_HitDR *pHit);

    void BVH2TraverseF32WithGrad(const float3 ray_pos, const float3 ray_dir, float tNear,
                                 uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                 CRT_HitDR *pHit);
  };
}