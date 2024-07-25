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

    float Intersect(const float3 ray_dir, float values[8], float d, 
                    float qNear, float qFar, float3 start_q);

    void dIntersect_dValues(const float3 ray_dir, float values[8], float d,
                            float qNear, float qFar, float3 start_q, float out_dValues[8]);

    void OctreeBrickIntersectWithGrad(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                      float tNear, uint32_t instId, uint32_t geomId,
                                      uint32_t a_start, uint32_t a_count,
                                      CRT_HitDR *pHit);

    void BVH2TraverseF32WithGrad(const float3 ray_pos, const float3 ray_dir, float tNear,
                                 uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                 CRT_HitDR *pHit);




    CRT_HitDR RayQuery_NearestHitWithGradDShape(float4 posAndNear, float4 dirAndFar,
                                                float relax_eps /* new */);

    void BVH2TraverseF32WithGradDShape(const float3 ray_pos, const float3 ray_dir, float tNear,
                                       uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                       float relax_eps, /* new */
                                       CRT_HitDR *pHit);

    void IntersectAllPrimitivesInLeafWithGradDShape(const float3 ray_pos, const float3 ray_dir,
                                                    float tNear, uint32_t instId, uint32_t geomId,
                                                    uint32_t a_start, uint32_t a_count,
                                                    float relax_eps, /* new */
                                                    CRT_HitDR *pHit);

    void OctreeBrickIntersectWithGradDShape(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                            float tNear, uint32_t instId, uint32_t geomId,
                                            uint32_t a_start, uint32_t a_count,
                                            float relax_eps, /* new */
                                            CRT_HitDR *pHit);

    float IntersectDShape(const float3 ray_dir, float values[8], float d, 
                          float qNear, float qFar, float3 start_q, float relax_eps, float2 &t_sdf_pair);

    MultiRendererDRPreset m_preset_dr;
  };
}