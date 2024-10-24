#include "BVH2DR.h"

using LiteMath::M_PI;
using LiteMath::clamp;

#define VERIFY_DERIVATIVES 0

namespace dr
{
  //it is an EXACT COPY of BVHRT::RayQuery_NearestHit, just using CRT_HitDR instead of CRT_Hit
  //but no change it TLAS traversal is needed
  CRT_HitDR BVHDR::RayQuery_NearestHitWithGrad(uint32_t ray_flags, float4 posAndNear, float4 dirAndFar, RayDiffPayload *relax_pt)
  {
    bool stopOnFirstHit = (dirAndFar.w <= 0.0f);
    if(stopOnFirstHit)
      dirAndFar.w *= -1.0f;
    const float3 rayDirInv = SafeInverse(to_float3(dirAndFar));

    CRT_HitDR hit;
    hit.t      = dirAndFar.w;
    hit.primId = uint32_t(-1);
    hit.instId = uint32_t(-1);
    hit.geomId = uint32_t(-1);
    hit.color  = float3(0.0f, 0.0, 0.0f);
    hit.sdf    = 1000.0f;
    hit.normal = float3(1.0f, 0.0f, 0.0f);

    relax_pt->missed_hit.t      = dirAndFar.w;
    relax_pt->missed_hit.primId = uint32_t(-1);
    relax_pt->missed_hit.instId = uint32_t(-1);
    relax_pt->missed_hit.geomId = uint32_t(-1);
    relax_pt->missed_hit.color  = float3(0.0f, 0.0, 0.0f);
    relax_pt->missed_hit.sdf    = 1000.0f;
    relax_pt->missed_hit.normal = float3(1.0f, 0.0f, 0.0f);
    relax_pt->missed_hit._pad1  = 1;

    for (int i=0;i<8;i++)
    {
      relax_pt->missed_indices[i] = INVALID_INDEX;
      relax_pt->missed_dSDF_dtheta[i] = 0.0f;
    }

    for (int i=0;i<MAX_PD_COUNT_COLOR;i++)
    {
      relax_pt->dDiffuse_dSc[i].index = INVALID_INDEX;
      relax_pt->dDiffuse_dSc[i].value = 0.0f;
    }
    
    for (int i=0;i<MAX_PD_COUNT_DIST;i++)
    {
      relax_pt->dDiffuseNormal_dSd[i].index = INVALID_INDEX;
      relax_pt->dDiffuseNormal_dSd[i].dDiffuse = float3(0.0f, 0.0f, 0.0f);
      relax_pt->dDiffuseNormal_dSd[i].dNorm    = float3(0.0f, 0.0f, 0.0f);
      relax_pt->dDiffuseNormal_dSd[i].dDist    = 0.0f;
    }

    {
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
          const uint32_t geomId = m_instanceData[instId].geomId;
      
          // transform ray with matrix to local space
          //
          const float3 ray_pos = matmul4x3(m_instanceData[instId].transformInv, to_float3(posAndNear));
          const float3 ray_dir = matmul3x3(m_instanceData[instId].transformInv, to_float3(dirAndFar)); // DON'float NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!
      
          BVH2TraverseF32WithGrad(ray_flags, ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, relax_pt, &hit);
        }
      } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //
    }

    if(hit.geomId < uint32_t(-1) && ((hit.geomId >> SH_TYPE) == TYPE_MESH_TRIANGLE)) 
    {
      const uint2 geomOffsets = m_geomData[hit.geomId & 0x0FFFFFFF].offset;
      hit.primId = m_primIndices[geomOffsets.x/3 + hit.primId];
    }
    
    if (relax_pt && relax_pt->missed_hit_candidate.sdf < relax_pt->missed_hit.sdf)
      relax_pt->missed_hit = relax_pt->missed_hit_candidate;

    return hit;
  }

  //it is an EXACT COPY of BVHRT::BVH2TraverseF32, just using CRT_HitDR instead of CRT_Hit
  //but no change it BLAS traversal is needed
  void BVHDR::BVH2TraverseF32WithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir, float tNear,
                                      uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                      RayDiffPayload *relax_pt,
                                      CRT_HitDR *pHit)
  {
    const uint32_t bvhOffset = m_geomData[geomId].bvhOffset;

    uint32_t stack[STACK_SIZE];
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
        IntersectAllPrimitivesInLeafWithGrad(ray_flags, ray_pos, ray_dir, tNear, instId, geomId, start, count, relax_pt, pHit);
      }

      // continue BVH traversal
      //
      top--;
      leftNodeOffset = stack[std::max(top,0)];

    } // end while (top >= 0)

  }

  //It is generally a copy of BVHRT::IntersectAllPrimitivesInLeaf, but with less types supported
  //After implementing HW RT acceleration in BVHRT, this can change
  void BVHDR::IntersectAllPrimitivesInLeafWithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                                   float tNear, uint32_t instId, uint32_t geomId,
                                                   uint32_t a_start, uint32_t a_count,
                                                   RayDiffPayload *relax_pt, CRT_HitDR *pHit)
  {
    uint32_t type = m_geomData[geomId].type;
    const float SDF_BIAS = 0.1f;
    const float tNearSdf = std::max(tNear, SDF_BIAS);
    switch (type)
    {
    case TYPE_MESH_TRIANGLE:
      IntersectAllTrianglesInLeafWithGrad(ray_flags, ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
      break;
    case TYPE_SDF_SBS_COL:
      OctreeBrickIntersectWithGrad(type, ray_flags, ray_pos, ray_dir, tNearSdf, instId, geomId, a_start, a_count, relax_pt, pHit);
      break;
    default:
      break;
    }
  }

  void BVHDR::IntersectAllTrianglesInLeafWithGrad(uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                                  float tNear, uint32_t instId, uint32_t geomId,
                                                  uint32_t a_start, uint32_t a_count,
                                                  CRT_HitDR *pHit)
  {
    //TODO if we want diff rendering of triangles 
  }

  static float3 eval_dist_trilinear_diff(const float values[8], float3 dp)
  {
    float ddist_dx = -(1-dp.y)*(1-dp.z)*values[0] + 
                     -(1-dp.y)*(  dp.z)*values[1] + 
                     -(  dp.y)*(1-dp.z)*values[2] + 
                     -(  dp.y)*(  dp.z)*values[3] + 
                      (1-dp.y)*(1-dp.z)*values[4] + 
                      (1-dp.y)*(  dp.z)*values[5] + 
                      (  dp.y)*(1-dp.z)*values[6] + 
                      (  dp.y)*(  dp.z)*values[7];
    
    float ddist_dy = -(1-dp.x)*(1-dp.z)*values[0] + 
                     -(1-dp.x)*(  dp.z)*values[1] + 
                      (1-dp.x)*(1-dp.z)*values[2] + 
                      (1-dp.x)*(  dp.z)*values[3] + 
                     -(  dp.x)*(1-dp.z)*values[4] + 
                     -(  dp.x)*(  dp.z)*values[5] + 
                      (  dp.x)*(1-dp.z)*values[6] + 
                      (  dp.x)*(  dp.z)*values[7];

    float ddist_dz = -(1-dp.x)*(1-dp.y)*values[0] + 
                      (1-dp.x)*(1-dp.y)*values[1] + 
                     -(1-dp.x)*(  dp.y)*values[2] + 
                      (1-dp.x)*(  dp.y)*values[3] + 
                     -(  dp.x)*(1-dp.y)*values[4] + 
                      (  dp.x)*(1-dp.y)*values[5] + 
                     -(  dp.x)*(  dp.y)*values[6] + 
                      (  dp.x)*(  dp.y)*values[7];
  
    return float3(ddist_dx, ddist_dy, ddist_dz);
  }

  static void eval_dist_trilinear_d_dtheta(float *dsdf_dtheta, float3 dp)
  {
    dsdf_dtheta[0] = (1-dp.x)*(1-dp.y)*(1-dp.z);
    dsdf_dtheta[1] = (1-dp.x)*(1-dp.y)*(  dp.z);
    dsdf_dtheta[2] = (1-dp.x)*(  dp.y)*(1-dp.z);
    dsdf_dtheta[3] = (1-dp.x)*(  dp.y)*(  dp.z);
    dsdf_dtheta[4] = (  dp.x)*(1-dp.y)*(1-dp.z);
    dsdf_dtheta[5] = (  dp.x)*(1-dp.y)*(  dp.z);
    dsdf_dtheta[6] = (  dp.x)*(  dp.y)*(1-dp.z);
    dsdf_dtheta[7] = (  dp.x)*(  dp.y)*(  dp.z);
  }

  static float3 eval_color_trilinear(const float3 values[8], float3 dp)
  {
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
           (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
           (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
           (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
           (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
           (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
           (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
           (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  }

  static float3x3 eval_color_trilinear_ddp(const float3 values[8], float3 dp)
  {
    float3 dcolor_dx = -(1-dp.y)*(1-dp.z)*values[0] + 
                       -(1-dp.y)*(  dp.z)*values[1] + 
                       -(  dp.y)*(1-dp.z)*values[2] + 
                       -(  dp.y)*(  dp.z)*values[3] + 
                        (1-dp.y)*(1-dp.z)*values[4] + 
                        (1-dp.y)*(  dp.z)*values[5] + 
                        (  dp.y)*(1-dp.z)*values[6] + 
                        (  dp.y)*(  dp.z)*values[7];
    
    float3 dcolor_dy = -(1-dp.x)*(1-dp.z)*values[0] + 
                       -(1-dp.x)*(  dp.z)*values[1] + 
                        (1-dp.x)*(1-dp.z)*values[2] + 
                        (1-dp.x)*(  dp.z)*values[3] + 
                       -(  dp.x)*(1-dp.z)*values[4] + 
                       -(  dp.x)*(  dp.z)*values[5] + 
                        (  dp.x)*(1-dp.z)*values[6] + 
                        (  dp.x)*(  dp.z)*values[7];

    float3 dcolor_dz = -(1-dp.x)*(1-dp.y)*values[0] + 
                        (1-dp.x)*(1-dp.y)*values[1] + 
                       -(1-dp.x)*(  dp.y)*values[2] + 
                        (1-dp.x)*(  dp.y)*values[3] + 
                       -(  dp.x)*(1-dp.y)*values[4] + 
                        (  dp.x)*(1-dp.y)*values[5] + 
                       -(  dp.x)*(  dp.y)*values[6] + 
                        (  dp.x)*(  dp.y)*values[7];

#if VERIFY_DERIVATIVES
    {
    static unsigned count = 0;
    static unsigned e_count = 0;
    static long double sliding_average = 0;
    float3x3 deriv_1 = LiteMath::make_float3x3_by_columns(dcolor_dx, dcolor_dy, dcolor_dz);
    float t = 0.001f;
    float3x3 deriv_2 = LiteMath::make_float3x3_by_columns(
                       (eval_color_trilinear(values, dp + t*float3(1,0,0)) - eval_color_trilinear(values, dp - t*float3(1,0,0)))/(2*t),
                       (eval_color_trilinear(values, dp + t*float3(0,1,0)) - eval_color_trilinear(values, dp - t*float3(0,1,0)))/(2*t),
                       (eval_color_trilinear(values, dp + t*float3(0,0,1)) - eval_color_trilinear(values, dp - t*float3(0,0,1)))/(2*t));

    float l1 = 0;
    float l2 = 0;
    float ldiff = 0;

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        l1 += deriv_1.row[i][j]*deriv_1.row[i][j];
        l2 += deriv_2.row[i][j]*deriv_2.row[i][j];
        ldiff += (deriv_1.row[i][j]-deriv_2.row[i][j])*(deriv_1.row[i][j]-deriv_2.row[i][j]);
      }
    }

    l1 = sqrt(l1);
    l2 = sqrt(l2);
    ldiff = sqrt(ldiff);
    float l = 0.5f*(l1 + l2);

    sliding_average = (count == 0) ? l : 0.999f*sliding_average + 0.001f*l;

    count++;
    if (ldiff > 0.01f*l && ldiff > 0.01f*sliding_average)
    {
      e_count++;
      printf("diff error: %f\n", ldiff);
      printf("deriv_1:\n");
      printf("%f %f %f\n", deriv_1.row[0][0], deriv_1.row[0][1], deriv_1.row[0][2]);
      printf("%f %f %f\n", deriv_1.row[1][0], deriv_1.row[1][1], deriv_1.row[1][2]);
      printf("%f %f %f\n", deriv_1.row[2][0], deriv_1.row[2][1], deriv_1.row[2][2]);
      printf("deriv_2:\n");
      printf("%f %f %f\n", deriv_2.row[0][0], deriv_2.row[0][1], deriv_2.row[0][2]);
      printf("%f %f %f\n", deriv_2.row[1][0], deriv_2.row[1][1], deriv_2.row[1][2]);
      printf("%f %f %f\n", deriv_2.row[2][0], deriv_2.row[2][1], deriv_2.row[2][2]);
    }
    if (count % 1000000 == 0)
      printf("eval_color_trilinear_ddp: %u errors out of %uM tries\n", e_count, count/1000000u);
    }
#endif
    return LiteMath::make_float3x3_by_columns(dcolor_dx, dcolor_dy, dcolor_dz);
  }

  static float3x3 normalize_diff(float3 v)
  {
    float3 n = v/sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    float a = dot(v, v);
    float b = 1.0/(a*sqrt(a));

    float3 dx_dv = b*float3(v.y*v.y + v.z*v.z, -v.x*v.y, -v.x*v.z);
    float3 dy_dv = b*float3(-v.x*v.y, v.x*v.x + v.z*v.z, -v.y*v.z);
    float3 dz_dv = b*float3(-v.x*v.z, -v.y*v.z, v.x*v.x + v.y*v.y);

#if VERIFY_DERIVATIVES
    {
    static unsigned count = 0;
    static unsigned e_count = 0;
    static long double sliding_average = 0;
    float3x3 deriv_1 = LiteMath::make_float3x3(dx_dv, dy_dv, dz_dv);
    float t = 0.001f;
    float3x3 deriv_2 = LiteMath::make_float3x3_by_columns(
                       (normalize(v + t*float3(1,0,0)) - normalize(v - t*float3(1,0,0)))/(2*t),
                       (normalize(v + t*float3(0,1,0)) - normalize(v - t*float3(0,1,0)))/(2*t),
                       (normalize(v + t*float3(0,0,1)) - normalize(v - t*float3(0,0,1)))/(2*t));

    float l1 = 0;
    float l2 = 0;
    float ldiff = 0;

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        l1 += deriv_1.row[i][j]*deriv_1.row[i][j];
        l2 += deriv_2.row[i][j]*deriv_2.row[i][j];
        ldiff += (deriv_1.row[i][j]-deriv_2.row[i][j])*(deriv_1.row[i][j]-deriv_2.row[i][j]);
      }
    }

    l1 = sqrt(l1);
    l2 = sqrt(l2);
    ldiff = sqrt(ldiff);
    float l = 0.5f*(l1 + l2);

    sliding_average = (count == 0) ? l : 0.999f*sliding_average + 0.001f*l;

    count++;
    if (ldiff > 0.025f*l && ldiff > 0.025f*sliding_average)
    {
      e_count++;
      /*
      printf("normalize_diff error: %f %f\n", ldiff/l, (float)(ldiff/sliding_average));
      printf("deriv_1:\n");
      printf("%f %f %f\n", deriv_1.row[0][0], deriv_1.row[0][1], deriv_1.row[0][2]);
      printf("%f %f %f\n", deriv_1.row[1][0], deriv_1.row[1][1], deriv_1.row[1][2]);
      printf("%f %f %f\n", deriv_1.row[2][0], deriv_1.row[2][1], deriv_1.row[2][2]);
      printf("deriv_2:\n");
      printf("%f %f %f\n", deriv_2.row[0][0], deriv_2.row[0][1], deriv_2.row[0][2]);
      printf("%f %f %f\n", deriv_2.row[1][0], deriv_2.row[1][1], deriv_2.row[1][2]);
      printf("%f %f %f\n", deriv_2.row[2][0], deriv_2.row[2][1], deriv_2.row[2][2]);
      */
    }
    if (count % 1000000 == 0)
      printf("normalize_diff: %u errors out of %uM tries\n", e_count, count/1000000u);
    }
#endif

    return LiteMath::make_float3x3(dx_dv, dy_dv, dz_dv);
  }

  static float3x3 eval_dist_trilinear_ddp_ddp(const float values[8], float3 dp)
  {
    float ddist_dxdx = 0;

    float ddist_dxdy =  (1-dp.z)*values[0] + 
                        (  dp.z)*values[1] + 
                       -(1-dp.z)*values[2] + 
                       -(  dp.z)*values[3] + 
                       -(1-dp.z)*values[4] + 
                       -(  dp.z)*values[5] + 
                        (1-dp.z)*values[6] + 
                        (  dp.z)*values[7];

    float ddist_dxdz = (1-dp.y)*values[0] + 
                      -(1-dp.y)*values[1] + 
                       (  dp.y)*values[2] + 
                      -(  dp.y)*values[3] + 
                      -(1-dp.y)*values[4] + 
                       (1-dp.y)*values[5] + 
                      -(  dp.y)*values[6] + 
                       (  dp.y)*values[7];



    float ddist_dydx = (1-dp.z)*values[0] + 
                       (  dp.z)*values[1] + 
                      -(1-dp.z)*values[2] + 
                      -(  dp.z)*values[3] + 
                      -(1-dp.z)*values[4] + 
                      -(  dp.z)*values[5] + 
                       (1-dp.z)*values[6] + 
                       (  dp.z)*values[7];
    
    float ddist_dydy = 0;

    float ddist_dydz = (1-dp.x)*values[0] + 
                      -(1-dp.x)*values[1] + 
                      -(1-dp.x)*values[2] + 
                       (1-dp.x)*values[3] + 
                       (  dp.x)*values[4] + 
                      -(  dp.x)*values[5] + 
                      -(  dp.x)*values[6] + 
                       (  dp.x)*values[7];
    
  

    float ddist_dzdx = (1-dp.y)*values[0] + 
                      -(1-dp.y)*values[1] + 
                       (  dp.y)*values[2] + 
                      -(  dp.y)*values[3] + 
                      -(1-dp.y)*values[4] + 
                       (1-dp.y)*values[5] + 
                      -(  dp.y)*values[6] + 
                       (  dp.y)*values[7];
    
    float ddist_dzdy = (1-dp.x)*values[0] + 
                      -(1-dp.x)*values[1] + 
                      -(1-dp.x)*values[2] + 
                       (1-dp.x)*values[3] + 
                       (  dp.x)*values[4] + 
                      -(  dp.x)*values[5] + 
                      -(  dp.x)*values[6] + 
                       (  dp.x)*values[7];
  
    float ddist_dzdz = 0;

#if VERIFY_DERIVATIVES
    {
    static unsigned count = 0;
    static unsigned e_count = 0;
    static long double sliding_average = 0;
    float3x3 deriv_1 = LiteMath::make_float3x3(float3(ddist_dxdx, ddist_dxdy, ddist_dxdz),
                                               float3(ddist_dydx, ddist_dydy, ddist_dydz),
                                               float3(ddist_dzdx, ddist_dzdy, ddist_dzdz));
    float t = 0.001f;
    float3x3 deriv_2 = LiteMath::make_float3x3_by_columns(
                       (eval_dist_trilinear_diff(values, dp + t*float3(1,0,0)) - eval_dist_trilinear_diff(values, dp - t*float3(1,0,0)))/(2*t),
                       (eval_dist_trilinear_diff(values, dp + t*float3(0,1,0)) - eval_dist_trilinear_diff(values, dp - t*float3(0,1,0)))/(2*t),
                       (eval_dist_trilinear_diff(values, dp + t*float3(0,0,1)) - eval_dist_trilinear_diff(values, dp - t*float3(0,0,1)))/(2*t));

    float l1 = 0;
    float l2 = 0;
    float ldiff = 0;

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        l1 += deriv_1.row[i][j]*deriv_1.row[i][j];
        l2 += deriv_2.row[i][j]*deriv_2.row[i][j];
        ldiff += (deriv_1.row[i][j]-deriv_2.row[i][j])*(deriv_1.row[i][j]-deriv_2.row[i][j]);
      }
    }

    l1 = sqrt(l1);
    l2 = sqrt(l2);
    ldiff = sqrt(ldiff);
    float l = 0.5f*(l1 + l2);

    sliding_average = (count == 0) ? l : 0.999f*sliding_average + 0.001f*l;

    count++;
    if (ldiff > 0.01f*l && ldiff > 0.01f*sliding_average)
    {
      e_count++;
      printf("diff error: %f\n", ldiff);
      printf("deriv_1:\n");
      printf("%f %f %f\n", deriv_1.row[0][0], deriv_1.row[0][1], deriv_1.row[0][2]);
      printf("%f %f %f\n", deriv_1.row[1][0], deriv_1.row[1][1], deriv_1.row[1][2]);
      printf("%f %f %f\n", deriv_1.row[2][0], deriv_1.row[2][1], deriv_1.row[2][2]);
      printf("deriv_2:\n");
      printf("%f %f %f\n", deriv_2.row[0][0], deriv_2.row[0][1], deriv_2.row[0][2]);
      printf("%f %f %f\n", deriv_2.row[1][0], deriv_2.row[1][1], deriv_2.row[1][2]);
      printf("%f %f %f\n", deriv_2.row[2][0], deriv_2.row[2][1], deriv_2.row[2][2]);
    }
    if (count % 1000000 == 0)
      printf("eval_dist_trilinear_ddp_ddp: %u errors out of %uM tries\n", e_count, count/1000000u);
    }
#endif

    return LiteMath::make_float3x3(float3(ddist_dxdx, ddist_dxdy, ddist_dxdz),
                                   float3(ddist_dydx, ddist_dydy, ddist_dydz),
                                   float3(ddist_dzdx, ddist_dzdy, ddist_dzdz));
  }

  static void eval_dist_trilinear_ddp_dvalues(const float values[8], float3 dp, float *out_mat /*3x8*/)
  {
    //ddist_dx_dvalues
    out_mat[0]  = -(1-dp.y)*(1-dp.z);
    out_mat[1]  = -(1-dp.y)*(  dp.z);
    out_mat[2]  = -(  dp.y)*(1-dp.z);
    out_mat[3]  = -(  dp.y)*(  dp.z);
    out_mat[4]  =  (1-dp.y)*(1-dp.z);
    out_mat[5]  =  (1-dp.y)*(  dp.z);
    out_mat[6]  =  (  dp.y)*(1-dp.z);
    out_mat[7]  =  (  dp.y)*(  dp.z);

    //ddist_dy_dvalues
    out_mat[8]  = -(1-dp.x)*(1-dp.z);
    out_mat[9]  = -(1-dp.x)*(  dp.z);
    out_mat[10] =  (1-dp.x)*(1-dp.z);
    out_mat[11] =  (1-dp.x)*(  dp.z);
    out_mat[12] = -(  dp.x)*(1-dp.z);
    out_mat[13] = -(  dp.x)*(  dp.z);
    out_mat[14] =  (  dp.x)*(1-dp.z);
    out_mat[15] =  (  dp.x)*(  dp.z);

    //ddist_dz_dvalues
    out_mat[16] = -(1-dp.x)*(1-dp.y);
    out_mat[17] =  (1-dp.x)*(1-dp.y);
    out_mat[18] = -(1-dp.x)*(  dp.y);
    out_mat[19] =  (1-dp.x)*(  dp.y);
    out_mat[20] = -(  dp.x)*(1-dp.y);
    out_mat[21] =  (  dp.x)*(1-dp.y);
    out_mat[22] = -(  dp.x)*(  dp.y);
    out_mat[23] =  (  dp.x)*(  dp.y);

#if VERIFY_DERIVATIVES
    {
    static unsigned count = 0;
    static unsigned e_count = 0;
    static long double sliding_average = 0;
    float t = 0.001f;
    float out_mat_ref[3*8]; /*3x8*/
    for (int i = 0; i < 8; i++)
    {
      ((float*)values)[i] += t;
      float3 t1 = eval_dist_trilinear_diff(values, dp);
      ((float*)values)[i] -= 2*t;
      float3 t2 = eval_dist_trilinear_diff(values, dp);
      ((float*)values)[i] += t;
      out_mat_ref[0*8 + i] = (t1.x - t2.x)/(2*t);
      out_mat_ref[1*8 + i] = (t1.y - t2.y)/(2*t);
      out_mat_ref[2*8 + i] = (t1.z - t2.z)/(2*t);
    }

    float l1 = 0;
    float l2 = 0;
    float ldiff = 0;

    for (int i = 0; i < 3*8; i++)
    {
      l1 += out_mat[i]*out_mat[i];
      l2 += out_mat_ref[i]*out_mat_ref[i];
      ldiff += (out_mat[i] - out_mat_ref[i])*(out_mat[i] - out_mat_ref[i]);
    }

    l1 = sqrt(l1);
    l2 = sqrt(l2);
    ldiff = sqrt(ldiff);
    float l = 0.5f*(l1 + l2);

    sliding_average = (count == 0) ? l : 0.999f*sliding_average + 0.001f*l;

    count++;
    if (ldiff > 0.01f*l && ldiff > 0.01f*sliding_average)
    {
      e_count++;
      printf("diff error: %f\n", ldiff);
    }
    if (count % 1000000 == 0)
      printf("eval_dist_trilinear_ddp_dvalues: %u errors out of %uM tries\n", e_count, count/1000000u);
    }
#endif

  }

  //transition from 0 (x=edge0) to 1 (x=edge1)
  static float step(float edge0, float edge1, float x)
  {
    float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t;
  }

  //Currently it is and exact copy of BVHRT::LocalSurfaceIntersection, but it will change later
  //because differential rendering of SDF requires collection additional data during intersection search

  float BVHDR::Intersect(uint32_t ray_flags, const float3 ray_dir, float values[8], float d, 
                         float qNear, float qFar, float3 start_q, RayDiffPayload *relax_pt)
  {
    const float EPS = 1e-6f, SDF_EPS = 1e-7f;
    float d_inv = 1.0f / d;
    float t = qNear;
    bool hit = false;
    unsigned iter = 0;

    if(relax_pt)
    {
      relax_pt->missed_hit_candidate.sdf = 1000.f;
    }

    float start_dist = eval_dist_trilinear(values, start_q + t * ray_dir);
    if (start_dist <= SDF_EPS || m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_BBOX)
    {
      hit = true;
    }
    else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ST)
    {
      const unsigned ST_max_iters = 256;
      float dist = start_dist;
      float3 pp0 = start_q + t * ray_dir;

      // Borders for bisection and SDF min point
      float2 t_dsdf_l { (qFar + 2), 0.f },
             t_dsdf_r { (qFar + 1), 0.f },
             t_sdf_res{ (qFar + 1), relax_pt ? relax_pt->missed_hit.sdf : 0.0f };

      // SDF values in t0, tFar
      const float t0 = 0.f;
      float2 sdf_dsdf_near{ eval_dist_trilinear(values, start_q + t0 * ray_dir),
                            dot(ray_dir, eval_dist_trilinear_diff(values, start_q + t0 * ray_dir)) };
      float2 sdf_dsdf_far{ eval_dist_trilinear(values, start_q + qFar * ray_dir),
                            dot(ray_dir, eval_dist_trilinear_diff(values, start_q + qFar * ray_dir)) };

      // Checking if nearest ray point in voxel has (SDF' <= 0), set left border if true
      if (sdf_dsdf_near.y <= 0.f)
        t_dsdf_l = { 0.f, sdf_dsdf_near.y };

      while (t < qFar && dist > SDF_EPS && iter < ST_max_iters)
      {
        t += dist * d_inv;
        dist = eval_dist_trilinear(values, start_q + t * ray_dir);
        float3 pp = start_q + t * ray_dir;
        iter++;

#ifdef DEBUG_PAYLOAD_STORE_SDF
          // Write minimal SDF value along the ray (found by ST)

          // if (relax_pt->sdf_i.empty())
          //   relax_pt->sdf_i.push_back(dist);
          // relax_pt->sdf_i[0] = std::min(relax_pt->sdf_i[0], dist);
#endif

        if (relax_pt && ray_flags & DR_RAY_FLAG_BORDER)
        {
          // Looking for SDF min

          float2 t_dsdf_curr{ t, dot(ray_dir, eval_dist_trilinear_diff(values, start_q + t * ray_dir)) };

          if (t_dsdf_curr.y <= 0.f)
          {
            t_dsdf_l = t_dsdf_curr;

            // If this is the last ST iteration for the voxel, current point has (SDF' <= 0)
            // and furthest ray point in voxel has (SDF' > 0), set ("current" var, and then) right border
            if ((t >= qFar || dist <= SDF_EPS || iter >= ST_max_iters) && sdf_dsdf_far.y > 0.f)
              t_dsdf_curr = { qFar, sdf_dsdf_far.y };
          }
          if (t_dsdf_curr.y > 0.f)
          {
            t_dsdf_r = t_dsdf_curr;

            if (t_dsdf_l.x < t_dsdf_r.x)
            {
              // Got both borders, start bisection
              const int bisec_num_iters = 12;
              for (int i = 0; i < bisec_num_iters; ++i)
              {
                t_dsdf_curr.x = (t_dsdf_l.x + t_dsdf_r.x) / 2;
                t_dsdf_curr.y = dot(ray_dir, eval_dist_trilinear_diff(values, start_q + t_dsdf_curr.x * ray_dir));
                if (t_dsdf_curr.y <= 0.f)
                  t_dsdf_l = t_dsdf_curr;
                else
                  t_dsdf_r = t_dsdf_curr;
              }
              // Update resulting point if new sdf is lesser
              float2 t_sdf_curr;
              t_sdf_curr.x = std::abs(t_dsdf_l.y) < std::abs(t_dsdf_r.y) ? t_dsdf_l.x : t_dsdf_r.x;
              t_sdf_curr.y = eval_dist_trilinear(values, start_q + t_sdf_curr.x * ray_dir);

              if (t_sdf_curr.x >= 0.f && t_sdf_curr.x <= qFar && t_sdf_curr.y > 0.f && t_sdf_curr.y < t_sdf_res.y)
              {
                t_sdf_res = t_sdf_curr;
              }
              // Bisection END, we're in (SDF' > 0) part, clear borders
              t_dsdf_l = { (qFar + 2) * d_inv, 0.f };
              t_dsdf_r = { (qFar + 1) * d_inv, 0.f };
            }
          }
        }
      }
      hit = (dist <= SDF_EPS);

      if (relax_pt && ray_flags & DR_RAY_FLAG_BORDER)
      {
        if (relax_pt->missed_hit._pad1 == 1)
        {
          if (sdf_dsdf_near.y >= 0)
          {
            if (sdf_dsdf_near.x > t0 && sdf_dsdf_near.x < relax_pt->missed_hit.sdf) // Found better relaxation point
            {
              relax_pt->missed_hit.t = t0;
              relax_pt->missed_hit.sdf = sdf_dsdf_near.x;
            }
          }
          relax_pt->missed_hit._pad1 = 0;
        }
        if (sdf_dsdf_far.x > 0.f && sdf_dsdf_far.y <= 0.f)
          relax_pt->missed_hit._pad1 = 1;

#ifdef DEBUG_PAYLOAD_STORE_SDF
        for (int t_i = 0; t_i <= 10; ++t_i)
        {
          float t_br = float(t_i) / 10.f * qFar;
          relax_pt->sdf_i.push_back(eval_dist_trilinear(values, start_q + t_br * ray_dir));
        }
#endif

        if (t_sdf_res.x >= t0 && t_sdf_res.x <= qFar && t_sdf_res.y > 0.f && t_sdf_res.y < relax_pt->missed_hit.sdf) // Found relaxation point
        {
          relax_pt->missed_hit.t = t_sdf_res.x;
          relax_pt->missed_hit.sdf = t_sdf_res.y;
        }
      }
    }
    else //if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC ||
        //    m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON ||
        //    m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
    {
      //finding exact intersection between surface sdf(x,y,z) = 0 and ray
      // based on paper "Ray Tracing of Signed Distance Function Grids, 
      // Journal of Computer Graphics Techniques (JCGT), vol. 11, no. 3, 94-113, 2022"
      // http://jcgt.org/published/0011/03/06/

      // define values and constants as proposed in paper
      float s000 = values[0]*d_inv;
      float s001 = values[1]*d_inv;
      float s010 = values[2]*d_inv;
      float s011 = values[3]*d_inv;
      float s100 = values[4]*d_inv;
      float s101 = values[5]*d_inv;
      float s110 = values[6]*d_inv;
      float s111 = values[7]*d_inv;

      float a = s101-s001;

      float k0 = s000;
      float k1 = s100-s000;
      float k2 = s010-s000;
      float k3 = s110-s010-k1;
      float k4 = k0-s001;
      float k5 = k1-a;
      float k6 = k2-(s011-s001);
      float k7 = k3-(s111-s011-a);

      float3 o = start_q;
      float3 d3 = ray_dir;

      float m0 = o.x*o.y;
      float m1 = d3.x*d3.y;
      float m2 = o.x*d3.y + o.y*d3.x;
      float m3 = k5*o.z - k1;
      float m4 = k6*o.z - k2;
      float m5 = k7*o.z - k3;

      float c0 = (k4*o.z - k0) + o.x*m3 + o.y*m4 + m0*m5;
      float c1 = d3.x*m3 + d3.y*m4 + m2*m5 + d3.z*(k4 + k5*o.x + k6*o.y + k7*m0);
      float c2 = m1*m5 + d3.z*(k5*d3.x + k6*d3.y + k7*m2);
      float c3 = k7*m1*d3.z;

      // the surface is defined by equation c3*t^3 + c2*t^2 + c1*t + c0 = 0;
      // solve this equation analytically or numerically using the Newton's method
      // see "Numerical Recipes - The Art of Scientific Computing - 3rd Edition" for details

      if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC)
      {
        float x1 = 1000;
        float x2 = 1000;
        float x3 = 1000;
        unsigned type = 0;
        if (std::abs(c3) > 1e-2)
        {
          type = 3;
          //it is a cubic equation, transform it to x^3 + a*x^2 + b*x + c = 0
          //use Vieta method to obtain 3 or 1 real roots
          float a = c2/c3;
          float b = c1/c3;
          float c = c0/c3;   

          float Q = (a*a - 3*b)/9;
          float R = (2*a*a - 9*a*b + 27*c)/54;
          float Q3 = Q*Q*Q;

          if (R*R < Q3) //equation has three real roots
          {
            float theta = std::acos(R/sqrt(Q3));
            x1 = -2*sqrt(Q)*std::cos(theta/3) - a/3;
            x2 = -2*sqrt(Q)*std::cos((theta+2*M_PI)/3) - a/3;
            x3 = -2*sqrt(Q)*std::cos((theta-2*M_PI)/3) - a/3;
          }
          else //equation has only one real roots
          {
            float A = -sign(R)*std::pow(std::abs(R) + sqrt(R*R - Q3), 1.0f/3.0f);
            float B = std::abs(A) > EPS ? Q/A : 0;
            x1 = A+B - a/3;
          }
        }
        else if (std::abs(c2) > 1e-4)
        {
          type = 2;
          //it is a quadratic equation a*x^2 + b*x + c = 0
          float a = c2;
          float b = c1;
          float c = c0;

          float D = b*b - 4*a*c;
          if (D > 0)
          {
            float q = -0.5f*(b + sign(b)*std::sqrt(D));
            x1 = q/a;
            if (std::abs(q) > EPS)
              x2 = c/q; 
          }
        }
        else if (std::abs(c1) > EPS)
        {
          type = 1;
          //it is a linear equation c1*x + c0 = 0
          x1 = -c0/c1;
        }
        //else
        //no roots or inf roots, something's fucked up so just drop it

        x1 = x1 < 0 ? 1000 : x1;
        x2 = x2 < 0 ? 1000 : x2;
        x3 = x3 < 0 ? 1000 : x3;

        //bool prev_hit = hit;
        //float nt = std::min(x1, std::min(x2,x3));
        //if (prev_hit && std::abs(t - nt) > 0.1)
        //  printf("%f-%f -- %f %f %f %f -- %f %f %f, type %u\n",t, nt, c3,c2,c1,c0, x1,x2,x3, type);
        t = std::min(x1, std::min(x2,x3));
        hit = (t >= 0 && t <= qFar);
      }
      else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON)
      {
        // our polynom is c3*t^3 + c2*t^2 + c1*t + c0 = 0;
        // it's derivative is  3*c3*t^2 + 2*c2*t + c1 = 0; 
        // find where it equals 0 to determine interval where the root is located
        // by solving a quadratic equation a*x^2 + b*x + c = 0
        float a = 3*c3;
        float b = 2*c2;
        float c = c1;

        float t0 = 0;
        float t1 = qFar;
        float t2 = qFar;
        float t3 = qFar;

        float D = b*b - 4*a*c;
        if (D >= 0)
        {
          float q = -0.5f*(b + sign(b)*std::sqrt(D));
          t1 = std::abs(a) > EPS ? q/a : t0;
          t2 = std::abs(q) > EPS ? c/q : qFar;

          float tmp = std::min(t1,t2);
          t2 = std::max(t1, t2);
          t1 = tmp;

          t1 = clamp(t1, t0, t3);
          t2 = clamp(t2, t0, t3);
        }
        
        //calculate sign of initial polynom at each critical point
        //sdf0 - sdf3 are inversed sdf values
        float sdf0 = c0;
        float sdf1 = (c0 + t1*(c1 + t1*(c2 + t1*c3)));
        float sdf2 = (c0 + t2*(c1 + t2*(c2 + t2*c3)));
        float sdf3 = (c0 + t3*(c1 + t3*(c2 + t3*c3)));
        bool s0 = sdf0 > 0;
        bool s1 = sdf1 > 0;
        bool s2 = sdf2 > 0;
        bool s3 = sdf3 > 0;

        //determine the range to apply Newton's method
        float nwt_min = t0;
        float nwt_max = t0;
        if (s0 != s1)
        {
          nwt_min = t0;
          nwt_max = t1;
        }
        else if (s1 != s2)
        {
          nwt_min = t1;
          nwt_max = t2;
        }
        else if (s2 != s3)
        {
          nwt_min = t2;
          nwt_max = t3;
        }

        float rtn = -100;

        if (nwt_min < nwt_max)
        {
          //perform Newton's method
          const unsigned max_iters = 10;
          unsigned iter = 0;
          rtn = 0.5f*(nwt_min + nwt_max);
          float f = 1000;
          while (iter < max_iters && std::abs(f) >= SDF_EPS)
          {
            f = c0 + rtn*(c1 + rtn*(c2 + rtn*c3));
            float df = c1 + rtn*(2*c2 + rtn*3*c3);
            float dx = f/(df + sign(df)*1e-9f);
            rtn -= dx;
            iter++;
          }
          t = rtn;
          hit = (t >= 0 && t <= qFar && std::abs(f) < SDF_EPS);
        }
        else
        {
          //no hit
          hit = false;
        }

        if (relax_pt && ray_flags & DR_RAY_FLAG_BORDER)
        {
          // Looking for SDF min
          float t_min = qFar + EPS;
          if (std::abs(a) > EPS)
          {
            // first or second root of (3*c3)*t^2 + (2*c2)*t + c1 = 0  (if x1 != x2)
            float root1 = -0.5f*(b + std::sqrt(D)) / a,
                  root2 = -0.5f*(b - std::sqrt(D)) / a;

            // t_min = sign(a) > 0 ? std::min(root1, root2) : std::max(root1, root2);
            t_min = (sign(a) > 0) == (root1 < root2) ? root1 : root2;
          }
          else if (b < -EPS)
          {
            // root of b*x + c = 0
            t_min = -c / b;
          }

          float sdf_min = (c0 + t_min*(c1 + t_min*(c2 + t_min*c3)));

          if (relax_pt->missed_hit._pad1 == 1)
          {
            if (c1 <= 0) // c1 == SDF'(0)
            {
              if (sdf0 < 0.f && -sdf0*d < relax_pt->missed_hit.sdf) // Found relaxation point
              {
                relax_pt->missed_hit.t = t0;
                relax_pt->missed_hit.sdf = -sdf0*d;
              }
            }
            relax_pt->missed_hit._pad1 = 0;
          }

#ifdef DEBUG_PAYLOAD_STORE_SDF
          for (int t_i = 0; t_i <= 10; ++t_i)
          {
            float t_br = float(t_i) / 10.f * qFar;
            relax_pt->sdf_i.push_back(-d*(c0 + t_br*(c1 + t_br*(c2 + t_br*c3))));
          }
#endif

          if (false)
          {
          printf("dinv = %f\n", d_inv);
          printf("values %f %f %f %f %f %f %f %f\n", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
          printf("t0 = %f t1 = %f t2 = %f t3 = %f, t = %f\n", t0, t1, t2, t3, t);
          printf("sdf0 = %f %fsdf1 = %f sdf2 = %f sdf3 = %f, sdf = %f\n\n", 
                 eval_dist_trilinear(values, start_q + t0 * ray_dir),
                 sdf0, sdf1, sdf2, sdf3,
                 c0 + t*(c1 + t*(c2 + t*c3)));
          }

          //if (!hit)
          //  printf("t_min = %f in (%f, %f)\n", t_min, qNear, qFar);

          if (t_min >= 0 && t_min <= qFar && sdf_min < 0.f && -sdf_min*d < relax_pt->missed_hit.sdf) // Found relaxation point
          {
            //printf("t_min = %f in (%f, %f)\n", t_min, qNear, qFar);
            relax_pt->missed_hit.t = t_min;
            relax_pt->missed_hit.sdf = -sdf_min*d;
          }
          if (sdf3 < 0.f && -sdf3*d < relax_pt->missed_hit.sdf && (c1 + qFar*(2*c2 + qFar*3*c3)) >= 0.f) // sdf3 is SDF(qFar)
          {
            relax_pt->missed_hit._pad1 = 1;
            relax_pt->missed_hit_candidate.t = qFar;
            relax_pt->missed_hit_candidate.sdf = -sdf3*d;

          }
        }

        //bool prev_hit = hit;
        //float nt = rtn;
        //if (prev_hit && std::abs(t - nt) > 0.1)
        //  printf("%f-%f -- %f %f %f %f -- %f -- %f %f %f %f %d %d %d %d\n",t, nt, c3,c2,c1,c0, rtn, t0, t1, t2, t3, s0, s1, s2, s3);
      }
      else //if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
      {
        const unsigned IT_max_iters = 256;
        const float k = 2;

        float e = 0.1f*qFar;
        float t_max = std::abs(c3) < EPS ? 1e6 : -c2/(3*c3);
        float df_max = 3*c3*t_max*t_max + 2*c2*t_max + c1;

        float dist = start_dist;
        float3 pp = start_q + t * ray_dir;

        while (t < qFar && dist > SDF_EPS && iter < IT_max_iters)
        {
          float df_1 = 3*c3*t*t + 2*c2*t + c1;
          float df_2 = 3*c3*(t+e)*(t+e) + 2*c2*(t+e) + c1;
          float L = (t_max > t && t_max < t + e) ? std::max(df_max, std::max(df_1, df_2)) : std::max(df_1, df_2);
          L = std::max(L, EPS);
          float s = std::min((dist*d_inv)/L, e);
          t += s;
          e = k*s;
          dist = eval_dist_trilinear(values, start_q + t * ray_dir);
          pp = start_q + t * ray_dir;
          iter++;
        }
        hit = (dist <= SDF_EPS);
      
      }
    }

    return hit ? t : qFar + EPS;
  }

  void BVHDR::dIntersect_dValues(uint32_t ray_flags, const float3 ray_dir, float values[8], float d,
                                 float qNear, float qFar, float3 start_q, float out_dValues[8])
  {
    //use finite differences to calculate dValues
    float delta = 0.001f;

    for (int i = 0; i < 8; i++)
    {
      values[i] += delta;
      float d1 = Intersect(ray_flags, ray_dir, values, d, qNear, qFar, start_q, nullptr);
      values[i] -= 2 * delta;
      float d2 = Intersect(ray_flags, ray_dir, values, d, qNear, qFar, start_q, nullptr);
      values[i] += delta;
      out_dValues[i] = (d1 - d2) / (2 * delta);
    }
  }

float BVHDR::load_distance_values_with_indices(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, 
                                               float values[8], uint32_t indices[8])
{
  float vmin = 1e6f;
  if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
      header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    for (int i = 0; i < 8; i++)
    {
      uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint32_t vId = vPos.x * v_size * v_size + vPos.y * v_size + vPos.z;
      uint32_t idx = m_SdfSBSData[v_off + vId];
      values[i] = m_SdfSBSDataF[idx];
      indices[i] = idx;
      // printf("%f\n", values[i]);
      vmin = std::min(vmin, values[i]);
    }
  }
  else
  {
    assert(false);
  }

  return vmin;
}

static float3 dp_to_nmq(float3 dp, float beta)
{
  float3 nmq = float3(0.5f, 0.5f, 0.5f);
  nmq.x = dp.x < 0.5f ? step(-beta, beta, dp.x) : step(-beta, beta, dp.x - 1.0f);
  nmq.y = dp.y < 0.5f ? step(-beta, beta, dp.y) : step(-beta, beta, dp.y - 1.0f);
  nmq.z = dp.z < 0.5f ? step(-beta, beta, dp.z) : step(-beta, beta, dp.z - 1.0f);
  return nmq;
}

  //It is basically a copy of BVHRT::OctreeBrickIntersect, but with additional calculation
  //of gradients. And also it supports only indexed SBS (e.g. SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F layout)
  void BVHDR::OctreeBrickIntersectWithGrad(uint32_t type, uint32_t ray_flags, const float3 ray_pos, const float3 ray_dir,
                                           float tNear, uint32_t instId, uint32_t geomId,
                                           uint32_t bvhNodeId, uint32_t a_count,
                                           RayDiffPayload *relax_pt, CRT_HitDR *pHit)
  {
    float values[8];
    uint32_t nodeId, primId;
    float d, qNear, qFar;
    float2 fNearFar;
    float3 start_q;

    qNear = 1.0f;

    uint32_t sdfId = m_geomData[geomId].offset.x;
    primId = bvhNodeId; // id of bbox in BLAS
    nodeId = primId + m_SdfSBSRoots[sdfId];
    SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
    uint32_t v_size = header.brick_size + 2 * header.brick_pad + 1;

    float px = m_SdfSBSNodes[nodeId].pos_xy >> 16;
    float py = m_SdfSBSNodes[nodeId].pos_xy & 0x0000FFFF;
    float pz = m_SdfSBSNodes[nodeId].pos_z_lod_size >> 16;
    float sz = m_SdfSBSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
    float sz_inv = 2.0f / sz;

    d = 2.0f / (sz * header.brick_size);

    float3 brick_min_pos = float3(-1, -1, -1) + sz_inv * float3(px, py, pz);
    float3 brick_max_pos = brick_min_pos + sz_inv * float3(1, 1, 1);
    float3 brick_size = brick_max_pos - brick_min_pos;

    float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
    float old_t = pHit->t;
    while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
    {
      float3 hit_pos = ray_pos + brick_fNearFar.x * ray_dir;
      float3 local_pos = (hit_pos - brick_min_pos) * (0.5f * sz * header.brick_size);
      float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size - 1e-6f));

      float3 min_pos = brick_min_pos + d * voxelPos;
      float3 max_pos = min_pos + d * float3(1, 1, 1);
      float3 size = max_pos - min_pos;
      
      uint32_t  missed_indices_tmp[8];

      float vmin = 1.0f;

      if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
          header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
      {
        uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
        for (int i = 0; i < 8; i++)
        {
          uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          uint32_t vId = vPos.x * v_size * v_size + vPos.y * v_size + vPos.z;
          missed_indices_tmp[i] = m_SdfSBSData[v_off + vId];
          values[i] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
          // printf("%f\n", values[i]);
          vmin = std::min(vmin, values[i]);
        }
      }
      else
      {
        //you should not be here
        for (int i = 0; i < 8; i++)
          missed_indices_tmp[i] = 0u;
      }

      fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
      if (tNear < fNearFar.x)
      {
        float3 start_pos = ray_pos + fNearFar.x * ray_dir;
        start_q = (start_pos - min_pos) * (0.5f * sz * header.brick_size);
        qFar = (fNearFar.y - fNearFar.x) * (0.5f * sz * header.brick_size);

        float prev_missed_hit_sdf = relax_pt->missed_hit.sdf;
        float t = Intersect(ray_flags, ray_dir, values, d, 0.0f, qFar, start_q, relax_pt); //relax_pt->missed_hit.t is relative here
        float tReal = fNearFar.x + d * t;
        
        if (ray_flags & DR_RAY_FLAG_BORDER)
        {
          //we found better candidate for a closest missed hit along the ray
          //we need to recalculate color, normal and derivatives in a new point
          if (relax_pt->missed_hit.sdf < prev_missed_hit_sdf && relax_pt->missed_hit.t < t)
          {
            float missed_t = relax_pt->missed_hit.t;
            relax_pt->missed_hit.t = fNearFar.x + d * relax_pt->missed_hit.t; //MultiRendrer expectes t as an absolute distance
            float3 q_ast = start_q + missed_t * ray_dir;
            float3 y_ast = ray_pos + (fNearFar.x + (d * missed_t)) * ray_dir;
            float3 dp_ast = (y_ast - brick_min_pos) * (0.5f * sz);
            float3 dSDF_dpos = eval_dist_trilinear_diff(values, q_ast)*(1/d); // grad

            //eval_dist_trilinear_diff calculates normal by the values in given voxel
            //however, the minimum point is often on the voxel's border and normal on the
            //border is not the same, bu we should calculate the smoothed normal because
            //we need exactly the geomentry normal, i.e. dSDF_dpos.
            //and this multiplier is a small hack how estimate this value
            float3 border_normal_mult = float3(std::abs(q_ast.x - 0.5f) > 0.5f - 1e-5f ? 0.0f : 1.0f,
                                               std::abs(q_ast.y - 0.5f) > 0.5f - 1e-5f ? 0.0f : 1.0f,
                                               std::abs(q_ast.z - 0.5f) > 0.5f - 1e-5f ? 0.0f : 1.0f);
            dSDF_dpos *= border_normal_mult;
            float dSDF_dpos_len = length(dSDF_dpos);
            
            if (dSDF_dpos_len > 1e-9f) 
            { 
              float dSDF_dvalues[8];
              eval_dist_trilinear_d_dtheta(dSDF_dvalues, q_ast); // dSDF/dTheta
              for (int i=0; i<8; ++i)
              {
                relax_pt->missed_dSDF_dtheta[i] = -dSDF_dvalues[i]/dSDF_dpos_len;
              }

              uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size * v_size * v_size;
              float3 colors[8];
              for (int i = 0; i < 8; i++)
                colors[i] = float3(m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 0],
                                   m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 1],
                                   m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 2]);
              
              relax_pt->missed_hit.color = eval_color_trilinear(colors, dp_ast);
              relax_pt->missed_hit.normal = dSDF_dpos / dSDF_dpos_len;

              for (int i = 0; i < 8; i++)
                relax_pt->missed_indices[i] = missed_indices_tmp[i];
            }
            // else: result is 0, ignore
          }
        }

        if (t <= qFar && tReal < pHit->t)
        {
          //assume 1) header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F(_IN)
          //       2) no instances

          pHit->t = tReal;
          pHit->primId = primId;
          pHit->instId = instId;
          pHit->geomId = geomId | (type << SH_TYPE);

          //with normal smoothing normal can depend on up to 64 SDF values
          //while actual intersection position depends only on 8 main values
          //as they are stored in the same structure, we should find out where
          //to store these main values
          unsigned dvalues_offset = 0;
          if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
          {
            float3 p0 = start_q + t * ray_dir;
            dvalues_offset += p0.x < 0.5f ? 4 : 0;
            dvalues_offset += p0.y < 0.5f ? 2 : 0;
            dvalues_offset += p0.z < 0.5f ? 1 : 0;
            dvalues_offset *= 8;
          }

          //calculate dt_dvalues
          float dt_dvalues[8];
          if (ray_flags & (DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS | DR_RAY_FLAG_DDIST_DPOS))
          {
            dIntersect_dValues(ray_flags, ray_dir, values, d, 0.0f, qFar, start_q, dt_dvalues);
            
            uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
            for (int i = 0; i < 8; i++)
            {
              uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
              uint32_t vId = vPos.x * v_size * v_size + vPos.y * v_size + vPos.z;

              relax_pt->dDiffuseNormal_dSd[dvalues_offset+i].index = m_SdfSBSData[v_off + vId];
              relax_pt->dDiffuseNormal_dSd[dvalues_offset+i].dDist = d * dt_dvalues[i]; //d(tReal)/dS
            }
          }

          //calculate hit normal and dNormal_dSd
          if (need_normal())
          {
            if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
            {
              /*
              variables used in comments:
              v_i - 8 values from i-th neighbor voxel
              k   - main voxel (so variable "values" is v_k)
              n_i - normal from i-th neighbor voxel
              trilinear3 - trilinear interpolation for normals
              other variables have the same names as in code

              During normals calculation relax_pt->dDiffuseNormal_dSd[dvalues_offset+i].index
              will be overwritten, but it's ok, if code is correct, index will remain the same
              */
              const float beta = 0.5f;
              float3 dp = start_q + t * ray_dir; //linear interpolation coefficients in voxels
              
              float3 normals[8];
              float values_n[8];
              uint32_t indices_n[8];

              float dD_d1[24];
              float dtrilinear3_di[8]; //it should be 8 3x3 matrices, but they are diagonal, so it's actually scalar
              float3 dnmq_ddp = float3(1,1,1);
              
              float3 nmq = dp_to_nmq(dp, beta);
              uint32_t k = dvalues_offset / 8;

              for (int i=0;i<8;i++)
                normals[i] = float3(0,0,0);

              if (ray_flags & DR_RAY_FLAG_DNORM_DPOS)
              {   
                eval_dist_trilinear_d_dtheta(dtrilinear3_di, nmq);
              }

              int3 voxelPos0 = int3(dp.x < 0.5f ? voxelPos.x - 1 : voxelPos.x, dp.y < 0.5f ? voxelPos.y - 1 : voxelPos.y, dp.z < 0.5f ? voxelPos.z - 1 : voxelPos.z);
              for (int i=0;i<8;i++)
              {
                int3 VoxelPosI = voxelPos0 + int3((i & 4) >> 2, (i & 2) >> 1, i & 1);
                int3 BrickPosI = int3((VoxelPosI.x >= 0 ? (VoxelPosI.x < header.brick_size ? 0 : 1) : -1),
                                      (VoxelPosI.y >= 0 ? (VoxelPosI.y < header.brick_size ? 0 : 1) : -1),
                                      (VoxelPosI.z >= 0 ? (VoxelPosI.z < header.brick_size ? 0 : 1) : -1));

                uint32_t neighborId = 0;
                neighborId += 3*3 * (BrickPosI.x + 1);
                neighborId +=   3 * (BrickPosI.y + 1);
                neighborId +=       (BrickPosI.z + 1);
                
                float3 dVoxelPos = float3(VoxelPosI) - voxelPos;
                uint32_t neighbor_nodeId;
                float3   neighbor_voxelPos;

                if (neighborId == 9+3+1)//we have our neighbor voxel in the same brick
                {
                  neighbor_nodeId = nodeId;
                  neighbor_voxelPos = float3(VoxelPosI);
                }
                else //we have our neighbor voxel in a different brick, read it from neigbors data
                {
                  uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
                  uint32_t neighbors_data_offset = v_size*v_size*v_size + 8;

                  neighbor_nodeId = m_SdfSBSData[v_off + neighbors_data_offset + neighborId];
                  neighbor_voxelPos = float3(VoxelPosI) - header.brick_size*float3(BrickPosI);
                }

                if (neighbor_nodeId != INVALID_IDX)
                {
                  load_distance_values_with_indices(neighbor_nodeId, neighbor_voxelPos, v_size, sz_inv, header, values_n, indices_n);
                  float3 dSDF_ddp = eval_dist_trilinear_diff(values_n, dp - dVoxelPos);
                  normals[i] = normalize(dSDF_ddp);
                
                  if (ray_flags & DR_RAY_FLAG_DNORM_DPOS)
                  {
                    //set indices for values_n
                    for (int j=0;j<8;j++)
                    {
                      relax_pt->dDiffuseNormal_dSd[8*i+j].index = indices_n[j];
                    }
                    //here we load dtrilinear3_di * d(n_i)_d(v_i) into dDiffuseNormal_dSd[8*i+j].dNorm
                    //and  dtrilinear3_di * d(n_i)_d(v_k) into dDiffuseNormal_dSd[8*k+j].dNorm

                    //if i != k
                    //  d(n_i)_d(v_i)[3x8] = dnormalize_dD[3x3] * dD_d1[3x8]
                    //  d(n_i)_d(v_k)[3x8] = dnormalize_dD[3x3] * ((dD_d2[3x3] * ray_dir[3x1])[3x1] * dt_dvalues[1x8])
                    //if i == k
                    //  d(n_k)_d(v_k)[3x8] = dnormalize_dD[3x3] * (dD_d1[3x8] + (dD_d2[3x3] * ray_dir[3x1])[3x1] * dt_dvalues[1x8])

                    float3x3 dNormalize_dD = normalize_diff(dSDF_ddp);
                    eval_dist_trilinear_ddp_dvalues(values_n, dp - dVoxelPos, dD_d1);
                    float3x3 dD_d2         = eval_dist_trilinear_ddp_ddp(values_n, dp - dVoxelPos);

                    //  d(n_i)_d(v_i)
                    for (int j = 0; j < 8; j++)
                    {
                      float3 dD_dv_ij = float3(dD_d1[8*0 + j], dD_d1[8*1 + j], dD_d1[8*2 + j]);
                      relax_pt->dDiffuseNormal_dSd[8*i+j].dNorm += dtrilinear3_di[i] * (dNormalize_dD * dD_dv_ij);
                    }

                    //  d(n_i)_d(v_k)
                    float3 t1 = dD_d2*ray_dir;
                    for (int j = 0; j < 8; j++) //dD_d1 = (dD_d2 * ray_dir) * dt_dvalues
                    {
                      dD_d1[8*0 + j] = t1.x*dt_dvalues[j];
                      dD_d1[8*1 + j] = t1.y*dt_dvalues[j];
                      dD_d1[8*2 + j] = t1.z*dt_dvalues[j];
                    }
                    for (int j = 0; j < 8; j++)
                    {
                      float3 dD_dv_ij = float3(dD_d1[8*0 + j], dD_d1[8*1 + j], dD_d1[8*2 + j]);
                      relax_pt->dDiffuseNormal_dSd[8*k+j].dNorm += dtrilinear3_di[i] * (dNormalize_dD * dD_dv_ij);
                    }
                  }
                }
              }

              float3 trilinear3 = eval_color_trilinear(normals, nmq);
              float3 smoothed_norm = normalize(trilinear3);
              pHit->normal = smoothed_norm;

              if (ray_flags & DR_RAY_FLAG_DNORM_DPOS)
              {
                //if i != k
                //dnormal_dv_i = dNormalize_dD * (dtrilinear3_di * d(n_i)_d(v_i))
                //             = dNormalize_dD * dDiffuseNormal_dSd[8*i+0 - 8*i+7].dNorm
                //dnormal_dv_k = dNormalize_dD * (sum{from 0 to 7}{dtrilinear3_di * d(n_i)_d(v_k)} + dtrilinear3_dnmq * dnmq_ddp * ddp_dv_k)
                
                //dDiffuseNormal_dSd[8*i+0 - 8*i+7].dNorm += dtrilinear3_dnmq * dnmq_ddp * ddp_dv_k
                //dnmq_ddp is a diagonal matrix, so it can be represented as vector
                float3x3 dNormalize_dD = normalize_diff(trilinear3);
                float3x3 dtrilinear3_dnmq = eval_color_trilinear_ddp(normals, nmq);
                for (int j=0;j<8;j++)
                {
                  float3 ddp_dv_kj = ray_dir * dt_dvalues[j];
                  relax_pt->dDiffuseNormal_dSd[8*k+j].dNorm += dtrilinear3_dnmq * (dnmq_ddp * ddp_dv_kj);
                }

                for (int i=0;i<8;i++)
                {
                  if (length(normals[i]) > 0.5) //normal is valid
                  {
                    //dnormal_dv_i
                    for (int j=0;j<8;j++)
                    {
                      relax_pt->dDiffuseNormal_dSd[8*i+j].dNorm = dNormalize_dD * relax_pt->dDiffuseNormal_dSd[8*i+j].dNorm;
                    }
                  }
                }

              }
            }
            else
            {
              //WARNING: NORMAL FIELD IS NOT CONTINUOUS HERE
              //use SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN for normal smmothing
              //or tricubic sampling for C1 surface and continuous normal field
            
              //Normal
              float3 p0 = start_q + t * ray_dir;
              float3 dSDF_dp0 = eval_dist_trilinear_diff(values, p0);
              pHit->normal = normalize(dSDF_dp0);

              //dNormal_dSd
              if (ray_flags & DR_RAY_FLAG_DNORM_DPOS)
              {
                float3x3 dNormalize_dD = normalize_diff(dSDF_dp0);
                float dD_d1[24];         eval_dist_trilinear_ddp_dvalues(values, p0, dD_d1);
                float3x3 dD_d2         = eval_dist_trilinear_ddp_ddp(values, p0);

                //normal = normalize(eval_dist_trilinear_diff(values, start_q + t(values) * ray_dir))
                //dnormal_dvalues[3x8] = dnormalize_dD[3x3] * (dD_d1[3x8] + (dD_d2[3x3] * ray_dir[3x1])[3x1] * dt_dvalues[1x8])
                float3 t1 = dD_d2*ray_dir;

                for (int i = 0; i < 8; i++) //dD_d1 = dD_d1 + (dD_d2 * ray_dir) * dt_dvalues
                {
                  dD_d1[8*0 + i] += t1.x*dt_dvalues[i];
                  dD_d1[8*1 + i] += t1.y*dt_dvalues[i];
                  dD_d1[8*2 + i] += t1.z*dt_dvalues[i];
                }

                for (int i = 0; i < 8; i++)
                  relax_pt->dDiffuseNormal_dSd[dvalues_offset+i].dNorm = dNormalize_dD * float3(dD_d1[8*0 + i], dD_d1[8*1 + i], dD_d1[8*2 + i]);
              }
            }
          }

          //calculate color, dDiffuse_dSc and dDiffuse_dSd
          if (true)
          {
            //Diffuse 
            float3 pos = ray_pos + pHit->t * ray_dir;
            float3 dp = (pos - brick_min_pos) * (0.5f * sz);
            uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size * v_size * v_size;
            float3 colors[8];
            for (int i = 0; i < 8; i++)
              colors[i] = float3(m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 2]);
            pHit->color = eval_color_trilinear(colors, dp);

            //dDiffuse_dSc
            if (ray_flags & DR_RAY_FLAG_DDIFFUSE_DCOLOR)
            {
              for (int i = 0; i < 8; i++)
              {
                float3 q = float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
                float3 lq = q*dp + (1-q)*(1-dp); //linear interpolation quotients, as above
                relax_pt->dDiffuse_dSc[i].index = m_SdfSBSData[t_off + i];
                relax_pt->dDiffuse_dSc[i].value = lq.x*lq.y*lq.z;
              }
            }

            //dDiffuse_dSd
            //Diffuse = Lerp(colors, (ray_pos + t(values) * ray_dir - brick_min_pos) * (0.5f * sz))
            //dDiffuse_dValues = dLerp_d1[3x3] * 0.5f*sz*ray_dir[3x1] * dt_dvalues[1x8]
            if (ray_flags & DR_RAY_FLAG_DDIFFUSE_DPOS)
            {
              float3x3 dLerp_d1 = eval_color_trilinear_ddp(colors, dp);
              for (int i = 0; i < 8; i++)
              {
                relax_pt->dDiffuseNormal_dSd[dvalues_offset+i].dDiffuse = dLerp_d1 * float3(0.5f*sz*ray_dir.x*dt_dvalues[i], 0.5f*sz*ray_dir.y*dt_dvalues[i], 0.5f*sz*ray_dir.z*dt_dvalues[i]);
              }
            }
          }
#if VERIFY_DERIVATIVES
          if (ray_flags & DR_RAY_FLAG_DNORM_DPOS)
          {
            static unsigned count = 0;
            static unsigned e_count = 0;
            static long double sliding_average = 0;

            float3 dNorm_dValues[8];
            float3 dDiffuse_dValues[8];
            float delta = 0.0001f;

            float3 norm[2];
            float3 color[2];
            for (int i = 0; i < 8; i++)
            {
              for (int j = 0; j < 2; j++)
              {
                values[i] += j == 0 ? delta : -delta;

                float t = Intersect(ray_flags, ray_dir, values, d, 0.0f, qFar, start_q, nullptr);
                float tReal = fNearFar.x + 2.0f * d * t;
                float3 p0 = start_q + t * ray_dir;
                float3 dSDF_dp0 = eval_dist_trilinear_diff(values, p0);
                norm[j] = normalize(dSDF_dp0);
                float3 pos = ray_pos + t * ray_dir;
                float3 dp = (pos - brick_min_pos) * (0.5f * sz);
                uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size * v_size * v_size;
                float3 colors[8];
                for (int i = 0; i < 8; i++)
                  colors[i] = float3(m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + i] + 2]);
                color[j] = eval_color_trilinear(colors, dp);

                values[i] -= j == 0 ? delta : -delta;
              }

              dNorm_dValues[i] = (norm[0] - norm[1]) / (2.0f * delta);
              dDiffuse_dValues[i] = (color[0] - color[1]) / (2.0f * delta);
            }

            float l1 = 0;
            float l2 = 0;
            float ldiff = 0;

            for (int i = 0; i < 8; i++)
            {
              l1 += dot(dNorm_dValues[i], dNorm_dValues[i]);
              l2 += dot(relax_pt->dDiffuseNormal_dSd[i].dNorm, relax_pt->dDiffuseNormal_dSd[i].dNorm);
              ldiff += dot(relax_pt->dDiffuseNormal_dSd[i].dNorm - dNorm_dValues[i], 
                           relax_pt->dDiffuseNormal_dSd[i].dNorm - dNorm_dValues[i]);
            }

            l1 = sqrt(l1);
            l2 = sqrt(l2);
            ldiff = sqrt(ldiff);
            float l = 0.5f*(l1 + l2) + 1e-10f;

            sliding_average = (count == 0) ? l : 0.999f*sliding_average + 0.001f*l;

            count++;
            if (ldiff > 0.05f*l && ldiff > 0.05f*sliding_average)
            {
              e_count++;
              if (false)
              {
              printf("diff error: %f %f\n", ldiff/l, (float)(ldiff/sliding_average));
              for (int i = 0; i < 8; i++)
              {
                printf("(%f %f %f)    (%f %f %f)\n", 
                       relax_pt->dDiffuseNormal_dSd[i].dNorm.x, relax_pt->dDiffuseNormal_dSd[i].dNorm.y, relax_pt->dDiffuseNormal_dSd[i].dNorm.z,
                       dNorm_dValues[i].x, dNorm_dValues[i].y, dNorm_dValues[i].z);
              }
              printf("\n");
              }
            }
            if (count % 1000000 == 0)
              printf("dNorm_dValues: %u errors out of %uM tries\n", e_count, count/1000000u);
          }
#endif

          break;
        }
      }

      brick_fNearFar.x += std::max(0.0f, fNearFar.y - brick_fNearFar.x) + 1e-6f;
    }
  }
}