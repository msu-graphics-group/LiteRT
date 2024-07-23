#include "BVH2DR.h"

using LiteMath::M_PI;
using LiteMath::clamp;

namespace dr
{
  //it is an EXACT COPY of BVHRT::RayQuery_NearestHit, just using CRT_HitDR instead of CRT_Hit
  //but no change it TLAS traversal is needed
  CRT_HitDR BVHDR::RayQuery_NearestHitWithGrad(float4 posAndNear, float4 dirAndFar)
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
    hit.coords[0] = 1.0f;
    hit.coords[1] = 0.0f;
    hit.coords[2] = 0.0f;
    hit.coords[3] = 0.0f;

    for (int i=0;i<8;i++)
      hit.dDiffuse_dS[i].index = PD::INVALID_INDEX;

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
      
          BVH2TraverseF32WithGrad(ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, &hit);
        }
      } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //
    }

    if(hit.geomId < uint32_t(-1) && ((hit.geomId >> SH_TYPE) == TYPE_MESH_TRIANGLE)) 
    {
      const uint2 geomOffsets = m_geomData[hit.geomId & 0x0FFFFFFF].offset;
      hit.primId = m_primIndices[geomOffsets.x/3 + hit.primId];
    }
    
    return hit;
  }

  //it is an EXACT COPY of BVHRT::BVH2TraverseF32, just using CRT_HitDR instead of CRT_Hit
  //but no change it BLAS traversal is needed
  void BVHDR::BVH2TraverseF32WithGrad(const float3 ray_pos, const float3 ray_dir, float tNear,
                                      uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
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
        IntersectAllPrimitivesInLeafWithGrad(ray_pos, ray_dir, tNear, instId, geomId, start, count, pHit);
      }

      // continue BVH traversal
      //
      top--;
      leftNodeOffset = stack[std::max(top,0)];

    } // end while (top >= 0)

  }

  //It is generally a copy of BVHRT::IntersectAllPrimitivesInLeaf, but with less types supported
  //After implementing HW RT acceleration in BVHRT, this can change
  void BVHDR::IntersectAllPrimitivesInLeafWithGrad(const float3 ray_pos, const float3 ray_dir,
                                                   float tNear, uint32_t instId, uint32_t geomId,
                                                   uint32_t a_start, uint32_t a_count,
                                                   CRT_HitDR *pHit)
  {
    uint32_t type = m_geomData[geomId].type;
    const float SDF_BIAS = 0.1f;
    const float tNearSdf = std::max(tNear, SDF_BIAS);
    switch (type)
    {
    case TYPE_MESH_TRIANGLE:
      IntersectAllTrianglesInLeafWithGrad(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
      break;
    case TYPE_SDF_SBS_COL:
      OctreeBrickIntersectWithGrad(type, ray_pos, ray_dir, tNearSdf, instId, geomId, a_start, a_count, pHit);
      break;
    default:
      break;
    }
  }

  void BVHDR::IntersectAllTrianglesInLeafWithGrad(const float3 ray_pos, const float3 ray_dir,
                                                  float tNear, uint32_t instId, uint32_t geomId,
                                                  uint32_t a_start, uint32_t a_count,
                                                  CRT_HitDR *pHit)
  {
    //TODO if we want diff rendering of triangles 
  }

  //Currently it is and exact copy of BVHRT::LocalSurfaceIntersection, but it will change later
  //because differential rendering of SDF requires collection additional data during intersection search
  void BVHDR::LocalSurfaceIntersectionWithGrad(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                               float values[8], uint32_t nodeId, uint32_t primId, float d, float qNear,
                                               float qFar, float2 fNearFar, float3 start_q,
                                               CRT_HitDR *pHit)
  {
    const float EPS = 1e-6f;
    float d_inv = 1.0f / d;
    float t = qNear;
    bool hit = false;
    unsigned iter = 0;

    float start_dist = eval_dist_trilinear(values, start_q + t * ray_dir);
    if (start_dist <= EPS || m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_BBOX)
    {
      hit = true;
    }
    else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ST)
    {
      const unsigned ST_max_iters = 256;
      float dist = start_dist;
      float3 pp0 = start_q + t * ray_dir;

      while (t < qFar && dist > EPS && iter < ST_max_iters)
      {
        t += dist * d_inv;
        dist = eval_dist_trilinear(values, start_q + t * ray_dir);
        float3 pp = start_q + t * ray_dir;
        iter++;
      }
      hit = (dist <= EPS);
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
        bool s0 = c0 > 0;
        bool s1 = (c0 + t1*(c1 + t1*(c2 + t1*c3))) > 0;
        bool s2 = (c0 + t2*(c1 + t2*(c2 + t2*c3))) > 0;
        bool s3 = (c0 + t3*(c1 + t3*(c2 + t3*c3))) > 0;

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
          while (iter < max_iters && std::abs(f) >= EPS)
          {
            f = c0 + rtn*(c1 + rtn*(c2 + rtn*c3));
            float df = c1 + rtn*(2*c2 + rtn*3*c3);
            float dx = f/(df + sign(df)*1e-9f);
            rtn -= dx;
          }
          t = rtn;
          hit = (t >= 0 && t <= qFar && std::abs(f) < EPS);
        }
        else
        {
          //no hit
          hit = false;
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

        while (t < qFar && dist > EPS && iter < IT_max_iters)
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
        hit = (dist <= EPS);
      
      }
    }

    float tReal = fNearFar.x + 2.0f * d * t;


  #if ON_CPU==1
    if (debug_cur_pixel)
    {
      printf("\n");
      printf("sdf type = %u\n", type);
      //printf("node bbox [(%f %f %f)-(%f %f %f)]\n", min_pos.x, min_pos.y, min_pos.z, max_pos.x, max_pos.y, max_pos.z);
      printf("sdf values %f %f %f %f %f %f %f %f\n", 
            values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7]);
      printf("t = %f in [0, %f], tReal = %f in [%f %f]\n",t,qFar,tReal,fNearFar.x,fNearFar.y);
      printf("ray_dir = (%f %f %f)\n", ray_dir.x, ray_dir.y, ray_dir.z);
      //printf("ray_pos = (%f %f %f)\n", ray_pos.x, ray_pos.y, ray_pos.z);
      printf("\n");
    }
  #endif

    if (t <= qFar && hit && tReal < pHit->t)
    {
      float3 norm = float3(0, 0, 1);
      if (need_normal())
      {
        float3 p0 = start_q + t * ray_dir;
        const float h = 0.001;
        float ddx = (eval_dist_trilinear(values, p0 + float3(h, 0, 0)) -
                    eval_dist_trilinear(values, p0 + float3(-h, 0, 0))) /
                    (2 * h);
        float ddy = (eval_dist_trilinear(values, p0 + float3(0, h, 0)) -
                    eval_dist_trilinear(values, p0 + float3(0, -h, 0))) /
                    (2 * h);
        float ddz = (eval_dist_trilinear(values, p0 + float3(0, 0, h)) -
                    eval_dist_trilinear(values, p0 + float3(0, 0, -h))) /
                    (2 * h);

        norm = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
      }
      pHit->t = tReal;
      pHit->primId = primId;
      pHit->instId = instId;
      pHit->geomId = geomId | (type << SH_TYPE);
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = norm.x;
      pHit->coords[3] = norm.y;

      if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
        pHit->primId = iter;
    }
  }

  //It is basically a copy of BVHRT::OctreeBrickIntersect, but with additional calculation
  //of gradients. And also it supports only indexed SBS (e.g. SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F layout)
  void BVHDR::OctreeBrickIntersectWithGrad(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                           float tNear, uint32_t instId, uint32_t geomId,
                                           uint32_t bvhNodeId, uint32_t a_count,
                                           CRT_HitDR *pHit)
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

      float vmin = 1.0f;

      if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
      {
        uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
        for (int i = 0; i < 8; i++)
        {
          uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          uint32_t vId = vPos.x * v_size * v_size + vPos.y * v_size + vPos.z;
          values[i] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
          // printf("%f\n", values[i]);
          vmin = std::min(vmin, values[i]);
        }
      }
      //else - error

      fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
      if (tNear < fNearFar.x && vmin <= 0.0f)
      {
        float3 start_pos = ray_pos + fNearFar.x * ray_dir;
        start_q = (start_pos - min_pos) * (0.5f * sz * header.brick_size);
        qFar = (fNearFar.y - fNearFar.x) * (0.5f * sz * header.brick_size);

        LocalSurfaceIntersectionWithGrad(type, ray_dir, instId, geomId, values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                                         pHit);                                                                                   /*out*/
      }

      brick_fNearFar.x += std::max(0.0f, fNearFar.y - brick_fNearFar.x) + 1e-6f;
    }

    // ray hit a brick
    if (pHit->t < old_t)
    {
      float3 pos = ray_pos + pHit->t * ray_dir;
      float3 dp = (pos - brick_min_pos) * (0.5f * sz);
      if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
      {
        uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size * v_size * v_size;

        float3 color = (1 - dp.x) * (1 - dp.y) * (1 - dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 0] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 0] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 0] + 2]) +
                       (1 - dp.x) * (1 - dp.y) * (dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 1] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 1] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 1] + 2]) +
                       (1 - dp.x) * (dp.y) * (1 - dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 2] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 2] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 2] + 2]) +
                       (1 - dp.x) * (dp.y) * (dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 3] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 3] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 3] + 2]) +
                       (dp.x) * (1 - dp.y) * (1 - dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 4] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 4] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 4] + 2]) +
                       (dp.x) * (1 - dp.y) * (dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 5] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 5] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 5] + 2]) +
                       (dp.x) * (dp.y) * (1 - dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 6] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 6] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 6] + 2]) +
                       (dp.x) * (dp.y) * (dp.z) * float3(m_SdfSBSDataF[m_SdfSBSData[t_off + 7] + 0], m_SdfSBSDataF[m_SdfSBSData[t_off + 7] + 1], m_SdfSBSDataF[m_SdfSBSData[t_off + 7] + 2]);

        color = clamp(floor(255.0f * color + 0.5f), 0.0f, 255.0f);

        pHit->coords[0] = color.x + color.y / 256.0f;
        pHit->coords[1] = color.z;
        // printf("color = %f %f %f coords = %f %f\n", color.x, color.y, color.z, pHit->coords[0], pHit->coords[1]);

        // set dDiffuse_dS
        for (int i = 0; i < 8; i++)
        {
          float3 q = float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          float3 t = q*dp + (1-q)*(1-dp); //linear interpolation quotients, as above
          pHit->dDiffuse_dS[i].index = m_SdfSBSData[t_off + i];
          pHit->dDiffuse_dS[i].size = 3;
          pHit->dDiffuse_dS[i].value = t.x*t.y*t.z;
        }
      }
      // else error
    }
  }
}