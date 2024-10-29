#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <chrono>

#include "BVH2Common.h"

using uvec3 = uint3;
using LiteMath::M_PI;
using LiteMath::clamp;


bool BVHRT::need_normal()
{
  return m_preset.render_mode == MULTI_RENDER_MODE_LAMBERT_NO_TEX || 
         m_preset.render_mode == MULTI_RENDER_MODE_NORMAL  ||
         m_preset.render_mode == MULTI_RENDER_MODE_PHONG_NO_TEX ||
         m_preset.render_mode == MULTI_RENDER_MODE_LAMBERT ||
         m_preset.render_mode == MULTI_RENDER_MODE_PHONG;
}

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

void BVHRT::IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_Hit *pHit)
{
  uint32_t type = m_geomData[geomId].type;
  const float SDF_BIAS = 0.1f;
  const float tNearSdf = std::max(tNear, SDF_BIAS);
  
  //m_abstractObjectPtrs[geomId]->Intersect(type, ray_pos, ray_dir, tNearSdf, instId, geomId, a_start, a_count, pHit, this);
}

float BVHRT::eval_dist_trilinear(const float values[8], float3 dp)
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

//intersects ray with the particular leaf node of SDF strcuture
//it reqires a 1-to-1 mapping between leaf nodes of SDF and BVH
//like in frame octree, SVS and SBS with multiple BVH nodes per brick
void BVHRT::RayNodeIntersection(uint32_t type, const float3 ray_pos, const float3 ray_dir, 
                                float tNear, uint32_t geomId, uint32_t bvhNodeId, 
                                float values[8], uint32_t &primId, uint32_t &nodeId, float &d, 
                                float &qNear, float &qFar, float2 &fNearFar, float3 &start_q)
{
  float3 min_pos, max_pos;

  if (type == TYPE_SDF_FRAME_OCTREE)
  {
#ifndef DISABLE_SDF_FRAME_OCTREE
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = m_origNodes[bvhNodeId].leftOffset;
    nodeId = primId + m_SdfFrameOctreeRoots[sdfId];
    min_pos = m_origNodes[bvhNodeId].boxMin;
    max_pos = m_origNodes[bvhNodeId].boxMax;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = m_SdfFrameOctreeNodes[nodeId].values[i];

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
  else if (type == TYPE_SDF_FRAME_OCTREE_TEX)
  {
#ifndef DISABLE_SDF_FRAME_OCTREE_TEX
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = m_origNodes[bvhNodeId].leftOffset;
    nodeId = primId + m_SdfFrameOctreeTexRoots[sdfId];
    min_pos = m_origNodes[bvhNodeId].boxMin;
    max_pos = m_origNodes[bvhNodeId].boxMax;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = m_SdfFrameOctreeTexNodes[nodeId].values[i];

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
  else if (type == TYPE_SDF_SVS)
  {
#ifndef DISABLE_SDF_SVS
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = bvhNodeId;
    nodeId = primId + m_SdfSVSRoots[sdfId];

    float px = m_SdfSVSNodes[nodeId].pos_xy >> 16;
    float py = m_SdfSVSNodes[nodeId].pos_xy & 0x0000FFFF;
    float pz = m_SdfSVSNodes[nodeId].pos_z_lod_size >> 16;
    float sz = m_SdfSVSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
    float d_max = 2*1.41421356f/sz;

    min_pos = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
    max_pos = min_pos + 2.0f*float3(1,1,1)/sz;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = -d_max + 2*d_max*(1.0/255.0f)*((m_SdfSVSNodes[nodeId].values[i/4] >> (8*(i%4))) & 0xFF);

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
  else //if (type == TYPE_SDF_SBS)
  {
#ifndef DISABLE_SDF_SBS
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = bvhNodeId; //id of bbox in BLAS
    nodeId = m_SdfSBSRemap[primId + m_geomData[geomId].offset.y].x; //id of node (brick) in SBS
    uint32_t voxelId = m_SdfSBSRemap[primId + m_geomData[geomId].offset.y].y;
    SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
    uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
    uint3 voxelPos = uint3(voxelId/(v_size*v_size), voxelId/v_size%v_size, voxelId%v_size);

    float px = m_SdfSBSNodes[nodeId].pos_xy >> 16;
    float py = m_SdfSBSNodes[nodeId].pos_xy & 0x0000FFFF;
    float pz = m_SdfSBSNodes[nodeId].pos_z_lod_size >> 16;
    float sz = m_SdfSBSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;

    min_pos = float3(-1,-1,-1) + 2.0f*(float3(px,py,pz)/sz + float3(voxelPos)/(sz*header.brick_size));
    max_pos = min_pos + 2.0f*float3(1,1,1)/(sz*header.brick_size);
    float3 size = max_pos - min_pos;

    //TODO: make it works with brick_size > 1
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    uint32_t vals_per_int = 4/header.bytes_per_value; 
    uint32_t bits = 8*header.bytes_per_value;
    uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
    float d_max = 2*1.41421356f/sz;
    float mult = 2*d_max/max_val;
    for (int i=0;i<8;i++)
    {
      uint3 vPos = voxelPos + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint32_t vId = vPos.x*v_size*v_size + vPos.y*v_size + vPos.z;
      values[i] = -d_max + mult*((m_SdfSBSData[v_off + vId/vals_per_int] >> (bits*(vId%vals_per_int))) & max_val);
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
}

void BVHRT::LocalSurfaceIntersection(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                     float values[8], uint32_t nodeId, uint32_t primId, float d, float qNear, 
                                     float qFar, float2 fNearFar, float3 start_q,
                                     CRT_Hit *pHit)
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
    
  #ifndef DISABLE_SDF_FRAME_OCTREE_TEX
    if (type == TYPE_SDF_FRAME_OCTREE_TEX)
    {
      float3 dp = start_q + t * ray_dir;
      
      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[0] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[2] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[4] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[6] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[8] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[10] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[12] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[14];

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[1] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[3] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[5] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[7] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[9] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[11] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[13] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[15];
    }
  #endif
  }
}

void BVHRT::OctreeNodeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                float tNear, uint32_t instId, uint32_t geomId,
                                uint32_t a_start, uint32_t a_count,
                                CRT_Hit *pHit)
{
  float values[8];
  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  RayNodeIntersection(type, ray_pos, ray_dir, tNear, geomId, a_start, /*in */
                      values, primId, nodeId, d, qNear, qFar, fNearFar, start_q); /*out*/

  //fast return if starting point in this exact node or type is not supported
  if (qNear > 0.0f) 
    return;
  
  LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, qNear, qFar, fNearFar, start_q, /*in */
                           pHit); /*out*/
}

void BVHRT::OctreeBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                 float tNear, uint32_t instId, uint32_t geomId,
                                 uint32_t bvhNodeId, uint32_t a_count,
                                 CRT_Hit *pHit)
{
  float values[8];
  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[geomId].offset.x;
  primId = bvhNodeId; //id of bbox in BLAS
  nodeId = primId + m_SdfSBSRoots[sdfId];
  SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
  uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;

  float px = m_SdfSBSNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSBSNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSBSNodes[nodeId].pos_z_lod_size >> 16;
  float sz = m_SdfSBSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
  float sz_inv = 2.0f/sz;
  
  d = 2.0f/(sz*header.brick_size);

  float3 brick_min_pos = float3(-1,-1,-1) + sz_inv*float3(px,py,pz);
  float3 brick_max_pos = brick_min_pos + sz_inv*float3(1,1,1);
  float3 brick_size = brick_max_pos - brick_min_pos;

  float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
  float old_t = pHit->t;
  while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
  {
    float3 hit_pos = ray_pos + brick_fNearFar.x*ray_dir;
    float3 local_pos = (hit_pos - brick_min_pos) * (0.5f*sz*header.brick_size);
    float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size-1e-6f));

    float3 min_pos = brick_min_pos + d*voxelPos;
    float3 max_pos = min_pos + d*float3(1,1,1);
    float3 size = max_pos - min_pos;

    float vmin = 1.0f;

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size*v_size + vPos.y*v_size + vPos.z;
        values[i] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
        //printf("%f\n", values[i]);
        vmin = std::min(vmin, values[i]);
      }
    }
    else
    {
      uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
      uint32_t vals_per_int = 4/header.bytes_per_value; 
      uint32_t bits = 8*header.bytes_per_value;
      uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
      float d_max = 1.41421356f*sz_inv;
      float mult = 2*d_max/max_val;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size*v_size + vPos.y*v_size + vPos.z;
        values[i] = -d_max + mult*((m_SdfSBSData[v_off + vId/vals_per_int] >> (bits*(vId%vals_per_int))) & max_val);
        vmin = std::min(vmin, values[i]);
      }
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)    
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * (0.5f*sz*header.brick_size);
      qFar = (fNearFar.y - fNearFar.x) * (0.5f*sz*header.brick_size);
    
      LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
    }
    
    brick_fNearFar.x += std::max(0.0f, fNearFar.y-brick_fNearFar.x) + 1e-6f;
  }
  
  //ray hit a brick
  if (pHit->t < old_t)
  {
    float3 pos = ray_pos + pHit->t*ray_dir;
    float3 dp = (pos - brick_min_pos)*(0.5f*sz);

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_UV16)
    {   
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;

      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+0] >> 16)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+1] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+2] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+3] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+4] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+5] >> 16)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+6] >> 16)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+7] >> 16));

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+0] & 0xFFFF)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+1] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+2] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+3] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+4] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+5] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+6] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+7] & 0xFFFF));
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_RGB8)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+0] >> 0) & 0xFF, (m_SdfSBSData[t_off+0] >> 8) & 0xFF, (m_SdfSBSData[t_off+0] >> 16) & 0xFF) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+1] >> 0) & 0xFF, (m_SdfSBSData[t_off+1] >> 8) & 0xFF, (m_SdfSBSData[t_off+1] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+2] >> 0) & 0xFF, (m_SdfSBSData[t_off+2] >> 8) & 0xFF, (m_SdfSBSData[t_off+2] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+3] >> 0) & 0xFF, (m_SdfSBSData[t_off+3] >> 8) & 0xFF, (m_SdfSBSData[t_off+3] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+4] >> 0) & 0xFF, (m_SdfSBSData[t_off+4] >> 8) & 0xFF, (m_SdfSBSData[t_off+4] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+5] >> 0) & 0xFF, (m_SdfSBSData[t_off+5] >> 8) & 0xFF, (m_SdfSBSData[t_off+5] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+6] >> 0) & 0xFF, (m_SdfSBSData[t_off+6] >> 8) & 0xFF, (m_SdfSBSData[t_off+6] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+7] >> 0) & 0xFF, (m_SdfSBSData[t_off+7] >> 8) & 0xFF, (m_SdfSBSData[t_off+7] >> 16) & 0xFF);

      color = clamp(floor(color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;

      //printf("color = %f %f %f coords = %f %f\n", color.x, color.y, color.z, pHit->coords[0], pHit->coords[1]);
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size*v_size*v_size;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+0]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+0]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+0]+2]) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+1]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+1]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+1]+2]) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+2]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+2]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+2]+2]) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+3]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+3]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+3]+2]) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+4]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+4]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+4]+2]) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+5]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+5]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+5]+2]) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+6]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+6]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+6]+2]) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+7]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+7]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+7]+2]);

      color = clamp(floor(255.0f*color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
      //printf("color = %f %f %f coords = %f %f\n", color.x, color.y, color.z, pHit->coords[0], pHit->coords[1]);
    }
  }
}

void BVHRT::IntersectAllSdfsInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{

  uint32_t type = m_geomData[geomId].type;
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  float3 min_pos = float3(0,0,0), max_pos = float3(0,0,0);

  switch (type)
  {
#ifndef DISABLE_SDF_GRID
  case TYPE_SDF_GRID:
    sdfId = m_geomData[geomId].offset.x;
    primId = 0;
    min_pos = float3(-1,-1,-1);
    max_pos = float3( 1, 1, 1);
    break;
#endif
#ifndef DISABLE_SDF_OCTREE
  case TYPE_SDF_OCTREE:
    sdfId = m_geomData[geomId].offset.x;
    primId = 0;
    min_pos = float3(-1,-1,-1);
    max_pos = float3( 1, 1, 1);
    break;
#endif
  default:
    break;
  }

  float l = length(ray_dir);
  float3 dir = ray_dir/l;
  SdfHit hit = sdf_sphere_tracing(type, sdfId, min_pos, max_pos, tNear, ray_pos, dir, need_normal());
  if (hit.hit_pos.w > 0)
  {
    float t = length(to_float3(hit.hit_pos)-ray_pos)/l;
    if (t > tNear && t < pHit->t)
    {
      float3 n = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, to_float3(hit.hit_norm)));
      pHit->t         = t;
      pHit->primId    = primId;
      pHit->instId    = instId;
      pHit->geomId    = geomId | (type << SH_TYPE);  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = n.x;
      pHit->coords[3] = n.y;

      if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
        pHit->primId = uint32_t(hit.hit_norm.w);
    }
  }
}

static float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}

#ifndef DISABLE_RF_GRID
int indexGrid(int x, int y, int z, int gridSize) {
    return (x + y * gridSize + z * gridSize * gridSize) * 28;
}

void lerpCellf(const float v0[28], const float v1[28], const float t, float memory[28])
{
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(v0[i], v1[i], t);
}

void BVHRT::lerpCell(const uint idx0, const uint idx1, const float t, float memory[28]) {
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(m_RFGridData[28 * idx0 + i], m_RFGridData[28 * idx1 + i], t);
}

// From Mitsuba 3
void sh_eval_2(const float3 &d, float fout[9])
{
  float x = d.x, y = d.y, z = d.z, z2 = z * z;
  float c0, c1, s0, s1, tmp_a, tmp_b, tmp_c;

  fout[0] = 0.28209479177387814;
  fout[2] = z * 0.488602511902919923;
  fout[6] = z2 * 0.94617469575756008 + -0.315391565252520045;
  c0 = x;
  s0 = y;

  tmp_a = -0.488602511902919978;
  fout[3] = tmp_a * c0;
  fout[1] = tmp_a * s0;
  tmp_b = z * -1.09254843059207896;
  fout[7] = tmp_b * c0;
  fout[5] = tmp_b * s0;
  c1 = x * c0 - y * s0;
  s1 = x * s0 + y * c0;

  tmp_c = 0.546274215296039478;
  fout[8] = tmp_c * c1;
  fout[4] = tmp_c * s1;
}

float eval_sh(float sh[28], float3 rayDir, const int offset)
{
  float sh_coeffs[9];
  sh_eval_2(rayDir, sh_coeffs);

  float sum = 0.0f;
  for (int i = 0; i < 9; i++)
    sum += sh[offset + i] * sh_coeffs[i];

  return sum;
}

void BVHRT::RayGridIntersection(float3 ray_dir, uint gridSize, float3 p, float3 lastP, uint4 ptrs, uint4 ptrs2, float &throughput, float3 &colour)
{
  float3 coords = p * (float)(gridSize);

  int3 nearCoords = clamp((int3)coords, int3(0), int3(gridSize - 1));
  int3 farCoords = clamp((int3)coords + int3(1), int3(0), int3(gridSize - 1));

  float3 lerpFactors = coords - (float3)nearCoords;

  float xy00[28];
  float xy10[28];
  float xy01[28];
  float xy11[28];
  
  lerpCell(ptrs[0], ptrs[1], lerpFactors.x, xy00);
  lerpCell(ptrs[2], ptrs2[0], lerpFactors.x, xy10);
  lerpCell(ptrs[3], ptrs2[2], lerpFactors.x, xy01);
  lerpCell(ptrs2[1], ptrs2[3], lerpFactors.x, xy11);


  float xyz0[28];
  float xyz1[28];
  lerpCellf(xy00, xy10, lerpFactors.y, xyz0);
  lerpCellf(xy01, xy11, lerpFactors.y, xyz1);

  float gridVal[28];
  lerpCellf(xyz0, xyz1, lerpFactors.z, gridVal);

  // relu
  /* if (gridVal[0] < 0.0) */
  /*   gridVal[0] = 0.0; */

  // for (size_t i = 0; i < 28; i++)
    // std::cout << (&grid[indexGrid(nearCoords[0], nearCoords[1], nearCoords[2], gridSize)])[i] << ' ';
  // std::cout << std::endl;

  float dist = 1.0f / (float) gridSize * 4.0f;
  /* float dist = length(p - lastP); */
  /* if (dist > sqrt(3) / (float)gridSize) */
  /*     dist -= ((int)(dist * (float)gridSize) - 1) / (float)gridSize; */

  float tr = clamp(exp(-gridVal[0] * m_RFGridScales[0] * dist), 0.0f, 1.0f);

  // std::cout << tr << ' ' << gridVal[0] << ' ' << length(p - lastP) << ' ' << gridSize << std::endl;

  // float3 RGB = float3(1.0f);
  float3 RGB = float3(sigmoid(eval_sh(gridVal, ray_dir, 1)), sigmoid(eval_sh(gridVal, ray_dir, 10)), sigmoid(eval_sh(gridVal, ray_dir, 19)));
  colour = colour + throughput * (1 - tr) * RGB;
  throughput *= tr;
}

void BVHRT::IntersectRFInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{
  uint32_t type = m_geomData[geomId].type;
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  auto bbox = m_origNodes[a_start];
  float2 smallBox = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), bbox.boxMin, bbox.boxMax);

  float depth = (smallBox.x + (smallBox.y - smallBox.x) * 0.5);
  float3 p = ray_pos + ray_dir * depth;

  float throughput = pHit->coords[0];
  float3 colour = float3(pHit->coords[1], pHit->coords[2], pHit->coords[3]);

  if (m_RFGridFlags[1] == 1) {
    uint4 ptrs;
    ptrs[0] = 8 * a_start;
    ptrs[1] = 8 * a_start + 1;
    ptrs[2] = 8 * a_start + 2;
    ptrs[3] = 8 * a_start + 3;

    uint4 ptrs2;
    ptrs2[0] = 8 * a_start + 4;
    ptrs2[1] = 8 * a_start + 5;
    ptrs2[2] = 8 * a_start + 6;
    ptrs2[3] = 8 * a_start + 7;

    RayGridIntersection(ray_dir, m_RFGridSizes[0], p, float3(0.0f), ptrs, ptrs2, throughput, colour);
  } else
    RayGridIntersection(ray_dir, m_RFGridSizes[0], p, float3(0.0f), m_RFGridPtrs[2 * a_start], m_RFGridPtrs[2 * a_start + 1], throughput, colour);
  
  // std::cout << throughput << std::endl;

  pHit->primId = a_start;
  pHit->geomId = geomId | (type << SH_TYPE);

  pHit->coords[0] = throughput;
  pHit->coords[1] = colour[0];
  pHit->coords[2] = colour[1];
  pHit->coords[3] = colour[2];

  pHit->t = depth;

  /* float3 edge = ray_pos + ray_dir * zNearAndFar.y; */
  /* pHit->adds[0] = length(edge - p); */
  /* pHit->adds[1] = 0.0f; */
  /* pHit->adds[2] = 0.0f; */
  /* pHit->adds[3] = 0.0f; */

  // std::cout << "Mew" << std::endl;
}
#endif

#ifndef DISABLE_GS_PRIMITIVE
float4 QuaternionMultiply(const float4& a, const float4& b) {
    float4 c;

    c.x = a.x * b.x - a.y * b.y - a.z * b.z - a.w * b.w;
    c.y = a.x * b.y + a.y * b.x + a.z * b.w - a.w * b.z;
    c.z = a.x * b.z - a.y * b.w + a.z * b.x + a.w * b.y;
    c.w = a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x;

    return c;
}

float4 QuaternionConjugate(const float4& q) {
    return float4(q.x, -q.y, -q.z, -q.w);
}

float3 RotatePoint(const float3& p, const float4& q) {
    float4 result = QuaternionMultiply(QuaternionMultiply(QuaternionConjugate(q), float4(0.0f, p.x, p.y, p.z)), q);

    return float3(result.y, result.z, result.w);
}

float3 ComputeColorFromSH(
    const int sh_degree,
    const float3& gaussian_pos,
    const float3& ray_pos,
    float4x4& data_r,
    float4x4& data_g,
    float4x4& data_b) {
  // Spherical harmonics coefficients
  const float SH_C0 = 0.28209479177387814f;
  const float SH_C1 = 0.4886025119029199f;
  const float SH_C2[] = {
    1.0925484305920792f,
    -1.0925484305920792f,
    0.31539156525252005f,
    -1.0925484305920792f,
    0.5462742152960396f
  };
  const float SH_C3[] = {
    -0.5900435899266435f,
    2.890611442640554f,
    -0.4570457994644658f,
    0.3731763325901154f,
    -0.4570457994644658f,
    1.445305721320277f,
    -0.5900435899266435f
  };

	float3 dir = gaussian_pos - ray_pos;
	dir = dir / length(dir);

  float3 sh[16];

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      const auto r = data_r[i][j];
      const auto g = data_g[i][j];
      const auto b = data_b[i][j];

      sh[i * 4 + j] = float3(r, g, b);
    }
  }

	float3 result = SH_C0 * sh[0];

	if (sh_degree > 0) {
		float x = dir.x;
		float y = dir.y;
		float z = dir.z;

		result = result - SH_C1 * y * sh[1] +
                      SH_C1 * z * sh[2] -
                      SH_C1 * x * sh[3];

		if (sh_degree > 1) {
			float xx = x * x;
      float yy = y * y;
      float zz = z * z;
			float xy = x * y;
      float yz = y * z;
      float xz = x * z;

			result = result +
				SH_C2[0] * xy * sh[4] +
				SH_C2[1] * yz * sh[5] +
				SH_C2[2] * (2.0f * zz - xx - yy) * sh[6] +
				SH_C2[3] * xz * sh[7] +
				SH_C2[4] * (xx - yy) * sh[8];

			if (sh_degree > 2) {
				result = result +
					SH_C3[0] * y * (3.0f * xx - yy) * sh[9] +
					SH_C3[1] * xy * z * sh[10] +
					SH_C3[2] * y * (4.0f * zz - xx - yy) * sh[11] +
					SH_C3[3] * z * (2.0f * zz - 3.0f * xx - 3.0f * yy) * sh[12] +
					SH_C3[4] * x * (4.0f * zz - xx - yy) * sh[13] +
					SH_C3[5] * z * (xx - yy) * sh[14] +
					SH_C3[6] * x * (xx - 3.0f * yy) * sh[15];
			}
		}
	}

	result += 0.5f;

	return max(result, float3(0.0f));
}

void BVHRT::CompressGaussianBuffer(CRT_Hit* hit) {
    const uint32_t sh_degree = 3;

    for (uint32_t i = 0; i < hit->gaussian_buffer_size; ++i) {
        const auto gaussian_index = hit->gaussian_index_buffer[i];
        const auto intersection = hit->ray_pos + hit->gaussian_tau_buffer[i] * hit->ray_dir;

        const auto mean = float3(
            m_gs_data[gaussian_index][0][0],
            m_gs_data[gaussian_index][0][1],
            m_gs_data[gaussian_index][0][2]);

        const auto opacity = sigmoid(m_gs_data[gaussian_index][0][3]);
        auto conic_3d = m_gs_conic_3d[gaussian_index];

        const auto power_a = (
            conic_3d[0][0] * (intersection - mean).x * (intersection - mean).x +
            conic_3d[1][1] * (intersection - mean).y * (intersection - mean).y +
            conic_3d[2][2] * (intersection - mean).z * (intersection - mean).z);

        const auto power_b = (
            conic_3d[0][1] * (intersection - mean).x * (intersection - mean).y +
            conic_3d[0][2] * (intersection - mean).x * (intersection - mean).z +
            conic_3d[1][2] * (intersection - mean).y * (intersection - mean).z);

        const auto power = -0.5f * power_a - power_b;

        if (power > 0.0f) {
            continue;
        }

        const auto alpha = min(0.99f, opacity * float(exp(power)));

        if (alpha < 1.0f / 255.0f) {
            continue;
        }

        const auto transparency = hit->coords[0] * (1.0f - alpha);

        if (transparency < 0.0001f) { 
            continue;
        }

        const auto weight = alpha * hit->coords[0];

        const auto diffuse_color = ComputeColorFromSH(
            sh_degree,
            mean,
            hit->ray_pos,
            m_gs_data_r[gaussian_index],
            m_gs_data_g[gaussian_index],
            m_gs_data_b[gaussian_index]);

        hit->coords[1] += diffuse_color.x * weight;
        hit->coords[2] += diffuse_color.y * weight;
        hit->coords[3] += diffuse_color.z * weight;

        hit->coords[0] = transparency;
    }

    hit->gaussian_buffer_size = 0;
}

void BVHRT::InsertToGaussianBuffer(CRT_Hit* hit, uint32_t index, float distance, float tau) {
    const uint32_t max_buffer_size = 2048;

    if (hit->gaussian_buffer_size == max_buffer_size) {
        CompressGaussianBuffer(hit);
    }

    uint32_t size = hit->gaussian_buffer_size;
    uint32_t position = 0;

    while (position < size && hit->gaussian_distance_buffer[position] < distance) {
        position += 1;
    }

    for (uint32_t i = size; i > position; --i) {
        hit->gaussian_index_buffer[i] = hit->gaussian_index_buffer[i - 1];
        hit->gaussian_distance_buffer[i] = hit->gaussian_distance_buffer[i - 1];
        hit->gaussian_tau_buffer[i] = hit->gaussian_tau_buffer[i - 1];
    }

    hit->gaussian_index_buffer[position] = index;
    hit->gaussian_distance_buffer[position] = distance;
    hit->gaussian_tau_buffer[position] = tau;

    hit->gaussian_buffer_size += 1;
}

void BVHRT::IntersectGSInLeaf(const float3& ray_pos, const float3& ray_dir,
                              float tNear, uint32_t instId,
                              uint32_t geomId, uint32_t a_start,
                              uint32_t a_count, CRT_Hit* pHit) {
      pHit->ray_pos = ray_pos;
      pHit->ray_dir = ray_dir;

      const auto mean = float3(
        m_gs_data[a_start][0][0],
        m_gs_data[a_start][0][1],
        m_gs_data[a_start][0][2]);

      const auto opacity = sigmoid(m_gs_data[a_start][0][3]);
      auto conic_3d = m_gs_conic_3d[a_start];

      // START -- this block of code does the same thing as the isocahedrons
      const auto scale = float3(
        exp(m_gs_data[a_start][2][0]),
        exp(m_gs_data[a_start][2][1]),
        exp(m_gs_data[a_start][2][2]));

      const auto rotation = normalize(float4(
        m_gs_data[a_start][1][0],
        m_gs_data[a_start][1][1], 
        m_gs_data[a_start][1][2],
        m_gs_data[a_start][1][3]));

      auto origin = ray_pos - mean;
      auto direction = ray_dir;

      origin = RotatePoint(origin, rotation);
      direction = RotatePoint(direction, rotation);

      // we need to intersect 99.73% of the Gaussian according to the 68-95-99.7 rule
      origin = origin / scale / 3.0f;
      direction = direction / scale / 3.0f;

      const auto normalized_direction = normalize(direction);
      const auto b2 = dot(origin, normalized_direction);
      const auto fd = origin - b2 * normalized_direction;

      const auto discriminant = 1.0f - dot(fd, fd);

      if (discriminant < 0.0f) {
        return;
      }
      // END -- this block of code does the same thing as the isocahedrons

      // first method
      {
        const auto c = dot(origin, origin) - 1.0f;
        const auto sqrt_d = sqrt(discriminant);

        const auto q = (b2 < 0.0f) ? sqrt_d - b2 : 0.0f - sqrt_d - b2;

        const auto t1 = c / q / length(direction);
        const auto t2 = q / length(direction);

        const auto tau = (t1 + t2) / 2.0f;
        const auto intersection = ray_pos + tau * ray_dir;

        pHit->primId = a_start;

        InsertToGaussianBuffer(
            pHit,
            a_start,
            length(mean - ray_pos),
            tau);

        return;
      }

      // second method – from the gaussiantracer.github.io/res/3DGRT_Anonymous_LQ.pdf
      {
        float4x4 S;

        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            S[i][j] = 0.0f;
          }
        }

        S[0][0] = scale.x;
        S[1][1] = scale.y;
        S[2][2] = scale.z;
        S[3][3] = 1.0f;

        const auto r = rotation.x;
        const auto x = rotation.y;
        const auto y = rotation.z;
        const auto z = rotation.w;

        float4x4 R;

        R[0][0] = 1.0f - 2.0f * (y * y + z * z);
        R[0][1] = 2.0f * (x * y - r * z);
        R[0][2] = 2.0f * (x * z + r * y);
        R[0][3] = 0.0f;
        R[1][0] = 2.0f * (x * y + r * z);
        R[1][1] = 1.0f - 2.0f * (x * x + z * z);
        R[1][2] = 2.0f * (y * z - r * x);
        R[1][3] = 0.0f;
        R[2][0] = 2.0f * (x * z - r * y);
        R[2][1] = 2.0f * (y * z + r * x);
        R[2][2] = 1.0f - 2.0f * (x * x + y * y);
        R[2][3] = 0.0f;
        R[3][0] = 0.0f;
        R[3][1] = 0.0f;
        R[3][2] = 0.0f;
        R[3][3] = 1.0f;

        const auto M = inverse4x4(S) * transpose(R);

        const auto o = ray_pos - mean;
        const auto d = ray_dir;

        const auto oo = M * float4(o.x, o.y, o.z, 0.0f);
        const auto dd = M * float4(d.x, d.y, d.z, 0.0f);

        const auto tau = -dot(oo, dd) / dot(dd, dd);
        const auto intersection = ray_pos + tau * ray_dir;

        pHit->primId = a_start;

        InsertToGaussianBuffer(
            pHit,
            a_start,
            length(mean - ray_pos),
            tau);

        return;
      }
}
#endif

SdfHit BVHRT::sdf_sphere_tracing(uint32_t type, uint32_t sdf_id, const float3 &min_pos, const float3 &max_pos,
                                 float tNear, const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  hit.hit_pos = float4(0,0,0,-1);
  hit.hit_norm = float4(1,0,0,0);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = std::max(tNear, tNear_tFar.x);
  float tFar = tNear_tFar.y;
  if (t > tFar)
    return hit;
  
  int iter = 0;
  float d = eval_distance_sdf(type, sdf_id, pos + t * dir);
  while (iter < 1000 && d > EPS && t < tFar)
  {
    t += d + EPS;
    d = eval_distance_sdf(type, sdf_id, pos + t * dir);
    iter++;
  }

  if (d > EPS)
    return hit;

  float3 p0 = pos + t * dir;
  float3 norm = float3(0,0,1);
  if (need_norm)
  {
    const float h = 0.001;
    float ddx = (eval_distance_sdf(type, sdf_id, p0 + float3(h, 0, 0)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (eval_distance_sdf(type, sdf_id, p0 + float3(0, h, 0)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (eval_distance_sdf(type, sdf_id, p0 + float3(0, 0, h)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(0, 0, -h))) /
                (2 * h);

    norm = normalize(float3(ddx, ddy, ddz));
    // fprintf(stderr, "st %d (%f %f %f)\n", iter, surface_normal->x, surface_normal->y, surface_normal->z);
  }
  // fprintf(stderr, "st %d (%f %f %f)", iter, p0.x, p0.y, p0.z);
  hit.hit_pos = to_float4(p0, 1);
  hit.hit_norm = to_float4(norm, float(iter));
  return hit;
}

float BVHRT::eval_distance_sdf(uint32_t type, uint32_t sdf_id, float3 pos)
{
  float val = 1000;
  switch (type)
  {
#ifndef DISABLE_SDF_GRID
  case TYPE_SDF_GRID:
    val = eval_distance_sdf_grid(sdf_id, pos);
    break;
#endif
#ifndef DISABLE_SDF_OCTREE
  case TYPE_SDF_OCTREE:
    val = eval_distance_sdf_octree(sdf_id, pos, 1000);
    break;
#endif
  default:
    break;
  }
  return val;
}

#ifndef DISABLE_SDF_GRID
float BVHRT::eval_distance_sdf_grid(uint32_t grid_id, float3 pos)
{
  uint32_t off = m_SdfGridOffsets[grid_id];
  uint3 size = m_SdfGridSizes[grid_id];

  //bbox for grid is a unit cube
  float3 grid_size_f = float3(size);
  float3 vox_f = grid_size_f*((pos-float3(-1,-1,-1))/float3(2,2,2)) - float3(0.5, 0.5, 0.5);
  vox_f = min(max(vox_f, float3(0.0f)), grid_size_f - float3(1e-5f));
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
#endif

#ifndef DISABLE_SDF_OCTREE
bool BVHRT::is_leaf(uint32_t offset)
{
  return (offset == 0) || ((offset & (1u<<31u)) > 0);
}

float BVHRT::eval_distance_sdf_octree(uint32_t octree_id, float3 position, uint32_t max_level)
{
  switch (m_preset.sdf_octree_sampler)
  {
  case SDF_OCTREE_SAMPLER_MIPSKIP_3X3:
    return sdf_octree_sample_mipskip_3x3(octree_id, position, max_level);
    break;
  case SDF_OCTREE_SAMPLER_MIPSKIP_CLOSEST:
    return sdf_octree_sample_mipskip_closest(octree_id, position, max_level);
    break;
    case SDF_OCTREE_SAMPLER_CLOSEST:
    return sdf_octree_sample_closest(octree_id, position, max_level);
    break;
  default:
    return 1e6;
    break;
  }
}

float BVHRT::sdf_octree_sample_mipskip_closest(uint32_t octree_id, float3 position, uint32_t max_level)
{
  float3 n_pos = clamp(0.5f*(position + 1.0f), 0.0f, 1.0f);//position in current neighborhood
  float d = 1;//size of current neighborhood
  float n_distances[8];
  uint32_t n_indices[8]; //0 index means that it is "virtual" node
  uint32_t non_leaf_nodes = 0; //how many nodes in neighborhood have children

  float prev_n_distances[8];
  uint32_t prev_n_indices[8]; //0 index means that it is "virtual" node

  //start with root's childer as a neighborhood
  uint32_t root_id = m_SdfOctreeRoots[octree_id];
  uint32_t r_idx = m_SdfOctreeNodes[root_id].offset;
  for (int i=0;i<8;i++)
  {
    n_distances[i] = m_SdfOctreeNodes[r_idx+i].value;
    n_indices[i] = r_idx+i;
    non_leaf_nodes += uint32_t(!is_leaf(m_SdfOctreeNodes[r_idx+i].offset));
  }

  int level = 1;
  while (non_leaf_nodes > 0 && level <= max_level)
  {
    for (int i=0;i<8;i++)
    {
      prev_n_distances[i] = n_distances[i];
      prev_n_indices[i] = n_indices[i];
    }
    //go 1 level deeper every iteration
    non_leaf_nodes = 0;

    uint3 idx8 = uint3(8.0f*fract(n_pos));
    uvec3 pidx[2], chidx[2];
    float3 n_pos_sh;
    for (int i=0;i<3;i++)
    {
      if (idx8[i] == 0)
        { pidx[0][i] = 0; chidx[0][i] = 0; pidx[1][i] = 0; chidx[1][i] = 0; n_pos_sh[i] = 0;}
      else if (idx8[i] <= 2)
        { pidx[0][i] = 0; chidx[0][i] = 0; pidx[1][i] = 0; chidx[1][i] = 1; n_pos_sh[i] = 0;}
      else if (idx8[i] <= 4)
        { pidx[0][i] = 0; chidx[0][i] = 1; pidx[1][i] = 1; chidx[1][i] = 0; n_pos_sh[i] = 0.25;}
      else if (idx8[i] <= 6)
        { pidx[0][i] = 1; chidx[0][i] = 0; pidx[1][i] = 1; chidx[1][i] = 1; n_pos_sh[i] = 0.5;}
      else //if (idx8[i] == 7)
        { pidx[0][i] = 1; chidx[0][i] = 1; pidx[1][i] = 1; chidx[1][i] = 1; n_pos_sh[i] = 0.5;}
    }
    
    //create new neighborhood
    for (uint32_t i=0;i<8;i++)
    {
      uint3 n_idx((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint3 cur_pidx = uint3(pidx[n_idx[0]][0], pidx[n_idx[1]][1], pidx[n_idx[2]][2]);
      uint3 cur_chidx = uint3(chidx[n_idx[0]][0], chidx[n_idx[1]][1], chidx[n_idx[2]][2]);

      uint32_t p_index = 4*cur_pidx.x + 2*cur_pidx.y + cur_pidx.z;
      if (prev_n_indices[p_index] > 0 &&                //p_index is a real node
          !is_leaf(m_SdfOctreeNodes[prev_n_indices[p_index]].offset))    //p_index has children
      {
        uint32_t ch_index = m_SdfOctreeNodes[prev_n_indices[p_index]].offset + 4*cur_chidx.x + 2*cur_chidx.y + cur_chidx.z;
        n_distances[i] = m_SdfOctreeNodes[ch_index].value;
        n_indices[i] = ch_index;      
        non_leaf_nodes += uint32_t(!is_leaf(m_SdfOctreeNodes[ch_index].offset));
      }
      else                                              //p_index is a leaf node
      {
        n_distances[i] = prev_n_distances[p_index];
        n_indices[i] = 0;   
      }
    }

    n_pos = fract(2.0f*(n_pos - n_pos_sh));
    d /= 2;

    level++;
  }

  //bilinear sampling
  float3 dp = clamp(2.0f*n_pos - float3(0.5, 0.5, 0.5), 0.0f, 1.0f);
  return (1-dp.x)*(1-dp.y)*(1-dp.z)*n_distances[0] + 
         (1-dp.x)*(1-dp.y)*(  dp.z)*n_distances[1] + 
         (1-dp.x)*(  dp.y)*(1-dp.z)*n_distances[2] + 
         (1-dp.x)*(  dp.y)*(  dp.z)*n_distances[3] + 
         (  dp.x)*(1-dp.y)*(1-dp.z)*n_distances[4] + 
         (  dp.x)*(1-dp.y)*(  dp.z)*n_distances[5] + 
         (  dp.x)*(  dp.y)*(1-dp.z)*n_distances[6] + 
         (  dp.x)*(  dp.y)*(  dp.z)*n_distances[7];
}

float BVHRT::sdf_octree_sample_closest(uint32_t octree_id, float3 position, uint32_t max_level)
{
  float3 pos = clamp(0.5f*(position + 1.0f), 0.0f, 1.0f);
  uint32_t idx = m_SdfOctreeRoots[octree_id];
  float d = 1;
  uint32_t level = 0;
  float3 p = float3(0,0,0);
  while (m_SdfOctreeNodes[idx].offset != 0 && level <= max_level)
  {
    float3 pindf = pos/d - p;
    uint32_t ch_index = 4*uint32_t(pindf.x >= 0.5) + 2*uint32_t(pindf.y >= 0.5) + uint32_t(pindf.z >= 0.5);
    //printf("%u pindf %f %f %f %u\n",idx, pindf.x, pindf.y, pindf.z, ch_index);
    idx = m_SdfOctreeNodes[idx].offset + ch_index;
    d = d/2;
    p = 2*p + float3((ch_index & 4) >> 2, (ch_index & 2) >> 1, ch_index & 1);
    level++;
  }
  //printf("\n");
  //printf("%u last pindf \n",idx);

  return m_SdfOctreeNodes[idx].value;
}

float BVHRT::sdf_octree_sample_mipskip_3x3(uint32_t octree_id, float3 position, uint32_t max_level)
{
  const uint32_t X_L = 1<<0;
  const uint32_t X_H = 1<<1;
  const uint32_t Y_L = 1<<2;
  const uint32_t Y_H = 1<<3;
  const uint32_t Z_L = 1<<4;
  const uint32_t Z_H = 1<<5;

  uint32_t CENTER = 9 + 3 + 1;
  float EPS = 1e-6;
  SDONeighbor neighbors[27];
  SDONeighbor new_neighbors[27];


  float3 n_pos = clamp(0.5f*(position + 1.0f), EPS, 1.0f-EPS);//position in current neighborhood
  float d = 1;//size of current neighborhood
  uint32_t level = 0;
  uint32_t root_id = m_SdfOctreeRoots[octree_id];
  for (int i=0;i<27;i++)
  {
    neighbors[i].node = m_SdfOctreeNodes[root_id];
    neighbors[i].overshoot = 0;
    if (i/9 == 0) neighbors[i].overshoot |= X_L;
    else if (i/9 == 2) neighbors[i].overshoot |= X_H;
    if (i/3%3 == 0) neighbors[i].overshoot |= Y_L;
    else if (i/3%3 == 2) neighbors[i].overshoot |= Y_H;
    if (i%3 == 0) neighbors[i].overshoot |= Z_L;
    else if (i%3 == 2) neighbors[i].overshoot |= Z_H;
  }
  neighbors[CENTER].overshoot = 0;

  while (!is_leaf(neighbors[CENTER].node.offset) && level < max_level)
  {
    int3 ch_shift = int3(n_pos.x >= 0.5, n_pos.y >= 0.5, n_pos.z >= 0.5);

    for (int i=0;i<27;i++)
    {
      int3 n_offset = int3(i/9, i/3%3, i%3); //[0,2]^3
      int3 p_idx = (n_offset + ch_shift + 1) / 2;
      int3 ch_idx = (n_offset + ch_shift + 1) - 2*p_idx;
      uint32_t p_offset = 9*p_idx.x + 3*p_idx.y + p_idx.z;
    
      if (is_leaf(neighbors[p_offset].node.offset)) //resample
      {
        float3 rs_pos = 0.5f*float3(2*p_idx + ch_idx) - 1.0f + 0.25f;//in [-1,2]^3

        //sample neighborhood
        float3 qx = clamp(float3(0.5-rs_pos.x,std::min(0.5f + rs_pos.x, 1.5f - rs_pos.x),-0.5+rs_pos.x),0.0f,1.0f);
        float3 qy = clamp(float3(0.5-rs_pos.y,std::min(0.5f + rs_pos.y, 1.5f - rs_pos.y),-0.5+rs_pos.y),0.0f,1.0f);
        float3 qz = clamp(float3(0.5-rs_pos.z,std::min(0.5f + rs_pos.z, 1.5f - rs_pos.z),-0.5+rs_pos.z),0.0f,1.0f);

        float res = 0.0;
        for (int i=0;i<3;i++)
          for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
              res += qx[i]*qy[j]*qz[k]*neighbors[9*i + 3*j + k].node.value;
        //sample neighborhood end

        new_neighbors[i].node.value = res;
        new_neighbors[i].node.offset = 0;
        new_neighbors[i].overshoot = 0;
      }
      else if (neighbors[p_offset].overshoot == 0) //pick child node
      {
        uint32_t ch_offset = 4*ch_idx.x + 2*ch_idx.y + ch_idx.z;
        uint32_t off = neighbors[p_offset].node.offset;
        new_neighbors[i].node = m_SdfOctreeNodes[off + ch_offset];
        new_neighbors[i].overshoot = 0;
      }
      else //pick child node, but mind the overshoot
      {
        /**/
        int3 ch_idx_overshoot = ch_idx;
        uint32_t osh = neighbors[p_offset].overshoot;
        uint32_t new_osh = 0;
        if (((osh&X_L) > 0) && p_idx.x == 0) 
          {ch_idx_overshoot.x = 0; new_osh |= X_L; }
        else if (((osh&X_H) > 0) && p_idx.x == 2) 
          {ch_idx_overshoot.x = 1; new_osh |= X_H; }
        if (((osh&Y_L) > 0) && p_idx.y == 0) 
          {ch_idx_overshoot.y = 0; new_osh |= Y_L; }
        else if (((osh&Y_H) > 0) && p_idx.y == 2) 
          {ch_idx_overshoot.y = 1; new_osh |= Y_H; }
        if (((osh&Z_L) > 0) && p_idx.z == 0) 
          {ch_idx_overshoot.z = 0; new_osh |= Z_L; }
        else if (((osh&Z_H) > 0) && p_idx.z == 2) 
          {ch_idx_overshoot.z = 1; new_osh |= Z_H; }

        uint32_t ch_offset = 4*ch_idx_overshoot.x + 2*ch_idx_overshoot.y + ch_idx_overshoot.z;
        uint32_t off = neighbors[p_offset].node.offset;
        new_neighbors[i].node = m_SdfOctreeNodes[off + ch_offset];
        new_neighbors[i].overshoot = new_osh;
      }
    }

    for (int i=0;i<27;i++)
      neighbors[i] = new_neighbors[i];

    n_pos = fract(2.0f*(n_pos - 0.5f*float3(ch_shift)));
    d /= 2;
    level++;
  }

  //sample neighborhood
  float3 qx = clamp(float3(0.5-n_pos.x,std::min(0.5f + n_pos.x, 1.5f - n_pos.x),-0.5+n_pos.x),0.0f,1.0f);
  float3 qy = clamp(float3(0.5-n_pos.y,std::min(0.5f + n_pos.y, 1.5f - n_pos.y),-0.5+n_pos.y),0.0f,1.0f);
  float3 qz = clamp(float3(0.5-n_pos.z,std::min(0.5f + n_pos.z, 1.5f - n_pos.z),-0.5+n_pos.z),0.0f,1.0f);

  float res = 0.0;
  for (int i=0;i<3;i++)
    for (int j=0;j<3;j++)
      for (int k=0;k<3;k++)
        res += qx[i]*qy[j]*qz[k]*neighbors[9*i + 3*j + k].node.value;
  return res;
  //sample neighborhood end
}
#endif

#ifndef DISABLE_SDF_FRAME_OCTREE
float BVHRT::eval_distance_sdf_frame_octree(uint32_t octree_id, float3 position)
{
  float3 pos = clamp(0.5f*(position + 1.0f), 0.0f, 1.0f);
  uint32_t idx = m_SdfFrameOctreeRoots[octree_id];
  float d = 1;
  float3 p = float3(0,0,0);
  float3 dp = pos;
  while (m_SdfFrameOctreeNodes[idx].offset != 0)
  {
    uint32_t ch_index = 4*uint32_t(dp.x >= 0.5) + 2*uint32_t(dp.y >= 0.5) + uint32_t(dp.z >= 0.5);
    //printf("%u dp %f %f %f %u\n",idx, dp.x, dp.y, dp.z, ch_index);
    idx = m_SdfFrameOctreeNodes[idx].offset + ch_index;
    d = d/2;
    p = 2*p + float3((ch_index & 4) >> 2, (ch_index & 2) >> 1, ch_index & 1);
    dp = pos/d - p;
  }
  //printf("\n");
  //printf("%u last dp \n",idx);

  //bilinear sampling
  return (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[0] + 
         (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[1] + 
         (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[2] + 
         (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[3] + 
         (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[4] + 
         (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[5] + 
         (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[6] + 
         (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[7];
}
#endif

void BVHRT::IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                        float tNear, uint32_t instId, uint32_t geomId,
                                        uint32_t a_start, uint32_t a_count,
                                        CRT_Hit *pHit)
{
  const uint2 a_geomOffsets = m_geomData[geomId].offset;

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
      pHit->t = t;
      pHit->primId = triId;
      pHit->instId = instId;
      pHit->geomId = geomId | (TYPE_MESH_TRIANGLE << SH_TYPE);
      pHit->coords[0] = u;
      pHit->coords[1] = v;

      if (need_normal())
      {
        float3 n = float3(1,0,0);
        if (m_preset.mesh_normal_mode == MESH_NORMAL_MODE_GEOMETRY)
        {
          n = cross(edge1, edge2);
        }
        else if (m_preset.mesh_normal_mode == MESH_NORMAL_MODE_VERTEX)
        {
          n = to_float3(m_vertNorm[a_geomOffsets.y + A] * (1.0f - u - v) + m_vertNorm[a_geomOffsets.y + B] * v + u * m_vertNorm[a_geomOffsets.y + C]);
        }

        n = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, n));

        pHit->coords[2] = n.x;
        pHit->coords[3] = n.y;
      }
      else
      {
        pHit->coords[2] = 0;
        pHit->coords[3] = 0;
      }
    }
  }
}

void BVHRT::BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear,
                                uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                CRT_Hit* pHit)
{
  const uint32_t bvhOffset = m_geomData[geomId].bvhOffset;

  uint32_t stack[STACK_SIZE];
  int top = 0;
  uint32_t leftNodeOffset = 0;

  const float3 rayDirInv = SafeInverse(ray_dir);
  while (top >= 0 && !(stopOnFirstHit && pHit->primId != uint32_t(-1)))
  {
#ifndef DISABLE_RF_GRID
    if (m_RFGridFlags.size() > 0 && pHit->coords[0] <= 0.01f)
      break;
#endif

    while (top >= 0 && ((leftNodeOffset & LEAF_BIT) == 0))
    {
      const BVHNodePair fatNode = m_allNodePairs[bvhOffset + leftNodeOffset];

      const uint32_t node0_leftOffset = fatNode.left.leftOffset;
      const uint32_t node1_leftOffset = fatNode.right.leftOffset;

      const float2 tm0 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.left.boxMin, fatNode.left.boxMax);
      const float2 tm1 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.right.boxMin, fatNode.right.boxMax);

#ifndef DISABLE_RF_GRID
      const bool hitChild0 = (tm0.x <= tm0.y) && (tm0.y >= tNear) && (tm0.x <= pHit->t || !stopOnFirstHit);
      const bool hitChild1 = (tm1.x <= tm1.y) && (tm1.y >= tNear) && (tm1.x <= pHit->t || !stopOnFirstHit);
#else
      const bool hitChild0 = (tm0.x <= tm0.y) && (tm0.y >= tNear) && (tm0.x <= pHit->t);
      const bool hitChild1 = (tm1.x <= tm1.y) && (tm1.y >= tNear) && (tm1.x <= pHit->t);
#endif
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
      CRT_LeafInfo leafInfo;
      leafInfo.aabbId = EXTRACT_START(leftNodeOffset);
      leafInfo.instId = instId;

      const float SDF_BIAS = 0.1f;
      const float tNearSdf = std::max(tNear, SDF_BIAS);
  
      m_abstractObjectPtrs[geomId]->Intersect( to_float4(ray_pos, tNearSdf), to_float4(ray_dir, 1e9f), leafInfo, pHit, this);
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
  const float3 rayDirInv = SafeInverse(to_float3(dirAndFar));

  CRT_Hit hit;
  hit.t      = dirAndFar.w;
  hit.primId = uint32_t(-1);
  hit.instId = uint32_t(-1);
  hit.geomId = uint32_t(-1);
  hit.coords[0] = 1.0f;
  hit.coords[1] = 0.0f;
  hit.coords[2] = 0.0f;
  hit.coords[3] = 0.0f;

#ifndef DISABLE_GS_PRIMITIVE
  hit.gaussian_buffer_size = 0;
#endif

  {
    uint32_t nodeIdx = 0;
    do
    {
#ifndef DISABLE_RF_GRID
      if (m_RFGridFlags.size() > 0 && hit.coords[0] <= 0.01f)
        break;
#endif

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
    
        BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, &hit);
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

CRT_Hit BVHRT::RayQuery_NearestHitGS(float4 posAndNear, float4 dirAndFar) {
    CRT_Hit hit = RayQuery_NearestHit(posAndNear, dirAndFar);

#ifndef DISABLE_GS_PRIMITIVE
    CompressGaussianBuffer(&hit);
#endif

    return hit;
}

bool BVHRT::RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar)
{
  dirAndFar.w *= -1.0f;
  CRT_Hit hit = RayQuery_NearestHit(posAndNear, dirAndFar);
  return (hit.geomId != uint32_t(-1));
}
