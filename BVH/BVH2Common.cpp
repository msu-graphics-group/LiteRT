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

//Octahedral Normal Vectors (ONV) encoding https://jcgt.org/published/0003/02/01/
float2 BVHRT::encode_normal(float3 v)
{
  float2 p = float2(v.x, v.y) * (1.0f / (std::abs(v.x) + std::abs(v.y) + std::abs(v.z)));
  float2 signNotZero = float2((p.x >= 0.0f) ? +1.0f : -1.0f, (p.y >= 0.0f) ? +1.0f : -1.0f);
  return (v.z <= 0.0f) ? ((1.0f - abs(float2(p.y, p.x))) * signNotZero) : p;
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
//like in frame octree or SVS
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
    float d_max = 2*1.73205081f/sz;

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
}

void BVHRT::LocalSurfaceIntersection(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                     #ifdef USE_TRICUBIC 
                                        float values[64]
                                      #else
                                        float values[8]
                                      #endif
                                     , uint32_t nodeId, uint32_t primId, float d, float qNear, 
                                     float qFar, float2 fNearFar, float3 start_q,
                                     CRT_Hit *pHit)
{
  const float EPS = 1e-6f;
  float d_inv = 1.0f / d;
  float t = qNear;
  bool hit = false;
  unsigned iter = 0;

  float start_dist = 10000;
  float start_sign = 1;

  #ifdef USE_TRICUBIC
    float point[3] = {(start_q + t * ray_dir).x, (start_q + t * ray_dir).y, (start_q + t * ray_dir).z};
    start_dist = tricubicInterpolation(values, point);
    start_sign = sign(start_dist);
    start_dist *= start_sign;
  #else
    start_dist = eval_dist_trilinear(values, start_q + t * ray_dir);
    start_sign = sign(start_dist);
    start_dist *= start_sign;
  #endif

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

      #ifdef USE_TRICUBIC
        float point[3] = {(start_q + t * ray_dir).x, (start_q + t * ray_dir).y, (start_q + t * ray_dir).z};
        dist = start_sign * tricubicInterpolation(values, point);
      #else
        dist = start_sign * eval_dist_trilinear(values, start_q + t * ray_dir);
      #endif
      
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
          iter++;
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
        dist = start_sign*eval_dist_trilinear(values, start_q + t * ray_dir);
        pp = start_q + t * ray_dir;
        iter++;
      }
      hit = (dist <= EPS);
     
    }
  }

  float tReal = fNearFar.x + d * t;


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
      #ifndef USE_TRICUBIC
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

      norm = start_sign*normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
      #else
      float3 p0 = start_q + t * ray_dir;
      const float h = 0.001;

      float p1[3] = {(p0 + float3(h, 0, 0)).x, (p0).y, (p0).z};
      float p2[3] = {(p0 + float3(-h, 0, 0)).x, p0.y, p0.z};
      
      float ddx = (tricubicInterpolation(values, p1) -
                   tricubicInterpolation(values, p2)) /
                  (2 * h);

      float p3[3] = {p0.x, (p0 + float3(0, h, 0)).y, p0.z};
      float p4[3] = {p0.x, (p0 + float3(0, -h, 0)).y, p0.z};
      float ddy = (tricubicInterpolation(values, p3) -
                   tricubicInterpolation(values, p4)) /
                  (2 * h);

      float p5[3] = {p0.x, p0.y, (p0 + float3(0, 0, h)).z};
      float p6[3] = {p0.x, p0.y, (p0 + float3(0, 0, -h)).z};

      float ddz = (tricubicInterpolation(values, p5) -
                   tricubicInterpolation(values, p6)) /
                  (2 * h);

      norm = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
      #endif
    }
    
    float2 encoded_norm = encode_normal(norm);

    pHit->t = tReal;
    pHit->primId = primId;
    pHit->instId = instId;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->coords[0] = 0;
    pHit->coords[1] = 0;
    pHit->coords[2] = encoded_norm.x;
    pHit->coords[3] = encoded_norm.y;

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

//transition from 0 (x=edge0) to 1 (x=edge1)
static float step(float edge0, float edge1, float x)
{
  float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t;
}


float BVHRT::load_distance_values(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, 
    #ifdef USE_TRICUBIC 
      float values[64]
    #else
      float values[8]
    #endif
)
{
  float vmin = 1e6f;
#ifndef DISABLE_SDF_SBS
  if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
      header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    
    #ifdef USE_TRICUBIC
      int3 p0 = int3(voxelPos) - 1;

      for (int x = 0; x < 4; x++)
      {
        for (int y = 0; y < 4; y++)
        {
          for (int z = 0; z < 4; z++)
          {
            int3 vPos = p0 + int3(x, y, z);
            uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad); 
            values[16 * z + 4 * y + x] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
            vmin = std::min(vmin, values[16 * z + 4 * y + x]);
          }
        }
      }
    #else
      //  (000) (001) (010) (011) (100) (101) (110) (111)

      for (int i = 0; i < 8; i++)
      {
        uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);
        values[i] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
        // printf("%f\n", values[i]);
        vmin = std::min(vmin, values[i]);
      }

      // for (int x = 0; x < 2; x++)
      // {
      //   for (int y = 0; y < 2; y++)
      //   {
      //     for (int z = 0; z < 2; z++)
      //     {
      //       uint3 vPos = uint3(voxelPos) + uint3(z, y, x);
      //       uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);
      //       values[4 * z + 2 * y + x] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
      //       vmin = std::min(vmin, values[4 * z + 2 * y + x]);
      //     }
      //   }
      // }
    #endif
  }
  else
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    uint32_t vals_per_int = 4 / header.bytes_per_value;
    uint32_t bits = 8 * header.bytes_per_value;
    uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
    float d_max = 1.73205081f * sz_inv;
    float mult = 2 * d_max / max_val;
    for (int i = 0; i < 8; i++)
    {
      uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);;
      values[i] = -d_max + mult * ((m_SdfSBSData[v_off + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
      vmin = std::min(vmin, values[i]);
    }
  }
#endif
  return vmin;
}

void BVHRT::OctreeBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                 float tNear, uint32_t instId, uint32_t geomId,
                                 uint32_t bvhNodeId, uint32_t a_count,
                                 CRT_Hit *pHit)
{
#ifndef DISABLE_SDF_SBS
  #ifdef USE_TRICUBIC
  float values[64];
  #else
  float values[8];
  #endif

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

    float vmin = load_distance_values(nodeId, voxelPos, v_size, sz_inv, header, values);

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)    
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * (0.5f*sz*header.brick_size);
      qFar = (fNearFar.y - fNearFar.x) * (0.5f*sz*header.brick_size);
    
      LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
    
      if (m_preset.interpolation_type == TRILINEAR_INTERPOLATION_MODE && header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN &&
          m_preset.normal_mode == NORMAL_MODE_SDF_SMOOTHED && 
          need_normal() && pHit->t != old_t)
      {
        const float beta = 0.5f;
        float3 dp = start_q + ((pHit->t - fNearFar.x)/d)*ray_dir; //linear interpolation coefficients in voxels

        float3 normals[8];
        float values_n[8];
        for (int i=0;i<8;i++)
          normals[i] = float3(0,0,0);
        
        float3 nmq = float3(0.5f,0.5f,0.5f);
        nmq.x = dp.x < 0.5f ? step(-beta, beta, dp.x) : step(-beta, beta, dp.x - 1.0f);
        nmq.y = dp.y < 0.5f ? step(-beta, beta, dp.y) : step(-beta, beta, dp.y - 1.0f);
        nmq.z = dp.z < 0.5f ? step(-beta, beta, dp.z) : step(-beta, beta, dp.z - 1.0f);

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
            load_distance_values(neighbor_nodeId, neighbor_voxelPos, v_size, sz_inv, header, values_n);
            normals[i] = normalize(eval_dist_trilinear_diff(values_n, dp - dVoxelPos));
          }
        }

        float3 smoothed_norm = (1-nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[0] + 
                               (1-nmq.x)*(1-nmq.y)*(  nmq.z)*normals[1] + 
                               (1-nmq.x)*(  nmq.y)*(1-nmq.z)*normals[2] + 
                               (1-nmq.x)*(  nmq.y)*(  nmq.z)*normals[3] + 
                               (  nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[4] + 
                               (  nmq.x)*(1-nmq.y)*(  nmq.z)*normals[5] + 
                               (  nmq.x)*(  nmq.y)*(1-nmq.z)*normals[6] + 
                               (  nmq.x)*(  nmq.y)*(  nmq.z)*normals[7];
        smoothed_norm = normalize(smoothed_norm);
        float2 encoded_norm = encode_normal(smoothed_norm);

        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
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
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
             header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
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
#endif
}

void BVHRT::OctreeAdaptBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                      float tNear, uint32_t instId, uint32_t geomId,
                                      uint32_t bvhNodeId, uint32_t a_count,
                                      CRT_Hit *pHit)
{
#ifndef DISABLE_SDF_SBS_ADAPT
  float values[8];
  uint32_t nodeId, primId;
  float qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[geomId].offset.x;
  primId = bvhNodeId; //id of bbox in BLAS
  nodeId = primId + m_SdfSBSAdaptRoots[sdfId];
  SdfSBSAdaptHeader header = m_SdfSBSAdaptHeaders[sdfId];


  float px = m_SdfSBSAdaptNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSBSAdaptNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSBSAdaptNodes[nodeId].pos_z_vox_size >> 16;
  float vs = m_SdfSBSAdaptNodes[nodeId].pos_z_vox_size & 0x0000FFFF;

  // voxel count
  uint3 brick_size{(m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad >> 16) & 0x000000FF,
                   (m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad >>  8) & 0x000000FF,
                   (m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad      ) & 0x000000FF};
  uint3 v_size = brick_size + 2*header.brick_pad + 1;


  float d = (2.0f/SDF_SBS_ADAPT_MAX_UNITS) * vs;
  float voxel_abs_size_inv = 1.f / d;
  float3 brick_abs_size = d * float3(brick_size);
  float3 brick_min_pos = float3(-1,-1,-1) + (2.0f/SDF_SBS_ADAPT_MAX_UNITS)*float3(px,py,pz);
  float3 brick_max_pos = brick_min_pos + brick_abs_size;


  float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
  float old_t = pHit->t;
  while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
  {
    float3 hit_pos = ray_pos + brick_fNearFar.x*ray_dir;
    float3 local_pos = (hit_pos - brick_min_pos) * voxel_abs_size_inv;
    uint3  voxel_pos{(uint32_t)clamp(local_pos.x, 1e-6f, brick_size.x-1e-6f),
                     (uint32_t)clamp(local_pos.y, 1e-6f, brick_size.y-1e-6f),
                     (uint32_t)clamp(local_pos.z, 1e-6f, brick_size.z-1e-6f)};

    float3 min_pos = brick_min_pos + d*float3(voxel_pos);
    float3 max_pos = min_pos + d;
    float3 size = max_pos - min_pos;

    float vmin = 1.0f;

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t v_off = m_SdfSBSAdaptNodes[nodeId].data_offset;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = voxel_pos + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size.y*v_size.z + vPos.y*v_size.z + vPos.z;
        values[i] = m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[v_off + vId]];
        vmin = std::min(vmin, values[i]);
      }
    }
    else
    {
      uint32_t v_off = m_SdfSBSAdaptNodes[nodeId].data_offset;
      uint32_t vals_per_int = 4/header.bytes_per_value; 
      uint32_t bits = 8*header.bytes_per_value;
      uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
      float d_max = 1.73205081f*std::max(std::max(brick_abs_size.x, brick_abs_size.y), brick_abs_size.z);
      float mult = 2*d_max/max_val;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = voxel_pos + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size.y*v_size.z + vPos.y*v_size.z + vPos.z;
        values[i] = -d_max + mult*((m_SdfSBSAdaptData[v_off + vId/vals_per_int] >> (bits*(vId%vals_per_int))) & max_val);
        vmin = std::min(vmin, values[i]);
      }
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * voxel_abs_size_inv;
      qFar = (fNearFar.y - fNearFar.x) * voxel_abs_size_inv;

      LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
    }

    brick_fNearFar.x += std::max(0.0f, fNearFar.y-brick_fNearFar.x) + 1e-6f;
  }

  //ray hit a brick
  if (pHit->t < old_t)
  {
    float3 pos = ray_pos + pHit->t*ray_dir;
    float3 dp = (pos - brick_min_pos)*voxel_abs_size_inv;

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_UV16)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + (v_size.x*v_size.y*v_size.z+vals_per_int-1)/vals_per_int;

      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+0] >> 16)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+1] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+2] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+3] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+4] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+5] >> 16)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+6] >> 16)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+7] >> 16));

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+0] & 0xFFFF)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+1] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+2] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+3] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+4] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+5] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+6] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+7] & 0xFFFF));
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_RGB8)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + (v_size.x*v_size.y*v_size.z+vals_per_int-1)/vals_per_int;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+0] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+0] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+0] >> 16) & 0xFF) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+1] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+1] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+1] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+2] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+2] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+2] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+3] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+3] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+3] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+4] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+4] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+4] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+5] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+5] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+5] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+6] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+6] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+6] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+7] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+7] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+7] >> 16) & 0xFF);

      color = clamp(floor(color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + v_size.x*v_size.y*v_size.z;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+2]) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+2]) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+2]) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+2]) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+2]) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+2]) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+2]) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+2]);

      color = clamp(floor(255.0f*color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
    }
  }
#endif
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
      float2 encoded_norm = encode_normal(n);

      pHit->t         = t;
      pHit->primId    = primId;
      pHit->instId    = instId;
      pHit->geomId    = geomId | (type << SH_TYPE);  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = encoded_norm.x;
      pHit->coords[3] = encoded_norm.y;

      if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
        pHit->primId = uint32_t(hit.hit_norm.w);
    }
  }
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

void BVHRT::lerpCell(const uint32_t idx0, const uint32_t idx1, const float t, float memory[28]) {
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

static float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}

void BVHRT::RayGridIntersection(float3 ray_dir, uint32_t gridSize, float3 p, float3 lastP, uint4 ptrs, uint4 ptrs2, float &throughput, float3 &colour)
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

void BVHRT::IntersectGSInLeaf(const float3& ray_pos, const float3& ray_dir,
                              float tNear, uint32_t instId,
                              uint32_t geomId, uint32_t a_start,
                              uint32_t a_count, CRT_Hit* pHit) {
    float sh_c0 = 0.28209479177387814f;

    for (uint32_t i = a_start; i < a_start + a_count; ++i) {
        float3 mean = float3(m_gs_data_0[i][0][0], m_gs_data_0[i][0][1], m_gs_data_0[i][0][2]);
        float3 scale = float3(exp(m_gs_data_0[i][1][3]), exp(m_gs_data_0[i][2][0]), exp(m_gs_data_0[i][2][1])) * 3.0f;
        float4 rotation = QuaternionConjugate(normalize(float4(m_gs_data_0[i][2][2], -m_gs_data_0[i][2][3], m_gs_data_0[i][3][0], m_gs_data_0[i][3][1])));
        float3 diffuse_color = float3(m_gs_data_0[i][0][3], m_gs_data_0[i][1][0], m_gs_data_0[i][1][1]) * sh_c0;
        float opacity = sigmoid(m_gs_data_0[i][1][2]);

        float3 origin = ray_pos - mean;
        float3 direction = ray_dir;

        origin = RotatePoint(origin, rotation);
        direction = RotatePoint(direction, rotation);

        origin    = origin / scale;
        direction = direction / scale;

        float a = dot(direction, direction);
        float b = 2.0f * dot(origin, direction);
        float c = dot(origin, origin) - 1.0f;

        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 1e-9f) {
            continue;
        }

        float t1 = (-b + sqrt(discriminant)) / (2.0f * a);
        float t2 = (-b - sqrt(discriminant)) / (2.0f * a);

        if (t1 > tNear && t2 > tNear) {
            float t = (t1 + t2) / 2.0f;
            float3 intersection = ray_pos + t * ray_dir;
            float3 distance = intersection - mean;

            float power  = -0.5f * (
                m_gs_conic[i][0][0] * distance.x * distance.x +
                m_gs_conic[i][1][1] * distance.y * distance.y +
                m_gs_conic[i][2][2] * distance.z * distance.z) -
                m_gs_conic[i][0][1] * distance.x * distance.y -
                m_gs_conic[i][0][2] * distance.x * distance.z -
                m_gs_conic[i][1][2] * distance.y * distance.z;

            if (power > 0.0f) {
                continue;
            }

            float alpha = min(0.99f, opacity * float(exp(power)));

            if (alpha < 1.0f / 255.0f) {
                continue;
            }

            float transparency = pHit->coords[0] * (1.0f - alpha);

            if (transparency < 0.0001f) { 
                break;
            }

            float weight = alpha * pHit->coords[0];

            pHit->coords[1] += (0.5f + diffuse_color.x) * weight;
            pHit->coords[2] += (0.5f + diffuse_color.y) * weight;
            pHit->coords[3] += (0.5f + diffuse_color.z) * weight;
            pHit->coords[0] = transparency;

            pHit->primId = i;
        }
    }
}
#endif

#ifndef DISABLE_NURBS
//////////////////// NURBS SECTION /////////////////////////////////////////////////////
float4 
BVHRT::control_point(uint i, uint j, NURBSHeader h)
{
  return LiteMath::float4 { 
      m_NURBSData[h.offset+(i*(h.m+1)+j)*4], 
      m_NURBSData[h.offset+(i*(h.m+1)+j)*4+1],
      m_NURBSData[h.offset+(i*(h.m+1)+j)*4+2],
      m_NURBSData[h.offset+(i*(h.m+1)+j)*4+3] 
  };
} 
float 
BVHRT::weight(uint i, uint j, NURBSHeader h)
{
  return m_NURBSData[weights_offset(h)+i*h.m+j];
}
float 
BVHRT::uknot(uint i, NURBSHeader h)
{
  return m_NURBSData[u_knots_offset(h)+i];
}
float 
BVHRT::vknot(uint i, NURBSHeader h)
{
  return m_NURBSData[v_knots_offset(h)+i];
}
int 
BVHRT::find_uspan(float u, NURBSHeader h) {
  int n = h.n;
  int p = h.p;

  if (u == uknot(n+1, h))
    return n;

  int l = p-1;
  int r = n+2;
  while(r-l > 1) {
    int m = (l+r)/2;
    if (u < uknot(m, h))
      r = m;
    else 
      l = m;
  }
  //assert(uknot(l, h) <= u && u < uknot(l+1, h));
  return l;
}
int 
BVHRT::find_vspan(float v, NURBSHeader h) {
  int n = h.m;
  int p = h.q;

  if (v == vknot(n+1, h))
    return n;

  int l = p-1;
  int r = n+2;
  while(r-l > 1) {
    int m = (l+r)/2;
    if (v < vknot(m, h))
      r = m;
    else 
      l = m;
  }
  //assert(vknot(l, h) <= v && v < vknot(l+1, h));
  return l;
}

void 
BVHRT::ubasis_funs(int i, float u, NURBSHeader h, float N[NURBS_MAX_DEGREE+1]) {
  int n = h.n;
  int p = h.p;

  N[0] = 1.0f;
  float left[NURBS_MAX_DEGREE+1];
  float right[NURBS_MAX_DEGREE+1];
  for (int i = 0; i < p+1; ++i)
    left[i] = right[i] = 0;

  for (int j = 1; j <= p; ++j) {
    left[j] = u - uknot(i+1-j, h);
    right[j] = uknot(i+j, h)-u;
    float saved = 0.0f;
    for (int r = 0; r < j; ++r) {
      float temp = N[r] / (right[r+1]+left[j-r]);
      N[r] = saved + right[r+1] * temp;
      saved = left[j-r] * temp;
    }
    N[j] = saved;
  }
}
void 
BVHRT::vbasis_funs(int i, float v, NURBSHeader h, float N[NURBS_MAX_DEGREE+1]) {
  int n = h.m;
  int p = h.q;
  N[0] = 1.0f;
  float left[NURBS_MAX_DEGREE+1];
  float right[NURBS_MAX_DEGREE+1];
  for (int i = 0; i < p+1; ++i)
    left[i] = right[i] = 0;

  for (int j = 1; j <= p; ++j) {
    left[j] = v - vknot(i+1-j, h);
    right[j] = vknot(i+j, h)-v;
    float saved = 0.0f;
    for (int r = 0; r < j; ++r) {
      float temp = N[r] / (right[r+1]+left[j-r]);
      N[r] = saved + right[r+1] * temp;
      saved = left[j-r] * temp;
    }
    N[j] = saved;
  }
}

float4 
BVHRT::nurbs_point(float u, float v, NURBSHeader h) {
  int n = h.n;
  int m = h.m;
  int p = h.p;
  int q = h.q;
  //assert(p <= NURBS_MAX_DEGREE);
  //assert(q <= NURBS_MAX_DEGREE);

  int uspan = find_uspan(u, h);
  float Nu[NURBS_MAX_DEGREE+1];
  for (int i = 0; i < p+1; ++i)
    Nu[i] = 0;
  ubasis_funs(uspan, u, h, Nu);
  
  int vspan = find_vspan(v, h);
  float Nv[NURBS_MAX_DEGREE+1];
  for (int i = 0; i < q+1; ++i)
    Nv[i] = 0;
  vbasis_funs(vspan, v, h, Nv);

  float4 temp[NURBS_MAX_DEGREE+1];
  for (int i = 0; i < q+1; ++i)
    temp[i] = float4{ 0.0f, 0.0f, 0.0f, 0.0f };

  for (int l = 0; l <= q; ++l) 
  for (int k = 0; k <= p; ++k) 
  {
    temp[l] += Nu[k] 
             * control_point(uspan-p+k, vspan-q+l, h) 
             * weight(uspan-p+k, vspan-q+l, h);
  }

  float4 res = { 0.0f, 0.0f, 0.0f, 0.0f };
  for (int l = 0; l <= q; ++l)
    res += Nv[l] * temp[l];
  
  return res/res.w;
}

float4 
BVHRT::uder(float u, float v, NURBSHeader h) {
  const float EPS = 1e-2f;
  LiteMath::float4 res = { 0.0f, 0.0f, 0.0f, 0.0f };
  res += (u+EPS > 1.0f) ? nurbs_point(u, v, h) : nurbs_point(u+EPS, v, h);
  res -= (u-EPS < 0.0f) ? nurbs_point(u, v, h) : nurbs_point(u-EPS, v, h);

  return res / (EPS * (1 + (int)((u+EPS <= 1.0f) && (u-EPS >= 0.0f))));
}

float4 
BVHRT::vder(float u, float v, NURBSHeader h) {
  const float EPS = 1e-2f;
  LiteMath::float4 res = { 0.0f, 0.0f, 0.0f, 0.0f };
  res += (v+EPS > 1.0f) ? nurbs_point(u, v, h) : nurbs_point(u, v+EPS, h);
  res -= (v-EPS < 0.0f) ? nurbs_point(u, v, h) : nurbs_point(u, v-EPS, h);

  return res / (EPS * (1 + (int)((v+EPS <= 1.0f) && (v-EPS >= 0.0f))));
}

bool 
BVHRT::uclosed(NURBSHeader h) {
  const float EPS = 1e-2f;
  for (int j = 0; j <= h.m; ++j) 
    if (length(control_point(0, j, h) - control_point(h.n, j, h)) > EPS)
      return false;
  return true;
}

bool 
BVHRT::vclosed(NURBSHeader h) {
  const float EPS = 1e-2f;
  for (int i = 0; i <= h.n; ++i) 
    if (length(control_point(i, 0, h) - control_point(i, h.m, h)) > EPS)
      return false;
  return true;
}

float closed_clamp(float val) {
  val -= int(val);
  if (val < 0.0f)
    return 1.0f+val;
  if (val > 1.0f)
    return val-1.0f;
  return val;
}

NURBS_HitInfo 
BVHRT::ray_nurbs_newton_intersection(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    NURBSHeader h) {
  const int max_steps = 16;
  const float EPS = 0.001f;

  bool u_closed = uclosed(h);
  bool v_closed = vclosed(h);

  float3 ortho_dir1 = (bool(ray.x) || bool(ray.z)) ? float3{ 0, 1, 0 } : float3{ 1, 0, 0 };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));
  // assert(dot(ortho_dir1, ortho_dir2) < EPS);
  // assert(dot(ortho_dir1, ray) < EPS);
  // assert(dot(ortho_dir2, ray) < EPS);

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  // assert(dot(P1, to_float4(pos, 1.0f)) < EPS);
  // assert(dot(P2, to_float4(pos, 1.0f)) < EPS);

  float2 uv = { 0.5f, 0.5f };
  float2 D = { dot(P1, nurbs_point(uv.x, uv.y, h)), dot(P2, nurbs_point(uv.x, uv.y, h)) };
  NURBS_HitInfo hit_info = { false, uv };

  int steps_left = max_steps-1;
  while(length(D) > EPS && (steps_left!=0)) {
    steps_left--;
    float2 J[2] = 
    { 
      { dot(P1, uder(uv.x, uv.y, h)), dot(P2, uder(uv.x, uv.y, h)) }, //col1
      { dot(P1, vder(uv.x, uv.y, h)), dot(P2, vder(uv.x, uv.y, h)) } //col2
    };

    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float2 J_inversed[2] = 
    {
      { J[1][1]/det, -J[0][1]/det },
      { -J[1][0]/det, J[0][0]/det }
    };

    uv = uv - (J_inversed[0]*D[0]+J_inversed[1]*D[1]);
    uv.x = u_closed ? closed_clamp(uv.x) : clamp(uv.x, 0.0f, 1.0f);
    uv.y = v_closed ? closed_clamp(uv.y) : clamp(uv.y, 0.0f, 1.0f);
    // assert(0 <= uv.x && uv.x <= 1);
    // assert(0 <= uv.y && uv.y <= 1);

    float2 new_D = { dot(P1, nurbs_point(uv.x, uv.y, h)), dot(P2, nurbs_point(uv.x, uv.y, h)) };
    
    if (length(new_D) > length(D))
      return hit_info;
    
    D = new_D;
  }

  if (length(D) > EPS)
    return hit_info;
  
  hit_info.hitten = true;
  hit_info.uv = uv;
  return hit_info;
}

void BVHRT::IntersectNURBS(const float3& ray_pos, const float3& ray_dir,
                           float tNear, uint32_t instId,
                           uint32_t geomId, CRT_Hit* pHit) {
  uint nurbsId = m_geomData[geomId].offset.x;
  NURBSHeader header  = m_NURBSHeaders[nurbsId];
  uint type = m_geomData[geomId].type;

  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);

  NURBS_HitInfo hit_pos = ray_nurbs_newton_intersection(ray_pos, ray_dir, header);
  if (hit_pos.hitten) {
    float2 uv = hit_pos.uv;
    float3 point = to_float3(nurbs_point(uv.x, uv.y, header));
    float t = dot(normalize(ray_dir), point-ray_pos);
    if (t < tNear_tFar.x || t > tNear_tFar.y)
      return;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->t = t;
    pHit->primId = 0;
    pHit->instId = instId;

    pHit->coords[0] = uv.x;
    pHit->coords[1] = uv.y;
  } 
}
//////////////////////// END NURBS SECTION ///////////////////////////////////////////////
#endif

void BVHRT::IntersectOpenVDB_Grid(const float3& ray_pos, const float3& ray_dir,
                           float tNear, uint32_t instId,
                           uint32_t geomId, CRT_Hit* pHit)
{
  uint openvdbId = m_geomData[geomId].offset.x;
  uint type = m_geomData[geomId].type;

  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);
  
  float dist = 1000;
  float t = std::max(tNear, tNear_tFar.x), tFar = tNear_tFar.y;
  float EPS = 1e-6f;
  bool hit = 0;
  int iter = 1;
  const uint32_t ST_max_iters = 256;

  openvdb::Vec3f rayOrigin(ray_pos.x, ray_pos.y, ray_pos.z), rayDirection(ray_dir.x, ray_dir.y, ray_dir.z);
  openvdb::Vec3f point = rayOrigin + t * rayDirection;
  openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler> sampler(*(openvdb_grid.sdfGrid));

  dist = std::abs(sampler.wsSample(point));

  while (t < tFar && dist > EPS && iter < ST_max_iters)
  {
    t += dist + EPS;
    point = rayOrigin + t * rayDirection;
    dist = std::abs(sampler.wsSample(point));
    
    iter++;  
  }
  
  hit = (dist <= EPS);

  if (!hit || t < tNear || t > tFar)
  {
    return;
  }

  pHit->geomId = geomId | (type << SH_TYPE);
  pHit->t = dot(normalize(ray_dir), float3(point.x(), point.y(), point.z()) - ray_pos);
  pHit->primId = 0;
  pHit->instId = instId;

  pHit->coords[0] = 0;
  pHit->coords[1] = 0;
  pHit->coords[2] = 0;
  pHit->coords[3] = 0;
}


float4 rayCapsuleIntersect(const float3& ray_pos, const float3& ray_dir,
                           const float3& pa, const float3& pb, float ra)
{
  float3 ba = pb - pa;
  float3 oa = ray_pos - pa;
  float3 ob = ray_pos - pb;
  float baba = dot(ba,ba);
  float bard = dot(ba,ray_dir);
  float baoa = dot(ba,oa);
  float rdoa = dot(ray_dir,oa);
  float oaoa = dot(oa,oa);
  float a = baba      - bard*bard;
  float b = baba*rdoa - baoa*bard;
  float c = baba*oaoa - baoa*baoa - ra*ra*baba;
  float h = b*b - a*c;

  if(h >= 0.f)
  {
    float dist = (-b-sqrt(h))/a;
    float y = baoa + dist*bard;

    // body
    if(y > 0.f && y < baba)
      return to_float4(normalize(oa+dist*ray_dir - ba*y/baba), dist);

    // cap
    float3 oc = (y <= 0.0) ? oa : ob;
    b = dot(ray_dir,oc);
    c = dot(oc,oc) - ra*ra;
    h = b*b - c;
    if(h > 0.f)
    {
      dist = -b - sqrt(h);
      float3 norm = normalize(ray_pos+dist*ray_dir + (oc - ray_pos));
      return to_float4(norm, dist);
    }
  }
  return float4(2e15f);
}

void BVHRT::IntersectGraphicPrims(const float3& ray_pos, const float3& ray_dir,
                                  float tNear, uint32_t instId,
                                  uint32_t geomId, uint32_t a_start,
                                  uint32_t a_count, CRT_Hit* pHit)
{
#ifndef DISABLE_GRAPHICS_PRIM

  const float EPS = 1e-5f, T_MAX = 1e15f;
  uint32_t primId     = m_geomData[geomId].offset.x;
  uint32_t nextPrimId = m_geomData[geomId].offset.y;
  GraphicsPrimHeader header = m_GraphicsPrimHeaders[primId];
  float3 color = header.color;

  uint32_t prim_len = 2u;
  if (header.prim_type == GRAPH_PRIM_POINT)
    prim_len = 1u;
  if (header.prim_type >= GRAPH_PRIM_LINE_COLOR)
    prim_len = 3u;
  
  uint32_t start_index = primId + prim_len * a_start;
  uint32_t end_index = start_index + prim_len; // 1 per bbox
  // start_index = primId;
  // end_index = nextPrimId;

  bool has_custom_color = header.prim_type >= GRAPH_PRIM_POINT_COLOR;

  if (header.prim_type == GRAPH_PRIM_POINT ||
      header.prim_type == GRAPH_PRIM_POINT_COLOR)
  {
    for (uint32_t i = start_index; i < end_index; i += prim_len)
    {
      float4 point = m_GraphicsPrimPoints[i];
      if (has_custom_color)
        color = to_float3(m_GraphicsPrimPoints[i+1]);

      float3 point3 = float3(point.x, point.y, point.z);
      float pt_radius = point.w;
      float3 pos_minus_point3 = ray_pos - point3;

      float b = 2.f * dot(ray_dir, pos_minus_point3);
      float c = dot(pos_minus_point3, pos_minus_point3) - pt_radius * pt_radius;
      float D = b*b - 4*c;
      if (D >= EPS)
      {
        D = std::sqrt(D);
        float t = -(b + D) * 0.5f;
        if (t < EPS)
          t = (-b + D) * 0.5f;

        if (t > tNear && t < pHit->t)
        {
          float3 norm = normalize((ray_pos + t*ray_dir - point3) * 100.f);
          float2 encoded_norm = encode_normal(norm);

          pHit->t         = t;
          pHit->primId    = primId;
          pHit->instId    = instId;
          pHit->geomId    = geomId | (TYPE_GRAPHICS_PRIM << SH_TYPE);
          pHit->coords[0] = color.x + color.y/256.0f;
          pHit->coords[1] = color.z;
          pHit->coords[2] = encoded_norm.x;
          pHit->coords[3] = encoded_norm.y;
        }
      }
    }
  }
  else
  {
    for (uint32_t i = start_index; i < end_index; i += prim_len)
    {
      float4 point1 = m_GraphicsPrimPoints[i  ];
      float4 point2 = m_GraphicsPrimPoints[i+1];
      if (has_custom_color)
        color = to_float3(m_GraphicsPrimPoints[i+2]);

      float3 point1_3 = float3(point1.x, point1.y, point1.z);
      float3 point2_3 = float3(point2.x, point2.y, point2.z);
      if (header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
        point2_3 = point2_3 * 0.71f + point1_3 * 0.29f; // so that the line segment doesn't clip through the cone
      float ra = point1.w; // line (cylinder) radius

      //assert(ra > EPS);

      float t = T_MAX;
      float3 norm = float3(0.f, 0.f, 0.f);

      if (header.prim_type == GRAPH_PRIM_LINE ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_COLOR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_COLOR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
      {
        float3 oc =  ray_pos - point1_3;
        float3 ba = point2_3 - point1_3;

        float baba = dot(ba, ba);
        float bard = dot(ba, ray_dir);
        float baoc = dot(ba, oc);

        float k2 = baba - (bard * bard);
        float k1 = baba*dot(oc, ray_dir) - baoc*bard;
        float k0 = baba*dot(oc, oc) - baoc*baoc - ra*ra*baba;

        float D = k1*k1 - k2*k0;

        if (D > 0.f)
        {
          float dist = -(k1 + std::sqrt(D)) / k2;
          float y = baoc + dist*bard;

          if (header.prim_type == GRAPH_PRIM_LINE || (y > 0.f && y < baba))
          {
            t = dist;
            norm = normalize(oc+t*ray_dir - ba*y/baba);
          }
          else
          {
            dist = ((y < 0.f ? 0.f : baba) - baoc)/bard;
            if (std::abs(k1+k2*dist)<std::sqrt(D))
            {
              t = dist;
              norm = normalize(ba*sign(y));
            }
          }
        }
      }
      if (header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
      {
        ra = length(point2_3 - point1_3) * 0.15f;
        point2_3 = float3(point2.x, point2.y, point2.z);
        float3 pa = point2_3 * 0.7f + point1_3 * 0.3f; // cone length along line segment is 0.3*length(b - a) for now
        float3 ba = point2_3 - pa;
        float3 oa = ray_pos - pa;
        float3 ob = ray_pos - point2_3;
        float  m0 = dot(ba,ba);
        float  m1 = dot(oa,ba);
        float  m2 = dot(ray_dir,ba);
        float  m3 = dot(ray_dir,oa);
        float  m5 = dot(oa,oa);
        float  m9 = dot(ob,ba);

        // cap - only one
        if(m1 < 0.f)
        {
          float3 tmp1 = oa * m2 - ray_dir * m1;
          if(dot(tmp1, tmp1) < (ra*ra*m2*m2))
          {
            t = -m1/m2;
            norm = normalize(-1.0f*ba);
          }
        }
        
        // body
        float rr = ra;
        float hy = m0 + rr*rr;
        float k2 = m0*m0    - m2*m2*hy;
        float k1 = m0*m0*m3 - m1*m2*hy + m0*ra*(rr*m2);
        float k0 = m0*m0*m5 - m1*m1*hy + m0*ra*(rr*m1*2.0 - m0*ra);

        float D = k1*k1 - k2*k0;
        if(D > 0.f)
        {
          float dist = -(k1 + std::sqrt(D)) / k2;
          float y = m1 + dist * m2;
          if(y >= 0.f && y <= m0)
          {
            t = dist;
            norm = normalize(m0*(m0*(oa+t*ray_dir)+rr*ba*ra)-ba*hy*y);
          }
        }
      }
      if (header.prim_type == GRAPH_PRIM_BOX ||
          header.prim_type == GRAPH_PRIM_BOX_COLOR)
      {
        float3 pa = min(point2_3, point1_3);
        float3 pb = max(point2_3, point1_3);
        float minmax[3][2] = {{pa.x, pb.x}, {pa.y, pb.y}, {pa.z, pb.z}};

        const int4 vert_indices{0,3,5,6};
        for (int vert_ind = 0; vert_ind < 4; ++vert_ind)
        {
          int axis_indices[3] = { int((vert_indices[vert_ind] & 4u) != 0),
                                  int((vert_indices[vert_ind] & 2u) != 0),
                                  int((vert_indices[vert_ind] & 1u) != 0) };
          pa = float3(minmax[0][axis_indices[0]],
                      minmax[1][axis_indices[1]],
                      minmax[2][axis_indices[2]]);
          for (int axis = 0; axis < 3; ++axis)
          {
            pb = pa;
            pb[axis] = minmax[axis][1-axis_indices[axis]];
            float4 norm_dist = rayCapsuleIntersect(ray_pos, ray_dir, pa, pb, ra);
            if (norm_dist.w < t)
            {
              t = norm_dist.w;
              norm = to_float3(norm_dist);
            }
          }
        }
      }

      if (t > tNear && t < pHit->t)
      {
        float2 encoded_norm = encode_normal(norm);

        pHit->t         = t;
        pHit->primId    = primId;
        pHit->instId    = instId;
        pHit->geomId    = geomId | (TYPE_GRAPHICS_PRIM << SH_TYPE);
        pHit->coords[0] = color.x + color.y/256.0f;
        pHit->coords[1] = color.z;
        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
    }
  }
#endif // DISABLE_GRAPHICS_PRIM
}

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
  if (m_preset.interpolation_type == TRILINEAR_INTERPOLATION_MODE)
  {
    if (vox_u.x < size.x-1 && vox_u.y < size.y-1 && vox_u.z < size.z-1)
    {
      for (uint32_t i=0;i<2;i++)
      {
        for (uint32_t j=0;j<2;j++)
        {
          for (uint32_t k=0;k<2;k++)
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
  } // tricubic interpolation
  else if (m_preset.interpolation_type == TRICUBIC_INTERPOLATION_MODE)
  {
    if (vox_u.x < size.x-2 && vox_u.y < size.y-2 && vox_u.z < size.z-2 && vox_u.x > 0 && vox_u.y > 0 && vox_u.z > 0)
    {
      uint32_t x0 = vox_u.x - 1, y0 = vox_u.y - 1, z0 = vox_u.z - 1;
      float f_dp[3] = {dp.x, dp.y, dp.z}, grid_part[64];

      for (int x = 0; x < 4; ++x)
      {
        for (int y = 0; y < 4; ++y)
        {
          for (int z = 0; z < 4; ++z)
          {
            grid_part[z * 4 * 4 + y * 4 + x] = m_SdfGridData[off + (z0 + z)*size.x*size.y + (y0 + y)*size.x + (x0 + x)];
          }
        }
      }
      res = tricubicInterpolation(grid_part, f_dp);
    }
    else if (vox_u.x < size.x-1 && vox_u.y < size.y-1 && vox_u.z < size.z-1)
    {
      for (uint32_t i=0;i<2;i++)
      {
        for (uint32_t j=0;j<2;j++)
        {
          for (uint32_t k=0;k<2;k++)
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
  }
  
  return res;
}
#endif

float tricubic_spline(float p0, float p1, float p2, float p3, float x)
{
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + x * (3.0 * (p1 - p2) + p3 - p0)));
}

float
BVHRT::tricubicInterpolation(const float grid[64], const float dp[3])
{
  float res = 0;
  float values_yz[16];
  float values_z[4];

  for (uint32_t j = 0; j < 4; ++j)
  {
    for (uint32_t k = 0; k < 4; ++k)
    {
      //m_SdfGridData[off + (vox_u.z)*size.x*size.y + (vox_u.y)*size.x + (vox_u.x)]
      values_yz[4*j + k] = 
        tricubic_spline(
            grid[k*4*4 + (j)*4 + (0)],
            grid[k*4*4 + (j)*4 + (1)],
            grid[k*4*4 + (j)*4 + (2)],
            grid[k*4*4 + (j)*4 + (3)], 
            dp[0]
      );
    }
  }

  for (uint32_t k = 0; k < 4; ++k)
  {
    values_z[k] = 
      tricubic_spline(
          values_yz[4*0 + k], 
          values_yz[4*1 + k], 
          values_yz[4*2 + k], 
          values_yz[4*3 + k], 
          dp[1]
    );
  }

  res = tricubic_spline(values_z[0], values_z[1], values_z[2], values_z[3], dp[2]);

  return res;
}

#ifdef USE_ENZYME
void __enzyme_autodiff(void*, ...);
int enzyme_dup;
int enzyme_dupnoneed;
int enzyme_out;
int enzyme_const;
#endif

void 
BVHRT::tricubicInterpolationDerrivative(const float grid[64], const float dp[3], float d_pos[3], float d_grid[64])
{
  #ifdef USE_ENZYME
  __enzyme_autodiff((void*)(tricubicInterpolation), 
                    enzyme_dup, grid, d_grid, 
                    enzyme_dup, dp, d_pos);

  #endif
}

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
#ifndef DISABLE_MESH
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
        if (m_preset.normal_mode == NORMAL_MODE_GEOMETRY)
        {
          n = cross(edge1, edge2);
        }
        else if (m_preset.normal_mode == NORMAL_MODE_VERTEX)
        {
          n = to_float3(m_vertNorm[a_geomOffsets.y + A] * (1.0f - u - v) + m_vertNorm[a_geomOffsets.y + B] * v + u * m_vertNorm[a_geomOffsets.y + C]);
        }

        n = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, n));
        float2 encoded_norm = encode_normal(n);

        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
      else
      {
        pHit->coords[2] = 0;
        pHit->coords[3] = 0;
      }
    }
  }
#endif
}

static inline int first_node(float3 t0, float3 tm)
{
uint32_t answer = 0;   // initialize to 00000000
// select the entry plane and set bits
if(t0.x > t0.y){
    if(t0.x > t0.z){ // PLANE YZ
        if(t0.x > tm.y) answer|=2;    // set bit at position 1
        if(t0.x > tm.z) answer|=1;    // set bit at position 0
        return (int) answer;
    }
}
else {
    if(t0.y > t0.z){ // PLANE XZ
        if(tm.x < t0.y) answer|=4;    // set bit at position 2
        if(t0.y > tm.z) answer|=1;    // set bit at position 0
        return (int) answer;
    }
}
// PLANE XY
if(tm.x < t0.z) answer|=4;    // set bit at position 2
if(tm.y < t0.z) answer|=2;    // set bit at position 1
return (int) answer;
}

static inline int new_node(float txm, int x, float tym, int y, float tzm, int z)
{
  return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
}

void BVHRT::OctreeIntersect(const float3 ray_pos, const float3 ray_dir, float tNear, 
                            uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                            CRT_Hit *pHit)
{
#ifndef DISABLE_SDF_FRAME_OCTREE_COMPACT
  float3 pos_ray_pos = ray_pos;
  float3 pos_ray_dir = ray_dir;
  //assume octree is box [-1,1]^3
  uint32_t a = 0;
  if (ray_dir.x < 0)
  {
    pos_ray_pos.x *= -1;
    pos_ray_dir.x *= -1;
    a |= 4;
  }

  if (ray_dir.y < 0)
  {
    pos_ray_pos.y *= -1;
    pos_ray_dir.y *= -1;
    a |= 2;
  }

  if (ray_dir.z < 0)
  {
    pos_ray_pos.z *= -1;
    pos_ray_dir.z *= -1;
    a |= 1;
  }

  pos_ray_pos *= -1;
  const float3 pos_ray_dir_inv = SafeInverse(pos_ray_dir);
  const float3 _t0 = pos_ray_pos * pos_ray_dir_inv - pos_ray_dir_inv;
  const float3 _t1 = pos_ray_pos * pos_ray_dir_inv + pos_ray_dir_inv;
  const float3 _l = _t1 - _t0;
  //printf("_t0 %f %f %f\n", _t0.x, _t0.y, _t0.z);
  //printf("_t1 %f %f %f\n", _t1.x, _t1.y, _t1.z);

  const uint3 nn_indices[8] = {uint3(4, 2, 1), uint3(5, 3, 8), uint3(6, 8, 3), uint3(7, 8, 8),
                               uint3(8, 6, 5), uint3(8, 7, 8), uint3(8, 8, 7), uint3(8, 8, 8)};

  OTStackElement stack[32];
  OTStackElement tmp_buf[4];

  int top = 0;
  int buf_top = 0;
  uint3 p;
  float3 p_f, t0, t1, tm;
  int currNode;
  uint32_t nodeId;
  uint32_t level_sz;
  float d, qFar;
  float2 fNearFar;
  float3 start_q;
  float values[8];

  stack[top].nodeId = 0;
  stack[top].curChildId = 0;
  stack[top].p_size = uint2(0,1);

    while (top >= 0)
    {
      level_sz = stack[top].p_size.y & 0xFFFF;
      p = uint3(stack[top].p_size.x >> 16, stack[top].p_size.x & 0xFFFF, stack[top].p_size.y >> 16);
      d = 1.0f/float(level_sz);
      p_f = float3(p);
      t0 = _t0 + d*p_f * _l;
      t1 = _t0 + d*(p_f + 1) * _l;

      if(stack[top].curChildId > 0) //leaf node
      {
        float tmin = std::max(t0.x, std::max(t0.y, t0.z));
        float tmax = std::min(t1.x, std::min(t1.y, t1.z));

        {
          uint32_t p_mask = level_sz - 1;
          uint3 real_p = uint3(((a & 4) > 0) ? (~p.x & p_mask) : p.x, 
                               ((a & 2) > 0) ? (~p.y & p_mask) : p.y, 
                               ((a & 1) > 0) ? (~p.z & p_mask) : p.z);

          nodeId = stack[top].nodeId;
          float sz = 0.5f*float(level_sz);
          d = 1.0f/sz;
          float3 min_pos = float3(-1,-1,-1) + d*float3(real_p.x,real_p.y,real_p.z);

          fNearFar = float2(tmin, tmax);
          float3 start_pos = ray_pos + fNearFar.x*ray_dir;
          start_q = sz*(start_pos - min_pos);
          qFar = sz*(fNearFar.y - fNearFar.x);
        }

        float d_max = 2*1.73205081f/float(level_sz);
        for (int i=0;i<8;i++)
        {
          values[i] = -d_max + 2*d_max*(1.0/255.0f)*((m_SdfCompactOctreeData[stack[top].nodeId + i/4] >> (8*(i%4))) & 0xFF);
        }
        
        LocalSurfaceIntersection(TYPE_SDF_FRAME_OCTREE, ray_dir, 0, 0, values, nodeId, nodeId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                                 pHit); /*out*/

        if (pHit->primId != 0xFFFFFFFF)
          top = -1;
        else
          top--;
      }
      else
      { 
        buf_top = 0;
        tm = 0.5f*(t0 + t1);

        currNode = first_node(t0, tm);
        do
        {
          //0-7 bits are child_is_active flags, next 8-15 bits are child_is_leaf flags
          uint32_t childrenInfo = m_SdfCompactOctreeData[stack[top].nodeId + 1];
          uint32_t childNode = currNode ^ a; //child node number, from 0 to 8

          // if child is active
          if ((childrenInfo & (1u << childNode)) > 0)
          {
            uint32_t currChildOffset = m_bitcount[childrenInfo & ((1u << childNode) - 1)]; //number of child in the list of active children
            uint32_t baseChildrenOffset = m_SdfCompactOctreeData[stack[top].nodeId];
            tmp_buf[buf_top].nodeId = baseChildrenOffset + 2u*currChildOffset;
            tmp_buf[buf_top].curChildId = childrenInfo & (1u << (childNode + 8)); // > 0 is child is leaf
            tmp_buf[buf_top].p_size = (stack[top].p_size << 1) | uint2(((currNode & 4) << (16-2)) | ((currNode & 2) >> 1), (currNode & 1) << 16);
            buf_top++;
          }
          //return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
          currNode = new_node(((currNode & 4) > 0) ? t1.x : tm.x, nn_indices[currNode].x,
                              ((currNode & 2) > 0) ? t1.y : tm.y, nn_indices[currNode].y,
                              ((currNode & 1) > 0) ? t1.z : tm.z, nn_indices[currNode].z);
        } while (currNode<8);

        for (int i = 0; i < buf_top; i++)
        {
          stack[top+i] = tmp_buf[buf_top-i-1];
        }
        top += buf_top - 1;
      }
    }
#endif
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
    
        if (m_geomData[geomId].type == TYPE_SDF_FRAME_OCTREE && m_preset.octree_intersect == OCTREE_INTERSECT_TRAVERSE)
          OctreeIntersect(ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, &hit);
        else
          BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, &hit);
      }
    } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //
  }

#ifndef DISABLE_MESH
  if(hit.geomId < uint32_t(-1) && ((hit.geomId >> SH_TYPE) == TYPE_MESH_TRIANGLE)) 
  {
    const uint2 geomOffsets = m_geomData[hit.geomId & 0x0FFFFFFF].offset;
    hit.primId = m_primIndices[geomOffsets.x/3 + hit.primId];
  }
#endif
  
  return hit;
}

bool BVHRT::RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar)
{
  dirAndFar.w *= -1.0f;
  CRT_Hit hit = RayQuery_NearestHit(posAndNear, dirAndFar);
  return (hit.geomId != uint32_t(-1));
}
