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

#ifndef LITERT_MINI
float BVHRT::eval_dist_prim(uint32_t prim_id, float3 p)
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
    uint32_t t_ofs1 = 0;
    uint32_t t_ofs2 = NEURAL_SDF_MAX_LAYER_SIZE;

    tmp_mem[t_ofs1 + 0] = p.x;
    tmp_mem[t_ofs1 + 1] = p.y;
    tmp_mem[t_ofs1 + 2] = p.z;

    for (int l = 0; l < prop.layer_count; l++)
    {
      uint32_t m_ofs = prop.layers[l].offset;
      uint32_t b_ofs = prop.layers[l].offset + prop.layers[l].in_size * prop.layers[l].out_size;
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

float BVHRT::eval_dist_sdf_conjunction(uint32_t conj_id, float3 p)
{
  SdfConjunction conj = m_SdfConjunctions[conj_id];
  float conj_d = -1e6;
  for (uint32_t pid = conj.offset; pid < conj.offset + conj.size; pid++)
  {
    float prim_d = m_SdfObjects[pid].distance_mult * eval_dist_prim(pid, p) +
                   m_SdfObjects[pid].distance_add;
    conj_d = max(conj_d, m_SdfObjects[pid].complement == 1 ? -prim_d : prim_d);
  }
  return conj_d;
}
#endif

void BVHRT::IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_Hit *pHit)
{
  uint32_t type = m_geomTypeByGeomId[geomId];
  switch (type)
  {
  case TYPE_MESH_TRIANGLE:
    IntersectAllTrianglesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_SDF_PRIMITIVE:
  case TYPE_SDF_GRID:
  case TYPE_SDF_OCTREE:
    IntersectAllSdfsInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_SDF_FRAME_OCTREE:
    if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_DEFAULT)
      IntersectAllSdfsInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    else
      OctreeNodeIntersect(type, ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_RF_GRID:
    IntersectRFInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  case TYPE_SDF_SVS:
  case TYPE_SDF_SBS:
    OctreeNodeIntersect(type, ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  default:
    break;
  }
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

void BVHRT::OctreeNodeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                float tNear, uint32_t instId, uint32_t geomId,
                                uint32_t a_start, uint32_t a_count,
                                CRT_Hit *pHit)
{
  const float EPS = 1e-6;

  float values[8];
  uint32_t nodeId, primId;
  float d, qFar;
  float2 fNearFar;
  float3 start_q;
  float3 min_pos, max_pos;

  if (type == TYPE_SDF_FRAME_OCTREE)
  {
    uint32_t sdfId =  m_geomOffsets[geomId].x;
    primId = m_origNodes[a_start].leftOffset;
    nodeId = primId + m_SdfFrameOctreeRoots[sdfId];
    min_pos = m_origNodes[a_start].boxMin;
    max_pos = m_origNodes[a_start].boxMax;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = m_SdfFrameOctreeNodes[nodeId].values[i];

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    fNearFar.x = std::max(fNearFar.x, tNear);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos)/(2.0f*d);
    qFar = (fNearFar.y - fNearFar.x) / (2.0f * d);
  }
  else if (type == TYPE_SDF_SVS)
  {
    uint32_t sdfId =  m_geomOffsets[geomId].x;
    primId = a_start;
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
    fNearFar.x = std::max(fNearFar.x, tNear);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos)/(2.0f*d);
    qFar = (fNearFar.y - fNearFar.x) / (2.0f * d);
  }
#ifndef LITERT_MINI
  else //if (type == TYPE_SDF_SBS)
  {
    uint32_t sdfId =  m_geomOffsets[geomId].x;
    primId = a_start; //id of bbox in BLAS
    nodeId = m_SdfSBSRemap[primId + m_geomOffsets[geomId].y].x; //id of node (brick) in SBS
    uint32_t voxelId = m_SdfSBSRemap[primId + m_geomOffsets[geomId].y].y;
    SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
    uint3 voxelPos = uint3(voxelId/(header.v_size*header.v_size), voxelId/header.v_size%header.v_size, voxelId%header.v_size);

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
      uint32_t vId = vPos.x*header.v_size*header.v_size + vPos.y*header.v_size + vPos.z;
      values[i] = -d_max + mult*((m_SdfSBSData[v_off + vId/vals_per_int] >> (bits*(vId%vals_per_int))) & max_val);
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    fNearFar.x = std::max(fNearFar.x, tNear);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos)/(2.0f*d);
    qFar = (fNearFar.y - fNearFar.x) / (2.0f * d);
  }
#endif

  float t = 0;
  bool hit = false;
  unsigned iter = 0;

  float start_dist = eval_dist_trilinear(values, start_q);
  if (start_dist <= EPS || m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_BBOX)
  {
    hit = true;
  }
  else if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_ST)
  {
    const unsigned ST_max_iters = 256;
    float dist = start_dist;
    float3 pp0 = start_q + t * ray_dir;

    while (t < qFar && dist > EPS && iter < ST_max_iters)
    {
      t += dist / (2.0f * d);
      dist = eval_dist_trilinear(values, start_q + t * ray_dir);
      float3 pp = start_q + t * ray_dir;
      iter++;
    }
    hit = (dist <= EPS);
  }
  else //if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC ||
       //    m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON ||
       //    m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
  {
    //finding exact intersection between surface sdf(x,y,z) = 0 and ray
    // based on paper "Ray Tracing of Signed Distance Function Grids, 
    // Journal of Computer Graphics Techniques (JCGT), vol. 11, no. 3, 94-113, 2022"
    // http://jcgt.org/published/0011/03/06/

    // define values and constants as proposed in paper
    float s000 = values[0]/d;
    float s001 = values[1]/d;
    float s010 = values[2]/d;
    float s011 = values[3]/d;
    float s100 = values[4]/d;
    float s101 = values[5]/d;
    float s110 = values[6]/d;
    float s111 = values[7]/d;

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

    if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC)
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
    else if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON)
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
    else //if (m_preset.sdf_frame_octree_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
    {
      const unsigned IT_max_iters = 256;
      const float k = 2;

      float e = 0.1f*qFar;
      float t_max = abs(c3) < EPS ? 1e6 : -c2/(3*c3);
      float df_max = 3*c3*t_max*t_max + 2*c2*t_max + c1;

      float dist = start_dist;
      float3 pp = start_q + t * ray_dir;

      while (t < qFar && dist > EPS && iter < IT_max_iters)
      {
        float df_1 = 3*c3*t*t + 2*c2*t + c1;
        float df_2 = 3*c3*(t+e)*(t+e) + 2*c2*(t+e) + c1;
        float L = (t_max > t && t_max < t + e) ? std::max(df_max, std::max(df_1, df_2)) : std::max(df_1, df_2);
        L = std::max(L, EPS);
        float s = std::min((dist / (2.0f * d))/L, e);
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
    printf("node bbox [(%f %f %f)-(%f %f %f)]\n", min_pos.x, min_pos.y, min_pos.z, max_pos.x, max_pos.y, max_pos.z);
    printf("sdf values %f %f %f %f %f %f %f %f\n", 
           values[0], values[1], values[2], values[3],
           values[4], values[5], values[6], values[7]);
    printf("t = %f in [0, %f], tReal = %f in [%f %f]\n",t,qFar,tReal,fNearFar.x,fNearFar.y);
    printf("\n");
  }
#endif

  if (t <= qFar && hit && tReal < pHit->t)
  {
    float3 norm = float3(0, 0, 1);
    if (m_preset.need_normal > 0)
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

      norm = normalize(float3(ddx, ddy, ddz));
    }
    pHit->t = tReal;
    pHit->primId = primId;
    pHit->instId = instId;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->coords[0] = 0;
    pHit->coords[1] = 0;
    pHit->coords[2] = norm.x;
    pHit->coords[3] = norm.y;

    if (m_preset.visualize_stat == VISUALIZE_STAT_SPHERE_TRACE_ITERATIONS)
      pHit->primId = iter;
  }
}

void BVHRT::IntersectAllSdfsInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{

  uint32_t type = m_geomTypeByGeomId[geomId];
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  float3 min_pos = float3(0,0,0), max_pos = float3(0,0,0);

  switch (type)
  {
#ifndef LITERT_MINI
  case TYPE_SDF_PRIMITIVE:
    sdfId = m_ConjIndices[m_geomOffsets[geomId].x + a_start];
    primId = sdfId;
    min_pos = to_float3(m_SdfConjunctions[sdfId].min_pos);
    max_pos = to_float3(m_SdfConjunctions[sdfId].max_pos);
    break;
#endif
  case TYPE_SDF_GRID:
  case TYPE_SDF_OCTREE:
    sdfId = m_geomOffsets[geomId].x;
    primId = 0;
    min_pos = float3(-1,-1,-1);
    max_pos = float3( 1, 1, 1);
    break;
  case TYPE_SDF_FRAME_OCTREE:
    sdfId =  m_geomOffsets[geomId].x;

    if (m_preset.sdf_frame_octree_blas == SDF_OCTREE_BLAS_NO)
    {
      primId = 0;
      min_pos = float3(-1,-1,-1);
      max_pos = float3( 1, 1, 1);
    }
    else if (m_preset.sdf_frame_octree_blas == SDF_OCTREE_BLAS_DEFAULT)
    {
      primId = m_origNodes[a_start].leftOffset;
      min_pos = m_origNodes[a_start].boxMin;
      max_pos = m_origNodes[a_start].boxMax;
    }
    break;
  default:
    break;
  }

  float l = length(ray_dir);
  float3 dir = ray_dir/l;
  SdfHit hit = sdf_sphere_tracing(type, sdfId, min_pos, max_pos, ray_pos, dir, m_preset.need_normal > 0);
  if (hit.hit_pos.w > 0)
  {
    float t = length(to_float3(hit.hit_pos)-ray_pos)/l;
    if (t > tNear && t < pHit->t)
    {
      pHit->t         = t;
      pHit->primId    = primId;
      pHit->instId    = instId;
      pHit->geomId    = geomId | (type << SH_TYPE);  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = hit.hit_norm.x;
      pHit->coords[3] = hit.hit_norm.y;

      if (m_preset.visualize_stat == VISUALIZE_STAT_SPHERE_TRACE_ITERATIONS)
        pHit->primId = uint32_t(hit.hit_norm.w);
    }
  }
}

int indexGrid(int x, int y, int z, int gridSize) {
    return (x + y * gridSize + z * gridSize * gridSize) * 28;
}

void lerpCellf(const float v0[28], const float v1[28], const float t, float memory[28])
{
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(v0[i], v1[i], t);
}

void BVHRT::lerpCell(const int idx0, const int idx1, const float t, float memory[28]) {
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(m_RFGridData[idx0 + i], m_RFGridData[idx1 + i], t);
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

void BVHRT::RayGridIntersection(float3 ray_pos, float3 ray_dir, float3 bbMin, float3 bbMax, uint gridSize, float3 p, float3 lastP, float &throughput, float3 &colour)
{
  float3 coords01 = (p - bbMin) / (bbMax - bbMin);
  float3 coords = coords01 * (float)(gridSize);

  int3 nearCoords = clamp((int3)coords, int3(0), int3(gridSize - 1));
  int3 farCoords = clamp((int3)coords + int3(1), int3(0), int3(gridSize - 1));

  float3 lerpFactors = coords - (float3)nearCoords;

  float xy00[28];
  float xy10[28];
  float xy01[28];
  float xy11[28];
  
  lerpCell(indexGrid(nearCoords[0], nearCoords[1], nearCoords[2], gridSize), indexGrid(farCoords[0], nearCoords[1], nearCoords[2], gridSize), lerpFactors.x, xy00);
  lerpCell(indexGrid(nearCoords[0], farCoords[1], nearCoords[2], gridSize), indexGrid(farCoords[0], farCoords[1], nearCoords[2], gridSize), lerpFactors.x, xy10);
  lerpCell(indexGrid(nearCoords[0], nearCoords[1], farCoords[2], gridSize), indexGrid(farCoords[0], nearCoords[1], farCoords[2], gridSize), lerpFactors.x, xy01);
  lerpCell(indexGrid(nearCoords[0], farCoords[1], farCoords[2], gridSize), indexGrid(farCoords[0], farCoords[1], farCoords[2], gridSize), lerpFactors.x, xy11);

  float xyz0[28];
  float xyz1[28];
  lerpCellf(xy00, xy10, lerpFactors.y, xyz0);
  lerpCellf(xy01, xy11, lerpFactors.y, xyz1);

  float gridVal[28];
  lerpCellf(xyz0, xyz1, lerpFactors.z, gridVal);

  // relu
  if (gridVal[0] < 0.0)
    gridVal[0] = 0.0;

  // for (size_t i = 0; i < 28; i++)
    // std::cout << (&grid[indexGrid(nearCoords[0], nearCoords[1], nearCoords[2], gridSize)])[i] << ' ';
  // std::cout << std::endl;

  float dist = length(p - lastP);
  // if (dist > sqrt(3) / (float)gridSize)
  //     dist -= ((int)(dist * (float)gridSize) - 1) / (float)gridSize;

  float tr = exp(-gridVal[0] * m_RFGridScales[0] * dist);

  // std::cout << tr << ' ' << gridVal[0] << ' ' << length(p - lastP) << ' ' << gridSize << std::endl;

  // float3 RGB = float3(1.0f);
  float3 RGB = float3(min(max(eval_sh(gridVal, ray_dir, 1), 0.0f), 1.0f), min(max(eval_sh(gridVal, ray_dir, 10), 0.0f), 1.0f), min(max(eval_sh(gridVal, ray_dir, 19), 0.0f), 1.0f));
  colour = colour + throughput * (1 - tr) * RGB;
  
  throughput *= tr;
}

float2 RayBoxIntersection(float3 ray_pos, float3 ray_dir, float3 boxMin, float3 boxMax)
{
  ray_dir.x = 1.0f / ray_dir.x; // may precompute if intersect many boxes
  ray_dir.y = 1.0f / ray_dir.y; // may precompute if intersect many boxes
  ray_dir.z = 1.0f / ray_dir.z; // may precompute if intersect many boxes

  float lo = ray_dir.x * (boxMin.x - ray_pos.x);
  float hi = ray_dir.x * (boxMax.x - ray_pos.x);

  float tmin = std::min(lo, hi);
  float tmax = std::max(lo, hi);

  float lo1 = ray_dir.y * (boxMin.y - ray_pos.y);
  float hi1 = ray_dir.y * (boxMax.y - ray_pos.y);

  tmin = std::max(tmin, std::min(lo1, hi1));
  tmax = std::min(tmax, std::max(lo1, hi1));

  float lo2 = ray_dir.z * (boxMin.z - ray_pos.z);
  float hi2 = ray_dir.z * (boxMax.z - ray_pos.z);

  tmin = std::max(tmin, std::min(lo2, hi2));
  tmax = std::min(tmax, std::max(lo2, hi2));

  return float2(tmin, tmax);
}

void BVHRT::IntersectRFInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{
  uint32_t type = m_geomTypeByGeomId[geomId];
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  float3 min_pos = float3(0,0,0), max_pos = float3(1,1,1);

  float l = length(ray_dir);
  float3 dir = ray_dir/l;

  auto bbox = m_origNodes[a_start];
  float2 zNearAndFar = RayBoxIntersection(ray_pos, dir, bbox.boxMin, bbox.boxMax);
  float3 p = ray_pos + dir * (zNearAndFar.x + zNearAndFar.y) / 2.0f;

  float3 lastP;
  if (pHit->adds[3] < 0.5f)
    lastP = float3(pHit->adds[0], pHit->adds[1], pHit->adds[2]);
  else
    lastP = ray_pos + dir * zNearAndFar.x;

  float throughput = pHit->coords[0];
  float3 colour = float3(pHit->coords[1], pHit->coords[2], pHit->coords[3]);

  RayGridIntersection(ray_pos, dir, min_pos, max_pos, m_RFGridSizes[0], p, lastP, throughput, colour);
  
  // std::cout << throughput << std::endl;
  
  pHit->primId = a_start;
  pHit->geomId = geomId | (type << SH_TYPE);
  pHit->coords[0] = throughput;
  pHit->coords[1] = colour[0];
  pHit->coords[2] = colour[1];
  pHit->coords[3] = colour[2];
  pHit->adds[0] = p[0];
  pHit->adds[1] = p[1];
  pHit->adds[2] = p[2];
  pHit->adds[3] = 0.0f;

  // std::cout << "Mew" << std::endl;
}

SdfHit BVHRT::sdf_sphere_tracing(uint32_t type, uint32_t sdf_id, const float3 &min_pos, const float3 &max_pos,
                                 const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  hit.hit_pos = float4(0,0,0,-1);
  hit.hit_norm = float4(1,0,0,0);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = tNear_tFar.x;
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
#ifndef LITERT_MINI
  case TYPE_SDF_PRIMITIVE:
    val = eval_dist_sdf_conjunction(sdf_id, pos);
    break;
#endif
  case TYPE_SDF_GRID:
    val = eval_distance_sdf_grid(sdf_id, pos);
    break;
  case TYPE_SDF_OCTREE:
    val = eval_distance_sdf_octree(sdf_id, pos, 1000);
    break;
  case TYPE_SDF_FRAME_OCTREE:
    val = eval_distance_sdf_frame_octree(sdf_id, pos);
    break;
  default:
    break;
  }
  return val;
}

float BVHRT::eval_distance_sdf_grid(uint32_t grid_id, float3 pos)
{
#ifndef LITERT_MINI
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
#else
  return 1000;
#endif
}

static constexpr uint32_t X_L = 1<<0;
static constexpr uint32_t X_H = 1<<1;
static constexpr uint32_t Y_L = 1<<2;
static constexpr uint32_t Y_H = 1<<3;
static constexpr uint32_t Z_L = 1<<4;
static constexpr uint32_t Z_H = 1<<5;

constexpr uint32_t INVALID_IDX = 1u<<31u;

struct SDONeighbor
{
  SdfOctreeNode node;
  uint32_t overshoot;
};

bool BVHRT::is_leaf(uint32_t offset)
{
  return (offset == 0) || ((offset & INVALID_IDX) > 0);
}

float BVHRT::eval_distance_sdf_octree(uint32_t octree_id, float3 position, uint32_t max_level)
{
#ifndef LITERT_MINI
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
#else
  return 1000;
#endif
}

#ifndef LITERT_MINI
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
      pHit->t = t;
      pHit->primId = triId;
      pHit->instId = instId;
      pHit->geomId = geomId | (TYPE_MESH_TRIANGLE << SH_TYPE);
      pHit->coords[0] = u;
      pHit->coords[1] = v;

      if (m_preset.need_normal > 0)
      {
        float3 n = normalize(cross(edge1, edge2));
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
  const float3 rayDirInv = SafeInverse(to_float3(dirAndFar));

  uint32_t stack[STACK_SIZE];

  CRT_Hit hit;
  hit.t      = dirAndFar.w;
  hit.primId = uint32_t(-1);
  hit.instId = uint32_t(-1);
  hit.geomId = uint32_t(-1);
  hit.coords[0] = 1.0f;
  hit.coords[1] = 0.0f;
  hit.coords[2] = 0.0f;
  hit.coords[3] = 0.0f;
  hit.adds[0] = 0.0f;
  hit.adds[1] = 0.0f;
  hit.adds[2] = 0.0f;
  hit.adds[3] = 1.0f;

  // std::cout << "-----------------------------------------" << std::endl;

  //no TLAS, only one instance
  if (m_nodesTLAS.size() == 1)
  {
    const float2 boxHit    = RayBoxIntersection2(to_float3(posAndNear), rayDirInv, m_nodesTLAS[0].boxMin, m_nodesTLAS[0].boxMax);
    const bool intersects  = (boxHit.x <= boxHit.y) && (boxHit.y > posAndNear.w);
    if (intersects)
    {
      const uint32_t instId = 0;
      const uint32_t geomId = m_geomIdByInstId[instId];

      // transform ray with matrix to local space
      const float3 ray_pos = matmul4x3(m_instMatricesInv[0], to_float3(posAndNear));
      const float3 ray_dir = matmul3x3(m_instMatricesInv[0], to_float3(dirAndFar));
      BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stack, stopOnFirstHit, &hit);
    }
  }
  else
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
        const uint32_t geomId = m_geomIdByInstId[instId];
    
        // transform ray with matrix to local space
        //
        const float3 ray_pos = matmul4x3(m_instMatricesInv[instId], to_float3(posAndNear));
        const float3 ray_dir = matmul3x3(m_instMatricesInv[instId], to_float3(dirAndFar)); // DON'float NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!
    
        BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stack, stopOnFirstHit, &hit);
      }
    } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //
  }

  if(hit.geomId < uint32_t(-1) && ((hit.geomId >> SH_TYPE) == TYPE_MESH_TRIANGLE)) 
  {
    const uint2 geomOffsets = m_geomOffsets[hit.geomId & 0x0FFFFFFF];
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
