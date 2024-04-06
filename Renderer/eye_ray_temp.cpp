////////////////////////////////////////////////////
//// input file: /home/sammael/grade/modules/LiteRT/Renderer/eye_ray.cpp
////////////////////////////////////////////////////
#include <cfloat>
#include <cstring>
#include <sstream>
//#include <iomanip>   

#include "eye_ray.h"
#include "../render_common.h"


void MultiRenderer::CastRaySingle(uint32_t tidX, uint32_t* out_color)
{
  //const uint XY = m_pakedXY[tidX];
  //const uint x  = (XY & 0x0000FFFF);
  //const uint y  = (XY & 0xFFFF0000) >> 16;
  //if(x >= 50 && y == 450)
  //{
  //  int a = 2; // put debug breakpoint here
  //}
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tidX, &rayPosAndNear, &rayDirAndFar);
  kernel_RayTrace  (tidX, &rayPosAndNear, &rayDirAndFar, out_color);
}

//bool g_debugPrint = false;

void MultiRenderer::kernel_InitEyeRay(uint32_t tidX, float4* rayPosAndNear, float4* rayDirAndFar)
{
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
  
  //if(x == 37 && y == 450)
  //{
  //  g_debugPrint = true;
  //}
  //else
  //  g_debugPrint = false;

  float3 rayDir = EyeRayDirNormalized((float(x)+0.5f)/float(m_width), (float(y)+0.5f)/float(m_height), m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, 1e9f);
}

void MultiRenderer::kernel_RayTrace(uint32_t tidX, const float4* rayPosAndNear,
                                   const float4* rayDirAndFar, uint32_t* out_color)
{
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;
  
  CRT_Hit hit   = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  if (hit.primId == 0xFFFFFFFF) //no hit
  {
    out_color[y * m_width + x] = 0;
    return;
  }
  else
  {
    float3 norm(hit.coords[2], hit.coords[3], sqrt(max(0.0f, 1-hit.coords[2]*hit.coords[2] - hit.coords[3]*hit.coords[3])));
    float q = max(0.1f, dot(norm, normalize(float3(1,1,1))));
    uint32_t col= uint32_t(255*q);
    out_color[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col;
    //out_color[y * m_width + x] = m_palette[(hit.primId) % palette_size];
  } 

  float z = hit.t;
  float z_near = 0.1;
  float z_far = 10;
  out_color[y * m_width + x] = 0xFFFF00FF; //if pixel is purple at the end, then something gone wrong!
  switch (m_preset.mode)
  {
    case MULTI_RENDER_MODE_MASK:
    out_color[y * m_width + x] = 0xFFFFFFFF;
    break;

    case MULTI_RENDER_MODE_LAMBERT:
    {
      float3 norm(hit.coords[2], hit.coords[3], sqrt(max(0.0f, 1-hit.coords[2]*hit.coords[2] - hit.coords[3]*hit.coords[3])));
      float q = max(0.1f, dot(norm, normalize(float3(1,1,1))));
      uint32_t col= uint32_t(255*q);
      out_color[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col;      
    }
    break;

    case MULTI_RENDER_MODE_DEPTH:
    {
      float d = (1 / z - 1 / z_near) / (1 / z_far - 1 / z_near);
      uint32_t col= uint32_t(255*d);
      out_color[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col;   
    }
    break;

    case MULTI_RENDER_MODE_LINEAR_DEPTH:
    {
      float d = ((z - z_near) / (z_far - z_near));
      uint32_t col= uint32_t(255*d);
      out_color[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col;   
    }
    break;

    case MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH:
    {
      float d = 1 - ((z - z_near) / (z_far - z_near));
      uint32_t col= uint32_t(255*d);
      out_color[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col;   
    }
    break;

    case MULTI_RENDER_MODE_PRIMIVIVE:
    out_color[y * m_width + x] = m_palette[(hit.primId) % palette_size];
    break;

    case MULTI_RENDER_MODE_TYPE:
    {
    unsigned type = hit.geomId >> SH_TYPE;
      out_color[y * m_width + x] = m_palette[type % palette_size];
    }
    break;

    case MULTI_RENDER_MODE_GEOM:
    {
      unsigned geomId = hit.geomId & 0x0FFFFFFF;
      out_color[y * m_width + x] = m_palette[geomId % palette_size];
    }
    break;

    case MULTI_RENDER_MODE_NORMAL:
    {
      float3 norm(hit.coords[2], hit.coords[3], sqrt(max(0.0f, 1-hit.coords[2]*hit.coords[2] - hit.coords[3]*hit.coords[3])));
      uint3 col = uint3(255*abs(norm));
      out_color[y * m_width + x] = 0xFF000000 | (col.z<<16) | (col.y<<8) | col.x; 
    }
    break;

    case MULTI_RENDER_MODE_BARYCENTRIC:
    {
      uint3 col = uint3(255*float3(hit.coords[0], hit.coords[1], 1-hit.coords[0]-hit.coords[1]));
      out_color[y * m_width + x] = 0xFF000000 | (col.z<<16) | (col.y<<8) | col.x; 
    }
    break;
    case MULTI_RENDER_MODE_SPHERE_TRACE_ITERATIONS:
    {
      float3 c1 = float3(0,1,0);
      float3 c2 = float3(1,0,0);
      float q = clamp(0.2*log2(float(hit.primId)), 0.0f,1.0f);
      uint3 col = uint3(255*(q*c1 + (1-q)*c2));
      out_color[y * m_width + x] = 0xFF000000 | (col.z<<16) | (col.y<<8) | col.x; 
    }
    break;
    default:
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline uint BlockIndex2D(uint tidX, uint tidY, uint a_width)
{
  const uint inBlockIdX = tidX % 4; // 4x4 blocks
  const uint inBlockIdY = tidY % 4; // 4x4 blocks
 
  const uint localIndex = inBlockIdY*4 + inBlockIdX;
  const uint wBlocks    = a_width/4;

  const uint blockX     = tidX/4;
  const uint blockY     = tidY/4;
  const uint offset     = (blockX + blockY*wBlocks)*4*4 + localIndex;
  return offset;
}

static inline uint SuperBlockIndex2D(uint tidX, uint tidY, uint a_width)
{
  const uint inBlockIdX = tidX % 4; // 4x4 blocks
  const uint inBlockIdY = tidY % 4; // 4x4 blocks
  const uint localIndex = inBlockIdY*4 + inBlockIdX;
  const uint wBlocks    = a_width/4;
  const uint blockX     = tidX/4;
  const uint blockY     = tidY/4;
  
  const uint inHBlockIdX = blockX % 2; // 2x2 SuperBlocks
  const uint inHBlockIdY = blockY % 2; // 2x2 SuperBlocks
  const uint localIndexH = inHBlockIdY*2 + inHBlockIdX;
  const uint wBlocksH    = wBlocks/2;
  const uint blockHX     = blockX/2;
  const uint blockHY     = blockY/2;

  return (blockHX + blockHY*wBlocksH)*8*8 + localIndexH*4*4 + localIndex;
}

static inline uint SuperBlockIndex2DOpt(uint tidX, uint tidY, uint a_width)
{
  const uint inBlockIdX = tidX & 0x00000003; // 4x4 blocks
  const uint inBlockIdY = tidY & 0x00000003; // 4x4 blocks
  const uint localIndex = inBlockIdY*4 + inBlockIdX;
  const uint wBlocks    = a_width >> 2;
  const uint blockX     = tidX    >> 2;
  const uint blockY     = tidY    >> 2;
  
  const uint inHBlockIdX = blockX & 0x00000001; // 2x2 SuperBlocks
  const uint inHBlockIdY = blockY & 0x00000001; // 2x2 SuperBlocks
  const uint localIndexH = inHBlockIdY*2 + inHBlockIdX;
  const uint wBlocksH    = wBlocks >> 1;
  const uint blockHX     = blockX  >> 1;
  const uint blockHY     = blockY  >> 1;

  return (blockHX + blockHY*wBlocksH)*64 + localIndexH*16 + localIndex;
}

void MultiRenderer::kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY)
{
  //const uint offset   = BlockIndex2D(tidX, tidY, m_width);
  const uint offset   = SuperBlockIndex2DOpt(tidX, tidY, m_width);
  out_pakedXY[offset] = ((tidY << 16) & 0xFFFF0000) | (tidX & 0x0000FFFF);
}

void MultiRenderer::PackXY(uint tidX, uint tidY)
{
  kernel_PackXY(tidX, tidY, m_packedXY.data());
}

void MultiRenderer::PackXYBlock(uint tidX, uint tidY, uint a_passNum)
{
  //for(int y=0; y < 16; y++) {
  //  for(int x = 0; x < 16; x++) {
  //    std::cout << std::setfill('0') << std::setw(2) << SuperBlockIndex2DOpt(x,y,m_width) << " ";
  //  }
  //  std::cout << std::endl;
  //}

  #pragma omp parallel for default(shared)
  for(int y=0;y<tidY;y++)
    for(int x=0;x<tidX;x++)
      PackXY(x, y);
}

void MultiRenderer::Clear(uint32_t a_width, uint32_t a_height, const char* a_what)
{
  PackXYBlock(a_width, a_height, 1);
}
////////////////////////////////////////////////////
//// input file: /home/sammael/grade/modules/LiteRT/BVH/BVH2Common.cpp
////////////////////////////////////////////////////
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
  case TYPE_SDF_FRAME_OCTREE:
    IntersectAllSdfsInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
    break;
  default:
    break;
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
  case TYPE_SDF_PRIMITIVE:
    sdfId = m_ConjIndices[m_geomOffsets[geomId].x + a_start];
    primId = sdfId;
    min_pos = to_float3(m_SdfConjunctions[sdfId].min_pos);
    max_pos = to_float3(m_SdfConjunctions[sdfId].max_pos);
    break;
  case TYPE_SDF_GRID:
  case TYPE_SDF_OCTREE:
    sdfId = m_geomOffsets[geomId].x;
    primId = 0;
    min_pos = float3(-1,-1,-1);
    max_pos = float3( 1, 1, 1);
    break;
  case TYPE_SDF_FRAME_OCTREE:
    sdfId =  m_geomOffsets[geomId].x;
    primId = m_origNodes[a_start].leftOffset;
    min_pos = m_origNodes[a_start].boxMin;
    max_pos = m_origNodes[a_start].boxMax;
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
  case TYPE_SDF_PRIMITIVE:
    val = eval_dist_sdf_conjunction(sdf_id, pos);
    break;
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
      const float3 ray_dir = matmul3x3(m_instMatricesInv[instId], to_float3(dirAndFar)); // DON'float NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!
  
      BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stack, stopOnFirstHit, &hit);
    }

  } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //

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
