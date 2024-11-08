#include <cfloat>
#include <cstring>
#include <sstream>
//#include <iomanip>   

#include "eye_ray.h"
#include "../render_common.h"

uint32_t MultiRenderer::encode_RGBA8(float4 c)
{
  uint4 col = uint4(255 * clamp(c, float4(0,0,0,0), float4(1,1,1,1)));
  return (col.w<<24) | (col.z<<16) | (col.y<<8) | col.x;
}

float4 MultiRenderer::decode_RGBA8(uint32_t c)
{
  uint4 col = uint4(c & 0xFF, (c>>8) & 0xFF, (c>>16) & 0xFF, (c>>24) & 0xFF);
  return float4(col.x * (1.0f/255.0f), col.y * (1.0f/255.0f), col.z * (1.0f/255.0f), col.w * (1.0f/255.0f));
}

//Octahedral Normal Vectors (ONV) decoding https://jcgt.org/published/0003/02/01/
float3 MultiRenderer::decode_normal(float2 e)
{
  float3 v = float3(e.x, e.y, 1.0f - std::abs(e.x) - std::abs(e.y));
  if (v.z < 0) 
  {
    float vx = v.x;
    v.x = (1.0f - std::abs(v.y)) * ((v.x >= 0.0) ? +1.0f : -1.0f);
    v.y = (1.0f - std::abs( vx)) * ((v.y >= 0.0) ? +1.0f : -1.0f);
  }
  return normalize(v);
}

uint3 pcg3d(uint3 v) 
{
    v = v * 1664525u + 1013904223u;

    v.x += v.y*v.z;
    v.y += v.z*v.x;
    v.z += v.x*v.y;

    v.x ^= v.x >> 16u;
    v.y ^= v.y >> 16u;
    v.z ^= v.z >> 16u;

    v.x += v.y*v.z;
    v.y += v.z*v.x;
    v.z += v.x*v.y;

    return v;
}

float3 MultiRenderer::rand3(uint32_t x, uint32_t y, uint32_t iter)
{
  x = x + 1233u*(iter+m_seed) % 171u;
  y = y + 453u*(iter+m_seed) % 765u;
  uint3 v = uint3(x, y, x ^ y);

  // http://www.jcgt.org/published/0009/03/02/
  v = v * 1664525u + 1013904223u;

  v.x += v.y * v.z;
  v.y += v.z * v.x;
  v.z += v.x * v.y;

  v.x ^= v.x >> 16u;
  v.y ^= v.y >> 16u;
  v.z ^= v.z >> 16u;

  v.x += v.y * v.z;
  v.y += v.z * v.x;
  v.z += v.x * v.y;

  return float3(v) * (1.0f/float(0xffffffffu));
}

float2 MultiRenderer::rand2(uint32_t x, uint32_t y, uint32_t iter)
{
  float3 r3 = rand3(x, y, iter);
  return float2(r3.x, r3.y);
}

void MultiRenderer::CastRaySingle(uint32_t tidX, uint32_t* out_color)
{
  if (tidX >= m_packedXY.size())
    return;

  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  if (x >= m_width || y >= m_height)
    return;

  float4 res_color = float4(0,0,0,0);
  uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
  float i_spp_sqrt = 1.0f/spp_sqrt;

  for (uint32_t i = 0; i < m_preset.spp; i++)
  {
    float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i + m_seed % 2) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
    float4 rayPosAndNear, rayDirAndFar;
    kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
    res_color += kernel_RayTrace(tidX, &rayPosAndNear, &rayDirAndFar);
  }
  res_color /= float4(m_preset.spp);
  uint4 col = uint4(255 * clamp(res_color, float4(0,0,0,0), float4(1,1,1,1)));
  out_color[y * m_width + x] = (col.w<<24) | (col.z<<16) | (col.y<<8) | col.x;
}

void MultiRenderer::CastRayFloatSingle(uint32_t tidX, float4* out_color)
{
  if (tidX >= m_packedXY.size())
    return;

  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  if (x >= m_width || y >= m_height)
    return;

  float4 res_color = float4(0,0,0,0);
  uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
  float i_spp_sqrt = 1.0f/spp_sqrt;

  for (uint32_t i = 0; i < m_preset.spp; i++)
  {
    float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i + m_seed % 2) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
    float4 rayPosAndNear, rayDirAndFar;
    kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
    res_color += kernel_RayTrace(tidX, &rayPosAndNear, &rayDirAndFar);
  }
  out_color[y * m_width + x] = res_color / float4(m_preset.spp);
}

//bool g_debugPrint = false;

void MultiRenderer::kernel_InitEyeRay(uint32_t tidX, float2 d, float4* rayPosAndNear, float4* rayDirAndFar)
{
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
  
  float3 rayDir = EyeRayDirNormalized((float(x)+d.x)/float(m_width), (float(y)+d.y)/float(m_height), m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, 1e9f);
}

float4 MultiRenderer::kernel_RayTrace(uint32_t tidX, const float4* rayPosAndNear,
                                      const float4* rayDirAndFar)
{
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;


  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);

  if (hit.primId == 0xFFFFFFFF) //no hit
    return float4(0,0,0,1);

  float z = hit.t;
  float z_near = 0.1;
  float z_far = 10;

  float4 res_color = float4(1,0,1,1); //if pixel is purple at the end, then something gone wrong!
  unsigned type = hit.geomId >> SH_TYPE;
  unsigned geomId = hit.geomId & GEOM_ID_MASK;
  float2 tc = float2(0, 0);
  float3 norm = float3(1,0,0);

  if (type == TYPE_MESH_TRIANGLE)
  {
#ifndef DISABLE_MESH
    const uint2 a_geomOffsets = m_geomOffsets[geomId];
    const uint32_t A = m_indices[a_geomOffsets.x + hit.primId * 3 + 0];
    const uint32_t B = m_indices[a_geomOffsets.x + hit.primId * 3 + 1];
    const uint32_t C = m_indices[a_geomOffsets.x + hit.primId * 3 + 2];

    const float2 A_tc = float2(m_vertices[a_geomOffsets.y + A].w, m_normals[a_geomOffsets.y + A].w);
    const float2 B_tc = float2(m_vertices[a_geomOffsets.y + B].w, m_normals[a_geomOffsets.y + B].w);
    const float2 C_tc = float2(m_vertices[a_geomOffsets.y + C].w, m_normals[a_geomOffsets.y + C].w);

    float3 raw_norm = float3(1, 0, 0);
    if (m_preset.normal_mode == NORMAL_MODE_GEOMETRY)
    {
      const float3 posA = to_float3(m_vertices[a_geomOffsets.y + A]);
      const float3 posB = to_float3(m_vertices[a_geomOffsets.y + B]);
      const float3 posC = to_float3(m_vertices[a_geomOffsets.y + C]);
      const float3 edge1 = posB - posA;
      const float3 edge2 = posC - posA;
      raw_norm = cross(edge1, edge2);
    }
    else if (m_preset.normal_mode == NORMAL_MODE_VERTEX)
    {
      const float4 normA = m_normals[a_geomOffsets.y + A];
      const float4 normB = m_normals[a_geomOffsets.y + B];
      const float4 normC = m_normals[a_geomOffsets.y + C];
      const float2 uv = float2(hit.coords[0], hit.coords[1]);
      const float4 norm4 = (1.0f - uv.x - uv.y) * normA + uv.y * normB + uv.x * normC;
      raw_norm = to_float3(norm4);
    }

    tc = (1.0f - hit.coords[0] - hit.coords[1]) * A_tc + hit.coords[1] * B_tc + hit.coords[0] * C_tc;
    norm = normalize(matmul4x3(m_instanceTransformInvTransposed[hit.instId], raw_norm));
#endif
  }
  else
  {
    tc = float2(hit.coords[0], hit.coords[1]);
    norm = decode_normal(float2(hit.coords[2], hit.coords[3]));
  }

  float norm_sign = sign(dot(-1.0f*to_float3(rayDir), norm));
  norm = norm * norm_sign;

  switch (m_preset.render_mode)
  {
  case MULTI_RENDER_MODE_MASK:
    res_color = float4(1, 1, 1, 1);
    break;

  case MULTI_RENDER_MODE_DEPTH:
  {
    float d = (1 / z - 1 / z_near) / (1 / z_far - 1 / z_near);
    res_color = float4(d, d, d, 1);
  }
  break;

  case MULTI_RENDER_MODE_LINEAR_DEPTH:
  {
    float d = ((z - z_near) / (z_far - z_near));
    res_color = float4(d, d, d, 1);
  }
  break;

  case MULTI_RENDER_MODE_HSV_DEPTH:
  {
    float3 isect_pt = to_float3(rayPos + z * rayDir);
    float z_transformed = (isect_pt.z + 1.f) * 2.f;
    z_transformed = z_transformed - int(z_transformed);
    if (z_transformed < 0.f)
      z_transformed = 1.f + z_transformed;
    float3 hsv_col = float3(z_transformed, 1.f, 1.f);

    // HSV -> RGB
    res_color = float4(hsv_col.z, hsv_col.z, hsv_col.z, 1.f);
    float Vmin = (1.f - hsv_col.y) * hsv_col.z;
    float H_i = 0.f;
    float a = (hsv_col.z - Vmin) * std::modf(hsv_col.x * 6.f, &H_i);
    uint32_t H_i_int = uint32_t(H_i);
    H_i_int = H_i_int <= 5u ? H_i_int : 5u;

    res_color[(2u + (H_i_int >> 1)) % 3u] = Vmin;
    if ((H_i_int & 1u) != 0u)
      res_color[H_i_int >> 1] = hsv_col.z - a;
    else
      res_color[(1u + (H_i_int >> 1)) % 3u] = Vmin + a;
    res_color.w = 1.f;
  }
  break;

  case MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH:
  {
    float d = 1 - ((z - z_near) / (z_far - z_near));
    res_color = float4(d, d, d, 1);
  }
  break;

  case MULTI_RENDER_MODE_PRIMITIVE:
  {
    res_color = decode_RGBA8(m_palette[(hit.primId) % palette_size]);
  }
  break;

  case MULTI_RENDER_MODE_TYPE:
  {
    res_color = decode_RGBA8(m_palette[type % palette_size]);
  }
  break;

  case MULTI_RENDER_MODE_GEOM:
  {
    res_color = decode_RGBA8(m_palette[geomId % palette_size]);
  }
  break;

  case MULTI_RENDER_MODE_NORMAL:
  {
    res_color = to_float4(abs(norm), 1);
  }
  break;

  case MULTI_RENDER_MODE_BARYCENTRIC:
  {
    res_color = float4(hit.coords[0], hit.coords[1], 1 - hit.coords[0] - hit.coords[1], 1);
  }
  break;

  case MULTI_RENDER_MODE_ST_ITERATIONS:
  {
    if (hit.primId == 0)
    {
      res_color = float4(0.5, 0.5, 0.5, 1);
    }
    else if (hit.primId <= 64)
    {
      float q = clamp(float(hit.primId) / 64, 0.0f, 1.0f);
      res_color = q * float4(0, 0, 1, 1) + (1 - q) * float4(0, 1, 0, 1);
    }
    else
    {
      float q = clamp(float(hit.primId - 64) / (256 - 64), 0.0f, 1.0f);
      res_color = q * float4(1, 0, 0, 1) + (1 - q) * float4(0, 0, 1, 1);
    }
  }
  break;

  case MULTI_RENDER_MODE_RF:
  {
    res_color = clamp(float4(hit.coords[1], hit.coords[2], hit.coords[3], 1.0f), 0.0f, 1.0f);
  }
  break;

  case MULTI_RENDER_MODE_GS:
  {
    res_color = clamp(float4(hit.coords[1], hit.coords[2], hit.coords[3], 1.0f), 0.0f, 1.0f);
  }
  break;

  case MULTI_RENDER_MODE_RF_DENSITY:
  {
    res_color = float4(1.0f - hit.coords[0], 1.0f - hit.coords[0], 1.0f - hit.coords[0], 1);
  }
  break;

  case MULTI_RENDER_MODE_TEX_COORDS:
  {
    res_color = float4(tc.x, tc.y, 0, 1);
  }
  break;

  case MULTI_RENDER_MODE_DIFFUSE:
  {
    float4 color = float4(0,0,1,1);
    if (type == TYPE_SDF_SBS_COL || type == TYPE_SDF_SBS_ADAPT_COL)
    {
      color.x = std::round(hit.coords[0])/255.0f;
      color.y = fract(hit.coords[0]);
      color.z = std::round(hit.coords[1])/255.0f;
    }
    else
    {
      unsigned matId = m_matIdbyPrimId[m_matIdOffsets[geomId].x + hit.primId % m_matIdOffsets[geomId].y];
      color = m_materials[matId].type == MULTI_RENDER_MATERIAL_TYPE_COLORED ? m_materials[matId].base_color : m_textures[m_materials[matId].texId]->sample(tc);
    }
    res_color = to_float4(to_float3(color), 1);
  }
  break;

  case MULTI_RENDER_MODE_LAMBERT:
  case MULTI_RENDER_MODE_LAMBERT_NO_TEX:
  {
    float3 color = float3(0,0,1);
    if (m_preset.render_mode == MULTI_RENDER_MODE_LAMBERT_NO_TEX)
      color = float3(1,1,1);
    else if (type == TYPE_SDF_SBS_COL || type == TYPE_SDF_SBS_ADAPT_COL || type == TYPE_GRAPHICS_PRIM)
    {
      color.x = std::round(hit.coords[0])/255.0f;
      color.y = fract(hit.coords[0]);
      color.z = std::round(hit.coords[1])/255.0f;
    }
    else
    {
      unsigned matId = m_matIdbyPrimId[m_matIdOffsets[geomId].x + hit.primId % m_matIdOffsets[geomId].y];
      color = to_float3(m_materials[matId].type == MULTI_RENDER_MATERIAL_TYPE_COLORED ? 
                        m_materials[matId].base_color : m_textures[m_materials[matId].texId]->sample(tc));
    }

    float3 final_color = float3(0,0,0);
    for (int i=0;i<m_lights.size();i++)
    {
      if (m_lights[i].type == LIGHT_TYPE_DIRECT)
      {
        float q = max(0.0f, dot(norm, m_lights[i].space));
        final_color += m_lights[i].color * color * q;
      }
      else if (m_lights[i].type == LIGHT_TYPE_POINT)
      {
        float3 surf_pos = to_float3(rayPos) + hit.t * to_float3(rayDir);
        float3 dir = m_lights[i].space - surf_pos;
        float l = length(dir);
        dir /= l;
        float q = max(0.0f, dot(norm, dir));
        final_color += m_lights[i].color * color * q / (l*l);
      }
      else
        final_color += m_lights[i].color * color;
    }

    res_color = to_float4(final_color, 1);
  }
  break;

  case MULTI_RENDER_MODE_PHONG:
  case MULTI_RENDER_MODE_PHONG_NO_TEX:
  {
    const float Kd = 0.25;
    const float Ks = 0.25;
    const int spec_pow = 32;
    const float BIAS = 1e-6f;

    float3 color = float3(0,0,1);
    if (m_preset.render_mode == MULTI_RENDER_MODE_PHONG_NO_TEX)
      color = float3(1,1,1);
    else if (type == TYPE_SDF_SBS_COL || type == TYPE_SDF_SBS_ADAPT_COL || type == TYPE_GRAPHICS_PRIM)
    {
      color.x = std::round(hit.coords[0])/255.0f;
      color.y = fract(hit.coords[0]);
      color.z = std::round(hit.coords[1])/255.0f;
    }
    else
    {
      unsigned matId = m_matIdbyPrimId[m_matIdOffsets[geomId].x + hit.primId % m_matIdOffsets[geomId].y];
      color = to_float3(m_materials[matId].type == MULTI_RENDER_MATERIAL_TYPE_COLORED ? 
                        m_materials[matId].base_color : m_textures[m_materials[matId].texId]->sample(tc));
    }

    float3 final_color = float3(0,0,0);
    for (int i=0;i<m_lights.size();i++)
    {
      if (m_lights[i].type == LIGHT_TYPE_AMBIENT)
      {
        final_color += m_lights[i].color * color;
      }
      else
      {
        float3 surf_pos = to_float3(rayPos) + (hit.t - BIAS) * to_float3(rayDir);
        float3 light_dir = float3(1,1,1);
        float light_dist = 1.0f;
        if (m_lights[i].type == LIGHT_TYPE_DIRECT)
        {
          light_dir = m_lights[i].space;
          light_dist = 1.0f;
          float q = max(0.0f, dot(norm, m_lights[i].space));
          final_color += m_lights[i].color * color * q;
        }
        else if (m_lights[i].type == LIGHT_TYPE_POINT)
        {
          float3 dir = m_lights[i].space - surf_pos;
          float l = length(dir);
          
          light_dir  = dir / l;
          light_dist = l;
        }

        CRT_Hit shadowHit = m_pAccelStruct->RayQuery_NearestHit(to_float4(surf_pos, rayPos.w), to_float4(light_dir, rayDir.w));
        float shade = (shadowHit.primId == 0xFFFFFFFF) ? 1 : 0;
        float3 view_dir = to_float3(rayDir);
        float3 reflect = -1.0f*light_dir + 2.0f * dot(norm, light_dir) * norm;
        float3 f_col = (shade * m_lights[i].color * (Kd * std::max(0.0f, dot(norm, light_dir)) + Ks * pow(std::max(0.0f, dot(norm, reflect)), spec_pow))) * color;
        final_color += f_col;
      }
    }

    res_color = to_float4(final_color, 1);    
  }
  break;

  default:
    break;
  }
  return res_color;
}

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
  if (tidX >= m_packedXY_width || tidY >= m_packedXY_height)
    return;

  const uint offset   = SuperBlockIndex2DOpt(tidX, tidY, m_packedXY_width);
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
  PackXYBlock(m_packedXY_width, m_packedXY_height, 1);
}
