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

void MultiRenderer::CastRaySingle(uint32_t tidX, uint32_t* out_color)
{
  if (tidX >= m_packedXY.size())
    return;

  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tidX, &rayPosAndNear, &rayDirAndFar);
  float4 res_color = kernel_RayTrace(tidX, &rayPosAndNear, &rayDirAndFar);
  
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

  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tidX, &rayPosAndNear, &rayDirAndFar);
  out_color[y * m_width + x] = kernel_RayTrace(tidX, &rayPosAndNear, &rayDirAndFar);
}

//bool g_debugPrint = false;

void MultiRenderer::kernel_InitEyeRay(uint32_t tidX, float4* rayPosAndNear, float4* rayDirAndFar)
{
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
  
  float3 rayDir = EyeRayDirNormalized((float(x)+0.5f)/float(m_width), (float(y)+0.5f)/float(m_height), m_projInv);
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


#ifndef DISABLE_GS_PRIMITIVE
  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHitGS(rayPos, rayDir);
#else
  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);
#endif

  if (hit.primId == 0xFFFFFFFF) //no hit
    return float4(0,0,0,1);

  float z = hit.t;
  float z_near = 0.1;
  float z_far = 10;

  float4 res_color = float4(1,0,1,1); //if pixel is purple at the end, then something gone wrong!
  unsigned type = hit.geomId >> SH_TYPE;
  unsigned geomId = hit.geomId & 0x0FFFFFFF;
  float2 tc = float2(0, 0);
  float3 norm = float3(1,0,0);

  if (type == TYPE_MESH_TRIANGLE)
  {
    const uint2 a_geomOffsets = m_geomOffsets[geomId];
    const uint32_t A = m_indices[a_geomOffsets.x + hit.primId * 3 + 0];
    const uint32_t B = m_indices[a_geomOffsets.x + hit.primId * 3 + 1];
    const uint32_t C = m_indices[a_geomOffsets.x + hit.primId * 3 + 2];

    const float2 A_tc = float2(m_vertices[a_geomOffsets.y + A].w, m_normals[a_geomOffsets.y + A].w);
    const float2 B_tc = float2(m_vertices[a_geomOffsets.y + B].w, m_normals[a_geomOffsets.y + B].w);
    const float2 C_tc = float2(m_vertices[a_geomOffsets.y + C].w, m_normals[a_geomOffsets.y + C].w);

    const float4 normA = m_normals[a_geomOffsets.y + A];
    const float4 normB = m_normals[a_geomOffsets.y + B];
    const float4 normC = m_normals[a_geomOffsets.y + C];
    const float2 uv    = float2(hit.coords[0], hit.coords[1]);
    const float4 norm4 = (1.0f - uv.x - uv.y)*normA + uv.y*normB + uv.x*normC;

    tc = (1.0f - hit.coords[0] - hit.coords[1]) * A_tc + hit.coords[1] * B_tc + hit.coords[0] * C_tc;
    norm = to_float3(norm4);
    // const uint2 a_geomOffsets = m_pAccelStruct-> m_geomData[geomId].offset;
  }
  else
  {
    tc = float2(hit.coords[0], hit.coords[1]);
    norm = float3(hit.coords[2], hit.coords[3], sqrt(max(0.0f, 1 - hit.coords[2] * hit.coords[2] - hit.coords[3] * hit.coords[3])));
  }

  switch (m_preset.render_mode)
  {
  case MULTI_RENDER_MODE_MASK:
    res_color = float4(1, 1, 1, 1);
    break;

  case MULTI_RENDER_MODE_LAMBERT_NO_TEX:
  {
    float q = max(0.1f, dot(norm, normalize(float3(1, 1, 1))));
    res_color = float4(q, q, q, 1);
  }
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

  case MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH:
  {
    float d = 1 - ((z - z_near) / (z_far - z_near));
    res_color = float4(d, d, d, 1);
  }
  break;

  case MULTI_RENDER_MODE_PRIMIVIVE:
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

  case MULTI_RENDER_MODE_PHONG_NO_TEX:
  {
    const float3 ambient_light_color = float3(1, 1, 1);
    const float Ka = 0.1;
    const float Kd = 1;
    const float Ks = 1;
    const int spec_pow = 32;
    const float BIAS = 1e-6f;

    float3 diffuse = float3(1, 1, 1);
    float3 light_dir = -1.0f * to_float3(m_mainLightDir);
    float3 light_color = to_float3(m_mainLightColor);

    float3 surf_pos = to_float3(rayPos) + (hit.t - BIAS) * to_float3(rayDir);
    CRT_Hit shadowHit = m_pAccelStruct->RayQuery_NearestHit(to_float4(surf_pos, rayPos.w), to_float4(-1.0f * light_dir, rayDir.w));
    float shade = (shadowHit.primId == 0xFFFFFFFF) ? 1 : 0;
    float3 view_dir = to_float3(rayDir);
    float3 reflect = light_dir - 2.0f * dot(norm, light_dir) * norm;
    float3 f_col = (shade * light_color * (Kd * std::max(0.0f, dot(norm, -1.0f * light_dir)) + Ks * pow(std::max(0.0f, dot(norm, reflect)), spec_pow)) +
                    ambient_light_color * Ka) *
                   diffuse;

    res_color = to_float4(clamp(f_col, 0.0f, 1.0f), 1);
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
    if (type == TYPE_SDF_SBS_COL)
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
  {
    float4 color = float4(0,0,1,1);
    if (type == TYPE_SDF_SBS_COL)
    {
      color.x = std::round(hit.coords[0])/255.0f;
      color.y = fract(hit.coords[0]);
      color.z = std::round(hit.coords[1])/255.0f;
    }
    else
    {
      unsigned matId = m_matIdbyPrimId[m_matIdOffsets[geomId].x + hit.primId % m_matIdOffsets[geomId].y];
      color = m_materials[matId].type == MULTI_RENDER_MATERIAL_TYPE_COLORED ? m_materials[matId].base_color : m_textures[m_materials[matId].texId]->sample(tc);
      //color = float4(1,0,1,0);
    }

    float q = max(0.1f, dot(norm, normalize(float3(1, 1, 1))));
    res_color = to_float4(q * to_float3(color), 1);
  }
  break;

  case MULTI_RENDER_MODE_PHONG:
  {
    const float3 ambient_light_color = float3(1, 1, 1);
    const float Ka = 0.1;
    const float Kd = 1;
    const float Ks = 1;
    const int spec_pow = 32;
    const float BIAS = 1e-6f;

    float4 color = float4(0,0,1,1);
    if (type == TYPE_SDF_SBS_COL)
    {
      color.x = std::round(hit.coords[0])/255.0f;
      color.y = fract(hit.coords[0]);
      color.z = std::round(hit.coords[1])/255.0f;
    }
    else
    {
      unsigned matId = m_matIdbyPrimId[m_matIdOffsets[geomId].x + hit.primId % m_matIdOffsets[geomId].y];
      color = m_materials[matId].type == MULTI_RENDER_MATERIAL_TYPE_COLORED ? m_materials[matId].base_color : m_textures[m_materials[matId].texId]->sample(tc);
      //color = float4(1,0,1,0);
    }

    float3 diffuse = to_float3(color);
    float3 light_dir = -1.0f * to_float3(m_mainLightDir);
    float3 light_color = to_float3(m_mainLightColor);

    float3 surf_pos = to_float3(rayPos) + (hit.t - BIAS) * to_float3(rayDir);
    CRT_Hit shadowHit = m_pAccelStruct->RayQuery_NearestHit(to_float4(surf_pos, rayPos.w), to_float4(-1.0f * light_dir, rayDir.w));
    float shade = (shadowHit.primId == 0xFFFFFFFF) ? 1 : 0;
    float3 view_dir = to_float3(rayDir);
    float3 reflect = light_dir - 2.0f * dot(norm, light_dir) * norm;
    float3 f_col = (shade * light_color * (Kd * std::max(0.0f, dot(norm, -1.0f * light_dir)) + Ks * pow(std::max(0.0f, dot(norm, reflect)), spec_pow)) +
                    ambient_light_color * Ka) *
                   diffuse;

    res_color = to_float4(f_col, 1);
  }
  break;

  default:
    break;
  }
  return res_color;
}

//ERROR LOG
/*
(6) Calc offsets for all class members; ingore unused members that were not marked on previous step
{
  placed classVariables num = 77
}


Program received signal SIGBUS, Bus error.
0x00005555559505da in main (argc=<optimized out>, argv=<optimized out>) at /home/sammael/kernel_slicer/kslicer_main.cpp:805
805           kernel.wgSize[0] = defaultWgSize[kernelDim-1][0];
(gdb) print kernelDim
$1 = 0
(gdb) print kernel
$2 = (kslicer::KernelInfo &) @0x555558c400a8: {return_type = "void", return_class = "", name = "kernel_InitEyeRay", 
  className = "MultiRenderer", interfaceName = "", args = std::vector of length 4, capacity 4 = {{type = "uint32_t", 
      name = "x", size = 1, kind = kslicer::DATA_KIND::KIND_POD, needFakeOffset = false, isThreadID = false, 
      isLoopSize = false, isThreadFlags = false, isReference = false, isContainer = false, containerType = "", 
      containerDataType = ""}, {type = "uint32_t", name = "y", size = 1, kind = kslicer::DATA_KIND::KIND_POD, 
      needFakeOffset = false, isThreadID = false, isLoopSize = false, isThreadFlags = false, isReference = false, 
      isContainer = false, containerType = "", containerDataType = ""}, {type = "float4 *", name = "rayPosAndNear", size = 1, 
      kind = kslicer::DATA_KIND::KIND_POINTER, needFakeOffset = true, isThreadID = false, isLoopSize = false, 
      isThreadFlags = false, isReference = false, isContainer = false, containerType = "", containerDataType = ""}, {
      type = "float4 *", name = "rayDirAndFar", size = 1, kind = kslicer::DATA_KIND::KIND_POINTER, needFakeOffset = true, 
      isThreadID = false, isLoopSize = false, isThreadFlags = false, isReference = false, isContainer = false, 
      containerType = "", containerDataType = ""}}, loopIters = std::vector of length 0, capacity 0, 
  debugOriginalText = "void MultiRenderer::kernel_InitEyeRay(uint32_t x, uint32_t y, float4* rayPosAndNear, float4* rayDirAndFar)\n{\n  float3 rayDir = EyeRayDirNormalized((float(x)+0.5f)/float(m_width), (float(y)+0.5f)/float"..., loopInsides = {B = {
      ID = 0}, E = {ID = 0}}, loopOutsidesInit = {B = {ID = 0}, E = {ID = 0}}, loopOutsidesFinish = {B = {ID = 0}, E = {
      ID = 0}}, hasInitPass = false, hasFinishPass = false, hasFinishPassSelf = false, astNode = 0x555557ddd9b8, 
  usedInMainFunc = true, isBoolTyped = false, usedInExitExpr = false, checkThreadFlags = false, isMega = false, 
  RetType = "void ", DeclCmd = "InitEyeRayCmd(uint32_t x, uint32_t y, float4* rayPosAndNear, float4* rayDirAndFar)", 
  usedContainers = std::unordered_map with 13 elements = {["m_pAccelStruct_m_allNodePairs"] = {
      type = "std::vector<BVHNodePair>", name = "m_pAccelStruct_m_allNodePairs", kind = kslicer::DATA_KIND::KIND_VECTOR, 
      isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, ["m_pAccelStruct_m_origNodes"] = {
--Type <RET> for more, q to quit, c to continue without paging--
      type = "std::vector<BVHNode>", name = "m_pAccelStruct_m_origNodes", kind = kslicer::DATA_KIND::KIND_VECTOR, 
      isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, ["m_pAccelStruct_m_primIndices"] = {
      type = "std::vector<uint32_t>", name = "m_pAccelStruct_m_primIndices", kind = kslicer::DATA_KIND::KIND_VECTOR, 
      isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, ["m_pAccelStruct_m_nodesTLAS"] = {
      type = "std::vector<BVHNode>", name = "m_pAccelStruct_m_nodesTLAS", kind = kslicer::DATA_KIND::KIND_VECTOR, 
      isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_SdfFrameOctreeTexRoots"] = {type = "std::vector<uint32_t>", 
      name = "m_pAccelStruct_m_SdfFrameOctreeTexRoots", kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, 
      isSetter = false, setterPrefix = "", setterSuffix = ""}, ["m_pAccelStruct_m_SdfFrameOctreeTexNodes"] = {
      type = "std::vector<SdfFrameOctreeTexNode>", name = "m_pAccelStruct_m_SdfFrameOctreeTexNodes", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_SdfSVSNodes"] = {type = "std::vector<SdfSVSNode>", name = "m_pAccelStruct_m_SdfSVSNodes", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_geomData"] = {type = "std::vector<GeomData>", name = "m_pAccelStruct_m_geomData", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_indices"] = {type = "std::vector<uint32_t>", name = "m_pAccelStruct_m_indices", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_SdfSVSRoots"] = {type = "std::vector<uint32_t>", name = "m_pAccelStruct_m_SdfSVSRoots", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_vertPos"] = {type = "std::vector<float4>", name = "m_pAccelStruct_m_vertPos", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
--Type <RET> for more, q to quit, c to continue without paging--
    ["m_pAccelStruct_m_vertNorm"] = {type = "std::vector<float4>", name = "m_pAccelStruct_m_vertNorm", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}, 
    ["m_pAccelStruct_m_instanceData"] = {type = "std::vector<InstanceData>", name = "m_pAccelStruct_m_instanceData", 
      kind = kslicer::DATA_KIND::KIND_VECTOR, isConst = false, isSetter = false, setterPrefix = "", setterSuffix = ""}}, 
  usedMembers = std::unordered_set with 18 elements = {[0] = "m_pAccelStruct_m_nodesTLAS", 
    [1] = "m_pAccelStruct_m_allNodePairs", [2] = "m_pAccelStruct_m_origNodes", [3] = "m_width", [4] = "m_height", 
    [5] = "m_pAccelStruct_m_primIndices", [6] = "m_projInv", [7] = "m_pAccelStruct_m_vertPos", [8] = "m_worldViewInv", 
    [9] = "m_pAccelStruct_m_SdfSVSRoots", [10] = "m_pAccelStruct_m_instanceData", 
    [11] = "m_pAccelStruct_m_SdfFrameOctreeTexRoots", [12] = "m_pAccelStruct_m_preset", [13] = "m_pAccelStruct_m_vertNorm", 
    [14] = "m_pAccelStruct_m_indices", [15] = "m_pAccelStruct_m_geomData", [16] = "m_pAccelStruct_m_SdfSVSNodes", 
    [17] = "m_pAccelStruct_m_SdfFrameOctreeTexNodes"}, subjectedToReduction = std::unordered_map with 0 elements, 
  texAccessInArgs = std::unordered_map with 0 elements, texAccessInMemb = std::unordered_map with 0 elements, 
  texAccessSampler = std::unordered_map with 0 elements, shittyFunctions = std::vector of length 0, capacity 0, 
  subkernels = std::vector of length 0, capacity 0, currentShit = {pointers = std::vector of length 0, capacity 0, 
    originalName = ""}, threadLocalArrays = std::unordered_map with 0 elements, be = {
    sharedDecls = std::vector of length 0, capacity 0, statements = std::vector of length 0, capacity 0, enabled = false, 
    wgNames = {"unknown", "unknown", "unknown"}, wgTypes = {"uint", "uint", "uint"}}, rewrittenText = "", rewrittenInit = "", 
  rewrittenFinish = "", wgSize = {256, 1, 1}, warpSize = 32, enableSubGroups = false, enableRTPipeline = false, 
  singleThreadISPC = false, openMpAndISPC = false, explicitIdISPC = false, isIndirect = false, indirectBlockOffset = 0, 
  indirectMakerOffset = 0, shaderFeatures = {useByteType = false, useShortType = false, useInt64Type = false, 
    useFloat64Type = false, useHalfType = false}}
(gdb) 
*/

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
  if (tidX >= m_width || tidY >= m_height)
    return;
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
