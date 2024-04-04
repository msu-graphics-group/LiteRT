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
  switch (m_presets.mode)
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