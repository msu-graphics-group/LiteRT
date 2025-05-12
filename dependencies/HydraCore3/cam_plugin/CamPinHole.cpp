#include "CamPinHole.h"

#include <cassert>
#include <cmath>
#include <cfloat>

using LiteMath::perspectiveMatrix;
using LiteMath::lookAt;
using LiteMath::inverse4x4;

CamPinHole::CamPinHole(){}
CamPinHole::~CamPinHole() {}

void CamPinHole::SetParameters(int a_width, int a_height, const CamParameters& a_params)
{
  m_width  = a_width;
  m_height = a_height;
  m_spectral_mode = a_params.spectralMode;
      
  m_proj    = perspectiveMatrix(a_params.fov, a_params.aspect, a_params.nearPlane, a_params.farPlane);
  m_projInv = inverse4x4(m_proj);
}

void CamPinHole::Init(int a_maxThreads)
{
  m_storedWaves.resize(a_maxThreads);
  m_randomGens.resize(a_maxThreads);
  #pragma omp parallel for default(shared)
  for(int i=0;i<a_maxThreads;i++)
    m_randomGens[i] = RandomGenInit(i + 12345*i);

  // init spectral curves
  m_cie_x      = Get_CIE_X();
  m_cie_y      = Get_CIE_Y();
  m_cie_z      = Get_CIE_Z();
}

void CamPinHole::MakeRaysBlock(RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, uint32_t in_blockSize, int subPassId)
{
  kernel1D_MakeEyeRay(int(in_blockSize), out_rayPosAndNear4f, out_rayDirAndFar4f, subPassId);
}

void CamPinHole::AddSamplesContributionBlock(float* out_color4f, const float* colors4f, uint32_t in_blockSize, 
                                             uint32_t a_width, uint32_t a_height, int subPassId)
{
  kernel1D_ContribSample(int(in_blockSize), colors4f, out_color4f, subPassId); 
}

void CamPinHole::kernel1D_MakeEyeRay(int in_blockSize, RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, int subPassId)
{
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int tid = 0; tid < in_blockSize; tid++)
  {
    const int x = (tid + subPassId*in_blockSize) % m_width;  // pitch-linear layout
    const int y = (tid + subPassId*in_blockSize) / m_width; // subPas is just a uniform slitting of image along the lines
    
    if(x == 256 && y == 256) // to debug target pixel
    {
      int a = 2;
    }

    float3 rayDir = EyeRayDirNormalized(float(x+0.5f)/float(m_width), float(y+0.5f)/float(m_height), m_projInv);
    float3 rayPos = float3(0,0,0);
  
    float4 wavelengths = float4(0,0,0,0);
    if(m_spectral_mode != 0)
    {
      auto genLocal     = m_randomGens[tid];
      float u           = rndFloat1_Pseudo(&genLocal);
      wavelengths       = SampleWavelengths(u, CAM_LAMBDA_MIN, CAM_LAMBDA_MAX);
      m_randomGens[tid] = genLocal;
    }

    RayPosAndW p1;
    RayDirAndT p2;

    p1.origin[0] = rayPos[0];
    p1.origin[1] = rayPos[1];
    p1.origin[2] = rayPos[2];
    p1.wave      = wavelengths.x;

    p2.direction[0] = rayDir[0];
    p2.direction[1] = rayDir[1];
    p2.direction[2] = rayDir[2];
    p2.time         = 0.0f;
  
    out_rayPosAndNear4f[tid] = p1;
    out_rayDirAndFar4f [tid] = p2;
    m_storedWaves      [tid] = wavelengths.x; // just remember waves in our buffer for camera
  }
}

void CamPinHole::kernel1D_ContribSample(int in_blockSize, const float* in_color, float* out_color, int subPassId)
{
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int tid = 0; tid < in_blockSize; tid++)
  {
    const int x = (tid + subPassId*in_blockSize) % m_width;  // pitch-linear layout
    const int y = (tid + subPassId*in_blockSize) / m_width; // subPas is just a uniform slitting of image along the lines

    //float4 color = float4(in_color[4*tid+0], in_color[4*tid+1], in_color[4*tid+2], in_color[4*tid+3]); // always float4
    float4 color;   
    if(m_spectral_mode != 0)
    {
      float data = in_color[tid];
      color = float4(data,data,data,data);
    }
    else
      color = float4(in_color[4*tid+0], in_color[4*tid+1], in_color[4*tid+2], in_color[4*tid+3]);

    if(m_spectral_mode != 0) // TODO: spectral framebuffer
    {
      const float4 wavelengths = float4(m_storedWaves[tid]);
                                  
      const float3 xyz = SpectrumToXYZ(color, wavelengths, CAM_LAMBDA_MIN, CAM_LAMBDA_MAX, m_cie_x.data(), m_cie_y.data(), m_cie_z.data(), false);
      color = to_float4(XYZToRGB(xyz), 1.0f);
    }

    out_color[(y*m_width+x)*4+0] += color[0];  // R
    out_color[(y*m_width+x)*4+1] += color[1];  // G
    out_color[(y*m_width+x)*4+2] += color[2];  // B
    //out_color[(y*m_width+x)*4+3] += color[3];// A
  }
}
