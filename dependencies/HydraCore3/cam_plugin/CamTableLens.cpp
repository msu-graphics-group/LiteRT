#include "CamTableLens.h"

#include <algorithm> // reverse
#include <cassert>
#include <cmath>
#include <cfloat>

using LiteMath::perspectiveMatrix;
using LiteMath::lookAt;
using LiteMath::inverse4x4;
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline bool Quadratic(float A, float B, float C, float *t0, float *t1) {
  // Find quadratic discriminant
  double discrim = (double)B * (double)B - 4. * (double)A * (double)C;
  if (discrim < 0.) 
    return false;
  double rootDiscrim = std::sqrt(discrim);
  float floatRootDiscrim   = float(rootDiscrim);
  // Compute quadratic _t_ values
  float q;
  if ((float)B < 0)
      q = -.5 * (B - floatRootDiscrim);
  else
      q = -.5 * (B + floatRootDiscrim);
  *t0 = q / A;
  *t1 = C / q;
  if ((float)*t0 > (float)*t1) 
  {
    // std::swap(*t0, *t1);
    float temp = *t0;
    *t0 = *t1;
    *t1 = temp;
  }
  return true;
}

static inline bool Refract(const float3 wi, const float3 n, float eta, float3 *wt) {
  // Compute $\cos \theta_\roman{t}$ using Snell's law
  float cosThetaI  = dot(n, wi);
  float sin2ThetaI = std::max(float(0), float(1.0f - cosThetaI * cosThetaI));
  float sin2ThetaT = eta * eta * sin2ThetaI;
  // Handle total internal reflection for transmission
  if (sin2ThetaT >= 1) return false;
  float cosThetaT = std::sqrt(1 - sin2ThetaT);
  *wt = eta * (-1.0f)*wi + (eta * cosThetaI - cosThetaT) * n;
  return true;
}

static inline float3 faceforward(const float3 n, const float3 v) { return (dot(n, v) < 0.f) ? (-1.0f)*n : n; }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CamTableLens::CamTableLens(){}
CamTableLens::~CamTableLens() {}

void CamTableLens::SetParameters(int a_width, int a_height, const CamParameters& a_params)
{
  m_width  = a_width;
  m_height = a_height;
  m_spectral_mode = a_params.spectralMode;
      
  m_proj    = perspectiveMatrix(a_params.fov, a_params.aspect, a_params.nearPlane, a_params.farPlane);
  m_projInv = inverse4x4(m_proj);
}

void CamTableLens::Init(int a_maxThreads)
{
  m_storedWaves.resize(a_maxThreads);
  m_randomGens.resize(a_maxThreads);
  m_storedCos4.resize(a_maxThreads);
  #pragma omp parallel for default(shared)
  for(int i=0;i<a_maxThreads;i++)
    m_randomGens[i] = RandomGenInit(i + 12345*i);

  // init spectral curves
  m_cie_x      = Get_CIE_X();
  m_cie_y      = Get_CIE_Y();
  m_cie_z      = Get_CIE_Z();
  
  // init optic parameters
  //
  lines = {
    { .curvatureRadius=0.0302249007f,  .thickness=0.00083350006f, .eta=1.62,        .apertureRadius=0.0151700005},   // 0
    { .curvatureRadius=0.0113931f,     .thickness=0.00741360011f, .eta=1.0,         .apertureRadius=0.0103400005},   // 1
    { .curvatureRadius=0.0752018988f,  .thickness=0.00106540008f, .eta=1.63900006f, .apertureRadius=0.00889999978f}, // 2
    { .curvatureRadius=0.00833490025,  .thickness=0.0111549003,   .eta=1.0,         .apertureRadius=0.00671000034},
    { .curvatureRadius=0.00958819967,  .thickness=0.00200540014,  .eta=1.65400004,  .apertureRadius=0.00451000035},
    { .curvatureRadius=0.0438676998,   .thickness=0.00538950041,  .eta=1.0,         .apertureRadius=0.00407000026},
    { .curvatureRadius=0.0,            .thickness=0.00141630007,  .eta=0.0,         .apertureRadius=0.00275000022},
    { .curvatureRadius=0.0294541009,   .thickness=0.00219339994,  .eta=1.51699996,  .apertureRadius=0.00298000011},
    { .curvatureRadius=-0.00522650033, .thickness=0.000971400063, .eta=1.80499995,  .apertureRadius=0.00292000012},
    { .curvatureRadius=-0.0142884003,  .thickness=6.27000045e-05, .eta=1.0,         .apertureRadius=0.00298000011},
    { .curvatureRadius=-0.0223726016,  .thickness=0.000940000056, .eta=1.67299998,  .apertureRadius=0.00298000011},
    { .curvatureRadius=-0.0150404004,  .thickness=0.0233591795,   .eta=1.0,         .apertureRadius=0.00326000014},
  };

  std::reverse(lines.begin(), lines.end()); // if you need this ...  

  m_diagonal = 0.035f;
  m_aspect   = 1.0f;

  // CalcPhysSize();
  //
  m_physSize.x = 2.0f*std::sqrt(m_diagonal * m_diagonal / (1.0f + m_aspect * m_aspect));
  m_physSize.y = m_aspect * m_physSize.x;

}

void CamTableLens::MakeRaysBlock(RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, uint32_t in_blockSize, int subPassId)
{
  kernel1D_MakeEyeRay(int(in_blockSize), out_rayPosAndNear4f, out_rayDirAndFar4f, subPassId);
}

void CamTableLens::AddSamplesContributionBlock(float* out_color4f, const float* colors4f, uint32_t in_blockSize, 
                                             uint32_t a_width, uint32_t a_height, int subPassId)
{
  kernel1D_ContribSample(int(in_blockSize), colors4f, out_color4f, subPassId); 
}


bool CamTableLens::IntersectSphericalElement(float radius, float zCenter, float3 rayPos, float3 rayDir, 
                                             float *t, float3 *n) const
{
  // Compute _t0_ and _t1_ for ray--element intersection
  const float3 o = rayPos - float3(0, 0, zCenter);
  const float  A = rayDir.x * rayDir.x + rayDir.y * rayDir.y + rayDir.z * rayDir.z;
  const float  B = 2 * (rayDir.x * o.x + rayDir.y * o.y + rayDir.z * o.z);
  const float  C = o.x * o.x + o.y * o.y + o.z * o.z - radius * radius;
  float  t0, t1;
  if (!Quadratic(A, B, C, &t0, &t1)) 
    return false;
  
  // Select intersection $t$ based on ray direction and element curvature
  bool useCloserT = (rayDir.z > 0.0f) != (radius < 0.0);
  *t = useCloserT ? std::min(t0, t1) : std::max(t0, t1);
  if (*t < 0.0f) 
    return false;
  
  // Compute surface normal of element at ray intersection point
  *n = normalize(o + (*t)*rayDir);
  *n = faceforward(*n, -1.0f*rayDir);
  return true;
}

bool CamTableLens::TraceLensesFromFilm(const float3 inRayPos, const float3 inRayDir, 
                                       float3* outRayPos, float3* outRayDir) const
{
  float elementZ = 0;
  // Transform _rCamera_ from camera to lens system space
  // 
  float3 rayPosLens = float3(inRayPos.x, inRayPos.y, -inRayPos.z);
  float3 rayDirLens = float3(inRayDir.x, inRayDir.y, -inRayDir.z);

  for(int i=0; i<lines.size(); i++)
  {
    const auto element = lines[i];                                  
    // Update ray from film accounting for interaction with _element_
    elementZ -= element.thickness;
    
    // Compute intersection of ray with lens element
    float t;
    float3 n;
    bool isStop = (element.curvatureRadius == 0.0f);
    if (isStop) 
    {
      // The refracted ray computed in the previous lens element
      // interface may be pointed towards film plane(+z) in some
      // extreme situations; in such cases, 't' becomes negative.
      if (rayDirLens.z >= 0.0f) 
        return false;
      t = (elementZ - rayPosLens.z) / rayDirLens.z;
    } 
    else 
    {
      const float radius  = element.curvatureRadius;
      const float zCenter = elementZ + element.curvatureRadius;
      if (!IntersectSphericalElement(radius, zCenter, rayPosLens, rayDirLens, &t, &n))
        return false;
    }

    // Test intersection point against element aperture
    const float3 pHit = rayPosLens + t*rayDirLens;
    const float r2    = pHit.x * pHit.x + pHit.y * pHit.y;
    if (r2 > element.apertureRadius * element.apertureRadius) 
      return false;
    
    rayPosLens = pHit;
    // Update ray path for from-scene element interface interaction
    if (!isStop) 
    {
      float3 wt;
      float etaI = lines[i+0].eta;                                                      
      float etaT = (i == lines.size()-1) ? 1.0f : lines[i+1].eta;
      if(etaT == 0.0f)
        etaT = 1.0f;                                                          
      if (!Refract(normalize((-1.0f)*rayDirLens), n, etaI / etaT, &wt))
        return false;
      rayDirLens = wt;
    }

  }

  // Transform _rLens_ from lens system space back to camera space
  //
  (*outRayPos) = float3(rayPosLens.x, rayPosLens.y, -rayPosLens.z);
  (*outRayDir) = float3(rayDirLens.x, rayDirLens.y, -rayDirLens.z);
  return true;  
}



void CamTableLens::kernel1D_MakeEyeRay(int in_blockSize, RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, int subPassId)
{
  #pragma omp parallel for default(shared)
  for(int tid = 0; tid < in_blockSize; tid++)
  {
    const int x = (tid + subPassId*in_blockSize) % m_width;  // pitch-linear layout
    const int y = (tid + subPassId*in_blockSize) / m_width; // subPas is just a uniform slitting of image along the lines
    
    auto genLocal = m_randomGens[tid];
    const float4 rands = rndFloat4_Pseudo(&genLocal);
    
    float3 ray_pos, ray_dir;
    float cosTheta;
    float4 wavelengths = float4(0,0,0,0);
    bool success = false;
    int failed = 0;
    const int MAX_TRIALS = 10;
    
    if(m_spectral_mode != 0)
      wavelengths = SampleWavelengths(rands.z, CAM_LAMBDA_MIN, CAM_LAMBDA_MAX);

    const float2 xy = 0.25f*m_physSize*float2(2.0f*(float(x+0.5f)/float(m_width))  - 1.0f, 
                                              2.0f*(float(y+0.5f)/float(m_height)) - 1.0f);
    
    ray_pos = float3(xy.x, xy.y, 0);

    const float2 rareSam  = LensRearRadius()*2.0f*MapSamplesToDisc(float2(rands.x - 0.5f, rands.y - 0.5f));
    const float3 shootTo  = float3(rareSam.x, rareSam.y, LensRearZ());
    const float3 ray_dirF = normalize(shootTo - ray_pos);

    cosTheta  = std::abs(ray_dirF.z);
    ray_dir   = ray_dirF;
    success   = TraceLensesFromFilm(ray_pos, ray_dir, &ray_pos, &ray_dir);

    if (!success) 
    {
      ray_pos = float3(0,-10000000.0,0.0); // shoot ray under the floor
      ray_dir = float3(0,-1,0);
      failed++;
    }
    else
    {
      ray_dir = float3(-1,-1,-1)*normalize(ray_dir);
      ray_pos = float3(-1,-1,-1)*ray_pos;
    }

    m_randomGens[tid] = genLocal;

    RayPosAndW p1;
    RayDirAndT p2;

    p1.origin[0] = ray_pos[0];
    p1.origin[1] = ray_pos[1];
    p1.origin[2] = ray_pos[2];
    p1.wave      = wavelengths.x;

    p2.direction[0] = ray_dir[0];
    p2.direction[1] = ray_dir[1];
    p2.direction[2] = ray_dir[2];
    p2.time         = 0.0f;
  
    out_rayPosAndNear4f[tid] = p1;
    out_rayDirAndFar4f [tid] = p2;
    m_storedWaves      [tid] = wavelengths.x; // just remember waves in our buffer for camera
    m_storedCos4       [tid] = cosTheta*cosTheta*cosTheta*cosTheta;
  }
}

void CamTableLens::kernel1D_ContribSample(int in_blockSize, const float* in_color, float* out_color, int subPassId)
{
  for(int tid = 0; tid < in_blockSize; tid++)
  {
    const int x = (tid + subPassId*in_blockSize) % m_width;  // pitch-linear layout
    const int y = (tid + subPassId*in_blockSize) / m_width; // subPas is just a uniform slitting of image along the lines

    //float4 color = in_color[tid]*m_storedCos4[tid];
    float4 color;   
    if(m_spectral_mode != 0)
    {
      float data = in_color[tid]*m_storedCos4[tid];
      color = float4(data,data,data,data);
    }
    else
      color = float4(in_color[4*tid+0], in_color[4*tid+1], in_color[4*tid+2], in_color[4*tid+3])*m_storedCos4[tid]; 

    if(m_spectral_mode != 0) // TODO: spectral framebuffer
    {
      float4 wavelengths = float4(m_storedWaves[tid]);                            
      const float3 xyz = SpectrumToXYZ(color, wavelengths, CAM_LAMBDA_MIN, CAM_LAMBDA_MAX, m_cie_x.data(), m_cie_y.data(), m_cie_z.data(), false);
      color = to_float4(XYZToRGB(xyz), 1.0f);
    }

    //out_color[y*m_width+x] += color;
    out_color[(y*m_width+x)*4+0] += color[0];  // R
    out_color[(y*m_width+x)*4+1] += color[1];  // G
    out_color[(y*m_width+x)*4+2] += color[2];  // B
    //out_color[(y*m_width+x)*4+3] += color[3];// A
  }
}
