#include "integrator_qmc.h"
#include "utils.h" // for progress bar

#include <cassert>

IntegratorQMC:: IntegratorQMC(int a_maxThreads, std::vector<uint32_t> a_features) : Integrator(a_maxThreads, a_features)
{
  qmc::init(m_qmcTable);
}

void IntegratorQMC::EnableQMC()
{
  const bool dof    = (m_camLensRadius > 0.0f) || (m_enableOpticSim != 0);
  const bool spd    = (m_spectral_mode != 0);
  const bool motion = (m_normMatrices2.size() != 0);

  assert(qmc::QRNG_DIMENSIONS >= 11); // code of this function assume >= 11 dimentions
  
  // QMC for: (optics, spd, time)
  //
  //m_qmcDofDim    = 2;
  //m_qmcSpdDim    = 4;
  //m_qmcMotionDim = 5;
  //m_qmcMatDim    = 0; 
  //m_qmcLgtDim    = 0; 
  //return;

  if(dof && spd && motion) // (0,1): pixel id; (2,3): lens; (4,5): spd and motion; (6,7): mat; (8,9,10): light.
  {
    m_qmcMotionDim = 5;
    m_qmcSpdDim    = 4;
    m_qmcMatDim    = 0; // 6; // may set to zero to enable OMC here
    m_qmcLgtDim    = 0; // 8; // may set to zero to enable OMC here
  }
  else if(dof && spd)    // (0,1): pixel id; (2,3): lens; (4): spd; (5,6): mat; (7,8,9): light.
  {
    m_qmcMotionDim = 0;
    m_qmcSpdDim    = 4;
    m_qmcMatDim    = 5;
    m_qmcLgtDim    = 7;
  }
  else if(spd && motion) // (0,1): pixel id; (2,3): motion and spd; (4,5): mat; (6,7,8): light.
  {
    m_qmcMotionDim = 2;
    m_qmcSpdDim    = 3;
    m_qmcMatDim    = 4;
    m_qmcLgtDim    = 6;
  }
  else if(dof && motion) // (0,1): pixel id; (2,3): lens; (4): motion; (5,6): mat; (7,8,9): light.
  {
    m_qmcMotionDim = 4;
    m_qmcSpdDim    = 0;
    m_qmcMatDim    = 5; // may set to zero to enable OMC here
    m_qmcLgtDim    = 7; // may set to zero to enable OMC here
  }
  else if(dof)  // (0,1): pixel id; (2,3): lens; (4,5): mat; (6,7,8): light.
  {
    m_qmcMotionDim = 0;
    m_qmcSpdDim    = 0;
    m_qmcMatDim    = 4;
    m_qmcLgtDim    = 6;
  }
  else if(spd)  // (0,1): pixel id; (2,3): mat; (4) : spd; (5,6,7): light.
  {
    m_qmcMotionDim = 0;
    m_qmcSpdDim    = 4;
    m_qmcMatDim    = 2;
    m_qmcLgtDim    = 5;
  }
  else if(motion) // (0,1): pixel id; (2,3): mat; (4) : motion; (5,6,7): light.
  {
    m_qmcMotionDim = 4;
    m_qmcSpdDim    = 0;
    m_qmcMatDim    = 2;
    m_qmcLgtDim    = 5;
  }
  else // (0,1): pixel id; (2,3): mat; (4,5,6): light.
  {
    m_qmcMotionDim = 0;
    m_qmcSpdDim    = 0;
    m_qmcMatDim    = 2;
    m_qmcLgtDim    = 4;
  }

  m_qmcDofDim = 2;
}

float  IntegratorQMC::GetRandomNumbersSpec(uint tid, RandomGen* a_gen) 
{ 
  if(m_qmcSpdDim != 0)
    return qmc::rndFloat(tid, m_qmcSpdDim, m_qmcTable[0]);
  else 
    return rndFloat1_Pseudo(a_gen);
}

float  IntegratorQMC::GetRandomNumbersTime(uint tid, RandomGen* a_gen) 
{ 
  if(m_qmcMotionDim != 0)
    return qmc::rndFloat(tid, m_qmcMotionDim, m_qmcTable[0]);
  else
    return rndFloat1_Pseudo(a_gen);
}

float4 IntegratorQMC::GetRandomNumbersLens(uint tid, RandomGen* a_gen) 
{ 
  float4 rands = rndFloat4_Pseudo(a_gen);
  rands.x = qmc::rndFloat(tid, 0, m_qmcTable[0]);
  rands.y = qmc::rndFloat(tid, 1, m_qmcTable[0]);
  if((m_camLensRadius > 0.0f || (m_enableOpticSim != 0)) && m_qmcDofDim != 0)
  {
    rands.z = qmc::rndFloat(tid, 2, m_qmcTable[0]);
    rands.w = qmc::rndFloat(tid, 3, m_qmcTable[0]);
  }
  return rands; 
}

float4 IntegratorQMC::GetRandomNumbersMats(uint tid, RandomGen* a_gen, int a_bounce) 
{ 
  float4 rands = rndFloat4_Pseudo(a_gen);
  if(a_bounce == 0 && m_qmcMatDim != 0)
  {
    rands.x = qmc::rndFloat(tid, m_qmcMatDim + 0, m_qmcTable[0]);
    rands.y = qmc::rndFloat(tid, m_qmcMatDim + 1, m_qmcTable[0]);
  }
  return rands; 
}

float4 IntegratorQMC::GetRandomNumbersLgts(uint tid, RandomGen* a_gen, int a_bounce)
{
  float4 rands = rndFloat4_Pseudo(a_gen); // don't use single rndFloat4 (!!!)
  float  rndId = rndFloat1_Pseudo(a_gen); // don't use single rndFloat4 (!!!)
  if(a_bounce == 0 && m_qmcLgtDim != 0)
  {
    rands.x = qmc::rndFloat(tid, m_qmcLgtDim + 0, m_qmcTable[0]);
    rands.y = qmc::rndFloat(tid, m_qmcLgtDim + 1, m_qmcTable[0]);
    rndId   = qmc::rndFloat(tid, m_qmcLgtDim + 2, m_qmcTable[0]);
  }
  return float4(rands.x, rands.y, rands.z, rndId);
}

float IntegratorQMC::GetRandomNumbersMatB(uint tid, RandomGen* a_gen, int a_bounce, int a_layer) 
{ 
  return rndFloat1_Pseudo(a_gen); 
}

uint IntegratorQMC::RandomGenId(uint tid) { return tid % uint(m_randomGens.size()); }

Integrator::EyeRayData IntegratorQMC::SampleCameraRay(RandomGen* pGen, uint tid)
{
  const float4 pixelOffsets = GetRandomNumbersLens(tid, pGen);
  float3 rayDir = EyeRayDirNormalized(pixelOffsets.x, pixelOffsets.y, m_projInv); // TODO: add support for viewport ... 
  float3 rayPos = float3(0,0,0);

  if (m_camLensRadius > 0.0f)
  {
    const float tFocus         = m_camTargetDist / (-rayDir.z);
    const float3 focusPosition = rayPos + rayDir*tFocus;
    const float2 xy            = m_camLensRadius*2.0f*MapSamplesToDisc(float2(pixelOffsets.z - 0.5f, pixelOffsets.w - 0.5f));
    rayPos.x += xy.x;
    rayPos.y += xy.y;
    rayDir = normalize(focusPosition - rayPos);
  }
  else if(m_enableOpticSim != 0) // not nessesary part of QMC. Just implemented here for test cases, could be moved in main class further  
  {
    const float2 xy = 0.25f*m_physSize*float2(2.0f*pixelOffsets.x - 1.0f, 2.0f*pixelOffsets.y - 1.0f);
    
    rayPos = float3(xy.x, xy.y, 0);
    
    const float2 rareSam  = LensRearRadius()*2.0f*MapSamplesToDisc(float2(pixelOffsets.z - 0.5f, pixelOffsets.w - 0.5f));
    const float3 shootTo  = float3(rareSam.x, rareSam.y, LensRearZ());
    const float3 ray_dirF = normalize(shootTo - rayPos);
    
    float cosTheta = std::abs(ray_dirF.z);
    rayDir         = ray_dirF;
    bool success   = TraceLensesFromFilm(rayPos, rayDir);
    
    if (!success) 
    {
      rayPos = float3(0,-10000000.0,0.0); // shoot ray under the floor
      rayDir = float3(0,-1,0);
    }
    else
    {
      rayDir = float3(-1,-1,-1)*normalize(rayDir);
      rayPos = float3(-1,-1,-1)*rayPos;
    }
  }
  
  /////////////////////////////////////////////////////
  uint x = uint(pixelOffsets.x*float(m_winWidth));
  uint y = uint(pixelOffsets.y*float(m_winHeight));
  if(x >= uint(m_winWidth-1))
    x = uint(m_winWidth-1);
  if(y >= uint(m_winHeight-1))
    y = uint(m_winHeight-1);
  /////////////////////////////////////////////////////

  EyeRayData res;
  {
    res.rayPos = rayPos;
    res.rayDir = rayDir;
    res.x      = x;
    res.y      = y;
    res.timeSam = 0.0f;
    res.waveSam = 1.0f;
    if(m_normMatrices2.size() != 0)
      res.timeSam = GetRandomNumbersTime(tid, pGen);
    if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
      res.waveSam = GetRandomNumbersSpec(tid, pGen);
    res.cosTheta = 1.0f;
  }
  
  RecordPixelRndIfNeeded(pixelOffsets, float2(res.waveSam,res.timeSam));

  return res;
}

void IntegratorQMC::kernel_ContributeToImage(uint tid, const uint* rayFlags, uint channels, const float4* a_accumColor, const RandomGen* gen,
                                             const uint* in_pakedXY, const float4* wavelengths, float* out_color)
{
  
  if(tid >= m_maxThreadId) // don't contrubute to image in any "record" mode
    return;
  
  const float4 pixelOffsets = GetRandomNumbersLens(tid, const_cast<RandomGen*>(gen));
  m_randomGens[RandomGenId(tid)] = *gen;
  if(m_disableImageContrib !=0)
    return;
  
  /////////////////////////////////////////////////////////////////////////////// change
  uint x = uint(pixelOffsets.x*float(m_winWidth));
  uint y = uint(pixelOffsets.y*float(m_winHeight));
  if(x >= uint(m_winWidth-1))
    x = uint(m_winWidth-1);
  if(y >= uint(m_winHeight-1))
    y = uint(m_winHeight-1);
  /////////////////////////////////////////////////////////////////////////////// change   

  float4 specSamples = *a_accumColor; 
  float4 tmpVal      = specSamples*m_camRespoceRGB;
  float3 rgb         = to_float3(tmpVal);
  
  if(KSPEC_SPECTRAL_RENDERING!=0 && m_spectral_mode != 0) 
  {
    const float4 waves   = *wavelengths;
    const uint rayFlags2 = *rayFlags; 
    rgb = SpectralCamRespoceToRGB(specSamples, waves, rayFlags2);
  }

  float4 colorRes = m_exposureMult * to_float4(rgb, 1.0f);
  
  /////////////////////////////////////////////////////////////////////////////// change, add atomics
  if(channels == 1)
  {
    const float mono = 0.2126f*colorRes.x + 0.7152f*colorRes.y + 0.0722f*colorRes.z;
    #pragma omp atomic
    out_color[y*m_winWidth+x] += mono;
  }
  else if(channels <= 4)
  { 
    #pragma omp atomic
    out_color[(y*m_winWidth+x)*channels + 0] += colorRes.x;
    #pragma omp atomic
    out_color[(y*m_winWidth+x)*channels + 1] += colorRes.y;
    #pragma omp atomic
    out_color[(y*m_winWidth+x)*channels + 2] += colorRes.z;
  }
  else
  {
    auto waves = (*wavelengths);
    auto color = (*a_accumColor)*m_exposureMult;
    for(int i=0;i<4;i++) {
      const float t         = (waves[i] - LAMBDA_MIN)/(LAMBDA_MAX-LAMBDA_MIN);
      const int channelId   = std::min(int(float(channels)*t), int(channels)-1);
      const int offsetPixel = int(y)*m_winWidth + int(x);
      const int offsetLayer = channelId*m_winWidth*m_winHeight;
      #pragma omp atomic
      out_color[offsetLayer + offsetPixel] += color[i];
    }
  }
  /////////////////////////////////////////////////////////////////////////////// end change, atomics
}

void IntegratorQMC::PathTraceBlock(uint pixelsNum, uint channels, float* out_color, uint a_passNum)
{
#ifdef WIN32
  using IndexType = int;
#else
  using IndexType = unsigned; // or int
#endif // WIN32


  const size_t samplesNum = std::min<size_t>(size_t(std::numeric_limits<IndexType>::max()), size_t(pixelsNum)*size_t(a_passNum));
  m_maxThreadId = uint(samplesNum);

  EnableQMC();

  std::cout << "[IntegratorQMC]: max spp = " << samplesNum/pixelsNum << std::endl;
  std::cout.flush();

  ConsoleProgressBar progress(pixelsNum*a_passNum);
  progress.Start();
  auto start = std::chrono::high_resolution_clock::now();

  #ifndef _DEBUG
  #pragma omp parallel for default(shared) 
  #endif
  for (IndexType i = 0; i < IndexType(samplesNum); ++i) 
  {
    PathTrace(i, channels, out_color);
    progress.Update();
  }
  progress.Done();
  shadowPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Integrator> CreateIntegratorQMC(int a_maxThreads = 1, std::vector<uint32_t> a_features = {}) 
{ 
  return std::make_shared<IntegratorQMC>(a_maxThreads, a_features); 
}
