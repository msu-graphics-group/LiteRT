#include "integrator_qmc.h"

#include <omp.h>
#include <iomanip>
#include "utils.h" // for progress bar

class IntegratorKMLT : public IntegratorQMC
{
public:

  IntegratorKMLT(int a_maxThreads, std::vector<uint32_t> a_features) : IntegratorQMC(a_maxThreads, a_features)
  {

  }

  float  GetRandomNumbersSpec(uint tid, RandomGen* a_gen) override;
  float  GetRandomNumbersTime(uint tid, RandomGen* a_gen) override;
  float4 GetRandomNumbersLens(uint tid, RandomGen* a_gen) override;
  float4 GetRandomNumbersMats(uint tid, RandomGen* a_gen, int a_bounce) override;
  float4 GetRandomNumbersLgts(uint tid, RandomGen* a_gen, int a_bounce) override;
  float  GetRandomNumbersMatB(uint tid, RandomGen* a_gen, int a_bounce, int a_layer) override;

  void   kernel_InitEyeRay(uint tid, const uint* packedXY, 
                           float4* rayPosAndNear, float4* rayDirAndFar, float4* wavelengths, 
                           float4* accumColor,    float4* accumuThoroughput,
                           RandomGen* gen, uint* rayFlags, MisData* misData, float* time, int* pX, int* pY);


  void   PathTraceBlock(uint pixelsNum, uint channels, float* out_color, uint a_passNum) override;
  
  float4 PathTraceF(uint tid, int*pX, int* pY);

  static constexpr uint BOUNCE_START = 6;
  static constexpr uint LGHT_ID      = 0;
  static constexpr uint MATS_ID      = 4;
  static constexpr uint BLND_ID      = 8;
  static constexpr uint PER_BOUNCE   = 10;

  
  inline uint RandsPerThread() const { return PER_BOUNCE*m_traceDepth + BOUNCE_START; }

  std::vector<float>    m_allRands;
  uint                  m_randsPerThread = 0;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr float MUTATE_COEFF_SCREEN = 128.0f;
constexpr float MUTATE_COEFF_BSDF   = 64.0f;

/**
\brief mutate random number in primary sample space -- interval [0,1]
\param valueX    - input original value
\param rands     - input pseudo random numbers
\param p2        - parameter of step size. The greater parameter is, the smaller step we gain. Default = 64.0f;
\param p1        - parameter of step size. The greater parameter is, the smaller step we gain. Default = 1024.0f;
\return mutated random float in range [0,1]

*/
static inline float MutateKelemen(float valueX, float2 rands, float p2, float p1)   // mutate in primary space
{
  const float s1    = 1.0f / p1;
  const float s2    = 1.0f / p2;
  const float power = -std::log(s2 / s1);
  const float dv    = std::max(s2*( std::exp(power*std::sqrt(rands.x)) - std::exp(power) ), 0.0f);

  if (rands.y < 0.5f)
  {
    valueX += dv;
    if (valueX > 1.0f)
      valueX -= 1.0f;
  }
  else
  {
    valueX -= dv;
    if (valueX < 0.0f)
      valueX += 1.0f;
  }

  return valueX;
}

float4 IntegratorKMLT::GetRandomNumbersLens(uint tid, RandomGen* a_gen) 
{ 
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersLens(tid, a_gen);
  else
  {
    float* data  = m_allRands.data() + tid*m_randsPerThread;
    return float4(data[0], data[1], data[2], data[3]);
  }
}

float  IntegratorKMLT::GetRandomNumbersSpec(uint tid, RandomGen* a_gen) 
{ 
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersSpec(tid, a_gen);
  else
  {
    float* data = m_allRands.data() + tid*m_randsPerThread;
    return data[4]; 
  }
}

float  IntegratorKMLT::GetRandomNumbersTime(uint tid, RandomGen* a_gen)
{
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersTime(tid, a_gen);
  else
  {
    float* data = m_allRands.data() + tid*m_randsPerThread;
    return data[5]; 
  }
}

float4 IntegratorKMLT::GetRandomNumbersMats(uint tid, RandomGen* a_gen, int a_bounce) 
{ 
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersMats(tid, a_gen, a_bounce);
  else
  {
    float* data  = m_allRands.data() + tid*m_randsPerThread + BOUNCE_START + a_bounce*PER_BOUNCE + MATS_ID;
    return float4(data[0], data[1], data[2], data[3]);
  }
}

float4 IntegratorKMLT::GetRandomNumbersLgts(uint tid, RandomGen* a_gen, int a_bounce)
{
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersLgts(tid, a_gen, a_bounce);
  else
  {
    float* data  = m_allRands.data() + tid*m_randsPerThread + BOUNCE_START + a_bounce*PER_BOUNCE + LGHT_ID;
    return float4(data[0], data[1], data[2], data[3]);
  }
}

float IntegratorKMLT::GetRandomNumbersMatB(uint tid, RandomGen* a_gen, int a_bounce, int a_layer) 
{ 
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::GetRandomNumbersMatB(tid, a_gen, a_bounce, a_layer);
  else
  {
    float* data   = m_allRands.data() + tid*m_randsPerThread + BOUNCE_START + a_bounce*PER_BOUNCE + BLND_ID;
    return data[a_layer];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IntegratorKMLT::kernel_InitEyeRay(uint tid, const uint* packedXY, 
                                       float4* rayPosAndNear, float4* rayDirAndFar, float4* wavelengths, 
                                       float4* accumColor,    float4* accumuThoroughput,
                                       RandomGen* gen, uint* rayFlags, MisData* misData, float* time, int* pX, int* pY) // 
{
  if(tid >= m_maxThreadId)
    return;

  *accumColor        = make_float4(0,0,0,0);
  *accumuThoroughput = make_float4(1,1,1,1);
  *rayFlags          = 0;
  *misData           = makeInitialMisData();
  
  RandomGen genLocal = m_randomGens[RandomGenId(tid)];

  EyeRayData r = SampleCameraRay(&genLocal, tid);
  
  if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
    *wavelengths = SampleWavelengths(r.waveSam, LAMBDA_MIN, LAMBDA_MAX);
  else
    *wavelengths = float4(0.0f);

  *time = r.timeSam;
 
  transform_ray3f(m_worldViewInv, &r.rayPos, &r.rayDir);

  *rayPosAndNear = to_float4(r.rayPos, 0.0f);
  *rayDirAndFar  = to_float4(r.rayDir, FLT_MAX);
  *gen           = genLocal;

  (*pX) = int(r.x);
  (*pY) = int(r.y);
}

float4 IntegratorKMLT::PathTraceF(uint tid, int*pX, int* pY)
{
  float4 accumColor, accumThroughput;
  float4 rayPosAndNear, rayDirAndFar;
  float4 wavelengths;
  RandomGen gen; 
  MisData   mis;
  uint      rayFlags;
  float     time;
  kernel_InitEyeRay(tid, m_packedXY.data(), &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &rayFlags, &mis, &time, pX, pY);

  for(uint depth = 0; depth < m_traceDepth; depth++) 
  {
    float4   shadeColor, hitPart1, hitPart2, hitPart3;
    uint instId;
    kernel_RayTrace2(tid, depth, &rayPosAndNear, &rayDirAndFar, &time, 
                     &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(isDeadRay(rayFlags))
      break;
    
    kernel_SampleLightSource(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &hitPart1, &hitPart2, &hitPart3, &rayFlags, &time,
                             depth, &gen, &shadeColor);

    kernel_NextBounce(tid, depth, &hitPart1, &hitPart2, &hitPart3, &instId, &shadeColor,
                      &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &mis, &rayFlags);

    if(isDeadRay(rayFlags))
      break;
  }

  kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput, &accumColor);
  
  // change
  //
  if(KSPEC_SPECTRAL_RENDERING!=0 && m_spectral_mode != 0) 
    accumColor = to_float4(SpectralCamRespoceToRGB(accumColor, wavelengths, rayFlags), 0.0f);

  return accumColor*m_exposureMult;
}

static inline float contribFunc(float4 color) // WORKS ONLY FOR RGB, sectral to RGB must be done earlier
{
  return std::max(0.333334f*(color.x + color.y + color.z), 0.0f);
}

uint32_t AlignedSize(uint32_t a_size, uint32_t a_alignment)
{
  if (a_size % a_alignment == 0)
    return a_size;
  else
  {
    uint32_t sizeCut = a_size - (a_size % a_alignment);
    return sizeCut + a_alignment;
  }
}

bool SaveImage4fToBMP(const float* rgb, int width, int height, const char* outfilename, float a_normConst, float a_gamma);

void IntegratorKMLT::PathTraceBlock(uint pixelsNum, uint channels, float* out_color, uint a_passNum)
{
  if(m_renderLayer == FB_DIRECT)
    return IntegratorQMC::PathTraceBlock(pixelsNum, channels, out_color, a_passNum);
  
  uint maxThreads = 1;
  #ifndef _DEBUG
  #pragma omp parallel
  {
    #pragma omp single
    maxThreads = omp_get_max_threads();
  }
  #endif

  m_randsPerThread = AlignedSize(RandsPerThread(), uint32_t(16)); 

  m_maxThreadId = maxThreads;
  m_randomGens.resize(maxThreads);
  m_allRands.resize(maxThreads*m_randsPerThread);

  const size_t samplesPerPass = (size_t(pixelsNum)*size_t(a_passNum)) / size_t(maxThreads);

  std::cout << "[IntegratorKMLT]: state size = " << m_randsPerThread << std::endl;
  std::cout.flush();

  /////
  //std::vector<float> m_omcImage(pixelsNum*4);
  //std::fill(m_omcImage.begin(), m_omcImage.end(), 0.0f);

  ConsoleProgressBar progress(pixelsNum*a_passNum);
  progress.Start();
  auto start = std::chrono::high_resolution_clock::now();
  
  double avgBrightnessOut  = 0.0f;
  float  avgAcceptanceRate = 0.0f;
  const float plarge       = 0.25f;                         // 25% of large step;

  #ifndef _DEBUG
  #pragma omp parallel default(shared)
  #endif
  {
    int tid = omp_get_thread_num();
    
    RandomGen gen1 = RandomGenInit(tid*7 + 1);
    RandomGen gen2 = RandomGenInit(tid);
    for (int i = 0; i < 10 + tid % 17; i++) 
    {
      NextState(&gen1);
      NextState(&gen2);
    }

    // (1) Initial State
    //
    int xScr = 0, yScr = 0;
    std::vector<float> xVec(m_randsPerThread);
    float* xNew = m_allRands.data() + m_randsPerThread*tid; 
    

    for(size_t i=0;i<xVec.size();i++)
      xVec[i] = rndFloat1_Pseudo(&gen2);
    for(size_t i=0;i<xVec.size();i++) // eval F(xVec)
      xNew[i] = xVec[i];

    float4 yColor = PathTraceF(tid, &xScr, &yScr);
    float  y      = contribFunc(yColor);

    // (2) Markov Chain
    //
    size_t accept     = 0;
    size_t largeSteps = 0;
    double accumBrightness = 0.0;

    for(size_t i=0;i<samplesPerPass;i++) 
    {
      const bool isLargeStep = (rndFloat1_Pseudo(&gen1) < plarge);  
  
      if (isLargeStep)                                      // large step
      {
        for(size_t i=0;i<xVec.size();i+=4) {
          const float4 r1 = rndFloat4_Pseudo(&gen2);
          xNew[i+0] = r1.x;
          xNew[i+1] = r1.y;
          xNew[i+2] = r1.z;
          xNew[i+3] = r1.w;
        }
      }
      else
      {
        const float4 r1 = rndFloat4_Pseudo(&gen2);
        const float4 r2 = rndFloat4_Pseudo(&gen2);

        xNew[0] = MutateKelemen(xVec[0], float2(r1.x, r1.y), MUTATE_COEFF_SCREEN*1.0f, 1024.0f); // screen 
        xNew[1] = MutateKelemen(xVec[1], float2(r1.z, r1.w), MUTATE_COEFF_SCREEN*1.0f, 1024.0f); // screen
        xNew[2] = MutateKelemen(xVec[2], float2(r2.x, r2.y), MUTATE_COEFF_BSDF, 1024.0f);        // lens
        xNew[3] = MutateKelemen(xVec[3], float2(r2.z, r2.w), MUTATE_COEFF_BSDF, 1024.0f);        // lens
       
        for(size_t i = 4; i < xVec.size(); i+=2) { 
          const float4 r1 = rndFloat4_Pseudo(&gen2);
          xNew[i+0] = MutateKelemen(xVec[i+0], float2(r1.x, r1.y), MUTATE_COEFF_BSDF, 1024.0f);
          xNew[i+1] = MutateKelemen(xVec[i+1], float2(r1.z, r1.w), MUTATE_COEFF_BSDF, 1024.0f);
        }
      }

      float  yOld      = y;
      float4 yOldColor = yColor;
  
      int xScrOld = xScr, yScrOld = yScr;
      int xScrNew = 0,    yScrNew = 0;
  
      float4 yNewColor = PathTraceF(tid, &xScrNew, &yScrNew); // eval F(xNew)
      float  yNew      = contribFunc(yNewColor);
      
      float a = (yOld == 0.0f) ? 1.0f : std::min(1.0f, yNew / yOld);
      float p = rndFloat1_Pseudo(&gen1);

      if (p <= a) // accept 
      { 
        for(size_t i=0;i<xVec.size();i++)
          xVec[i] = xNew[i];
        y      = yNew;
        yColor = yNewColor;
        xScr   = xScrNew;
        yScr   = yScrNew;
        accept++;
      }
      else        // reject
      {
        //x      = x;
        //y      = y;
        //yColor = yColor;
      }

      if(isLargeStep)
      {
        accumBrightness += double(yNew);
        largeSteps++;
      }

      // (5) contrib to image
      //
      float w1 = 1.0f;
      //float w2 = 1.0f;
      //if(largeSteps >= 256*256) // enable Kelemen MIS
      //{
      //  float avgB = float(accumBrightness / double(largeSteps));
      //  if(isLargeStep)
      //  {
      //    const float INewDivB  = contribFunc(yNewColor)/avgB;
      //    w1 = (1.0f/plarge)*INewDivB/(INewDivB + plarge);
      //    w2 = ((1.0f/plarge)/(1.0f-plarge))*plarge/(INewDivB + plarge);
      //    { 
      //      const int offset = yScrNew*m_winWidth + xScrNew;
      //      #pragma omp atomic
      //      out_color[offset*4+0] += yNewColor.x*w2;
      //      #pragma omp atomic
      //      out_color[offset*4+1] += yNewColor.y*w2;
      //      #pragma omp atomic
      //      out_color[offset*4+2] += yNewColor.z*w2;
      //    }
      //  }
      //}
      
      float3 contribAtY = w1*to_float3(yNewColor)*(1.0f / std::max(yNew, 1e-6f))*a;
      float3 contribAtX = w1*to_float3(yOldColor)*(1.0f / std::max(yOld, 1e-6f))*(1.0f - a);

      if (dot(contribAtX, contribAtX) > 1e-12f)
      { 
        const int offset = yScrOld*m_winWidth + xScrOld;
        #pragma omp atomic
        out_color[offset*4+0] += contribAtX.x;
        #pragma omp atomic
        out_color[offset*4+1] += contribAtX.y;
        #pragma omp atomic
        out_color[offset*4+2] += contribAtX.z;
      }

      if (dot(contribAtY, contribAtY) > 1e-12f)
      { 
        const int offset = yScrNew*m_winWidth + xScrNew;
        #pragma omp atomic
        out_color[offset*4+0] += contribAtY.x;
        #pragma omp atomic
        out_color[offset*4+1] += contribAtY.y;
        #pragma omp atomic
        out_color[offset*4+2] += contribAtY.z;
      }
      
      progress.Update();
    }

    #pragma omp atomic
    avgAcceptanceRate += float(accept);

    double avgBrightness = accumBrightness / double(largeSteps);
    #pragma omp atomic
    avgBrightnessOut += avgBrightness;
  }

  progress.Done();

  avgBrightnessOut  = avgBrightnessOut  / double(m_maxThreadId);
  avgAcceptanceRate = avgAcceptanceRate / float(pixelsNum*a_passNum);

  std::cout << "[IntegratorKMLT]: average brightness      = " << std::fixed << std::setprecision(2) << avgBrightnessOut << std::endl;
  std::cout << "[IntegratorKMLT]: average acceptance rate = " << std::fixed << std::setprecision(2) << 100.0f*avgAcceptanceRate << "%" << std::endl;
  std::cout.flush();

  //{
  //  const float normConstOMC = 1.0f/float(a_passNum);
  //  SaveImage4fToBMP(m_omcImage.data(), m_winWidth, m_winHeight, "z_out_omc.bmp", normConstOMC, 2.4f);
  //}

  double actualBrightness = 0.0;
  {
    for(uint i=0;i<pixelsNum;i++)
    {
      float4 color = float4(out_color[i*4+0], out_color[i*4+1], out_color[i*4+2], out_color[i*4+3]);
      actualBrightness += contribFunc(color);
    }
    actualBrightness /= double(pixelsNum);
  }
  
  const float normConst = float(a_passNum)*float(avgBrightnessOut/actualBrightness);
  for(uint i=0;i<pixelsNum*channels;i++)
    out_color[i] = out_color[i]*normConst; // + m_omcImage[i];
  
  //for(uint i=0;i<pixelsNum*channels;i++)
  //  out_color[i] = m_omcImage[i]*(1.0f/plarge);

  shadowPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Integrator> CreateIntegratorKMLT(int a_maxThreads = 1, std::vector<uint32_t> a_features = {}) 
{ 
  return std::make_shared<IntegratorKMLT>(a_maxThreads, a_features); 
}