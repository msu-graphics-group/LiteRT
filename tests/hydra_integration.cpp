#include "hydra_integration.h"
#include "HydraCore3/integrator_pt.h"
#ifdef USE_GPU
  #include "HydraCore3/integrator_pt_generated.h"
  #include "vk_context.h"
#endif

std::shared_ptr<ISceneObject> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName);

void toLDRImage(const float *rgb, int width, int height, float a_normConst, float a_gamma, uint32_t *pixelData, bool a_flip)
{
  const float invGamma = 1.0f / a_gamma;

  for (int y = 0; y < height; y++) // flip image and extract pixel data
  {
    int offset1 = y * width;
    int offset2 = a_flip ? (height - y - 1) * width : offset1;
    for (int x = 0; x < width; x++)
    {
      float color[4];
      color[0] = clamp(std::pow(rgb[4 * (offset1 + x) + 0] * a_normConst, invGamma), 0.0f, 1.0f);
      color[1] = clamp(std::pow(rgb[4 * (offset1 + x) + 1] * a_normConst, invGamma), 0.0f, 1.0f);
      color[2] = clamp(std::pow(rgb[4 * (offset1 + x) + 2] * a_normConst, invGamma), 0.0f, 1.0f);
      color[3] = 1.0f;
      pixelData[offset2 + x] = RealColorToUint32(color);
    }
  }
}

HydraRenderPreset getDefaultHydraRenderPreset()
{
  HydraRenderPreset preset;
  preset.integratorType = Integrator::INTEGRATOR_MIS_PT;
  preset.fbLayer = Integrator::FB_COLOR;
  preset.spp = 1;

  return preset;
}

HydraRenderer::HydraRenderer(unsigned device)
{
  m_preset = getDefaultHydraRenderPreset();
  m_device = device;
}

void HydraRenderer::SetPreset(uint32_t a_width, uint32_t a_height, HydraRenderPreset a_preset)
{
  assert(a_width <= MAX_WIDTH && a_height <= MAX_HEIGHT);

  m_width  = a_width;
  m_height = a_height;
  m_preset = a_preset;

  realColor.resize(m_width * m_height * 4);
}

  bool HydraRenderer::LoadScene(const char* a_scenePath)
  {
    if (m_pImpl)
      m_pImpl.reset();

    //get info about features in scene
    std::string sceneDir       = "";
    SceneInfo sceneInfo = {};
    sceneInfo.spectral  = false;
    auto features = Integrator::PreliminarySceneAnalysis(a_scenePath, sceneDir.c_str(), &sceneInfo);

    //create Hydra renderer for this scene
#ifdef USE_GPU
    if(m_device == DEVICE_GPU || m_device == DEVICE_GPU_RTX)
    {
      unsigned int a_preferredDeviceId = 0;

      size_t gpuAuxMemSize = MAX_WIDTH*MAX_HEIGHT*sizeof(float) + 16 * 1024 * 1024; // reserve for frame buffer and other

      // advanced way, init device with features which is required by generated class
      std::vector<const char*> requiredExtensions;
      auto deviceFeatures = Integrator_Generated::ListRequiredDeviceFeatures(requiredExtensions);
      auto ctx            = vk_utils::globalContextInit(requiredExtensions, true, a_preferredDeviceId, &deviceFeatures, gpuAuxMemSize, 1);

      // advanced way, you can disable some pipelines creation which you don't actually need;
      // this will make application start-up faster
      Integrator_Generated::EnabledPipelines().enableRayTraceMega               = true;
      Integrator_Generated::EnabledPipelines().enableCastSingleRayMega          = false; // not used, for testing only
      Integrator_Generated::EnabledPipelines().enablePackXYMega                 = true;  // always true for this main.cpp;
      Integrator_Generated::EnabledPipelines().enablePathTraceFromInputRaysMega = false; // always false in this main.cpp; see cam_plugin main
      Integrator_Generated::EnabledPipelines().enablePathTraceMega              = true;
      Integrator_Generated::EnabledPipelines().enableNaivePathTraceMega         = false;

      // advanced way
      auto pObj = std::make_shared<Integrator_Generated>(MAX_WIDTH*MAX_HEIGHT, features);
      pObj->SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
      pObj->SetResourcesDir("./dependencies/HydraCore3"); //directory to search for shaders
      pObj->SetVulkanContext(ctx);
      pObj->InitVulkanObjects(ctx.device, ctx.physicalDevice, MAX_WIDTH*MAX_HEIGHT);
      m_pImpl = pObj;
    }
    else
#endif
    {
      m_pImpl = std::make_shared<Integrator>(MAX_WIDTH*MAX_HEIGHT,features);
      m_pImpl->SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
    }

    SetViewport(0, 0, m_width, m_height);

    //actually load scene
    m_pImpl->LoadScene(a_scenePath, sceneDir.c_str());

    return true;
  }

  void HydraRenderer::Clear(uint32_t a_width, uint32_t a_height, const char* a_what)
  {
    assert(m_pImpl);
    m_pImpl->PackXYBlock(m_width, m_height, 1);
  }
  void HydraRenderer::Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum)
  {
    assert(a_width == m_width && a_height == m_height);
    assert(m_pImpl);

    float timings[4] = {0,0,0,0};
    m_pImpl->SetFrameBufferLayer(m_preset.fbLayer);
    // m_pImpl->SetCamId(0);
    m_pImpl->SetViewport(0, 0, m_width, m_height);

    std::cout << "[main]: PathTraceBlock(MIS-PT) ... " << std::endl;
    std::fill(realColor.begin(), realColor.end(), 0.0f);
    
    m_pImpl->SetIntegratorType(m_preset.integratorType);
    m_pImpl->UpdateMembersPlainData();

    for (int i = 0; i < a_passNum; i++)
      m_pImpl->PathTraceBlock(m_width*m_height, 4, realColor.data(), m_preset.spp);
      
    m_pImpl->GetExecutionTime("PathTraceBlock", timings);
    std::cout << "PathTraceBlock(exec) = " << timings[0]              << " ms " << std::endl;
    std::cout << "PathTraceBlock(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
    std::cout << "PathTraceBlock(ovrh) = " << timings[3]              << " ms " << std::endl;
    
    const float normConst = 1.0f/float(a_passNum*m_preset.spp);
    toLDRImage(realColor.data(), m_width, m_height, normConst, GAMMA, imageData, true);
  }

  void HydraRenderer::SetViewport(int a_xStart, int a_yStart, int a_width, int a_height) 
  {
    assert(a_width == m_width && a_height == m_height);

    m_pImpl->SetSpectralMode(false);
    m_pImpl->SetFrameBufferSize(m_width, m_height);
    m_pImpl->SetViewport(a_xStart,a_yStart,m_width,m_height);
  }

  void HydraRenderer::SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct)
  {
    assert(m_pImpl);
    m_pImpl->SetAccelStruct(a_customAccelStruct);
  }

  std::shared_ptr<ISceneObject> HydraRenderer::GetAccelStruct()
  {
    assert(m_pImpl);
    return m_pImpl->m_pAccelStruct;
  }
  
  void HydraRenderer::GetExecutionTime(const char* a_funcName, float a_out[4])
  {
    assert(m_pImpl);
    m_pImpl->GetExecutionTime(a_funcName, a_out);
  }

  void HydraRenderer::CommitDeviceData()
  {
    assert(m_pImpl);
    m_pImpl->CommitDeviceData();
  }

  void HydraRenderer::UpdateCamera(const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj)
  {
    assert(m_pImpl);
    m_pImpl->SetWorldView(a_worldView);
    m_pImpl->SetProj(a_proj);
  }

void hydra_integration_example(unsigned device, std::string scene_filename)
{
  int FB_WIDTH        = 512;
  int FB_HEIGHT       = 512;
  int FB_CHANNELS     = 4;

  int PASS_NUMBER     = 512; //spp

  std::string scenePath      = scene_filename;
  std::string sceneDir       = "";          // alternative path of scene library root folder (by default it is the folder where scene xml is located)
  std::string imageOut       = "saves/hydra_test_out.png";
  std::string integratorType = "mispt";
  std::string fbLayer        = "color";
  std::string resourceDir    = ".";
  float gamma                = 2.4f; // out gamma, special value, see save image functions.

  int camId = 0;

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Integrator> pImpl = nullptr;

  SceneInfo sceneInfo = {};
  sceneInfo.spectral  = false;
  auto features = Integrator::PreliminarySceneAnalysis(scenePath.c_str(), sceneDir.c_str(), &sceneInfo);

  std::vector<float> realColor(FB_WIDTH*FB_HEIGHT*FB_CHANNELS);
#ifdef USE_GPU
  if(device == DEVICE_GPU || device == DEVICE_GPU_RTX)
  {
    unsigned int a_preferredDeviceId = 0;

    // simple way
    //
    //auto ctx = vk_utils::globalContextGet(enableValidationLayers, a_preferredDeviceId);
    //pImpl = CreateIntegrator_Generated(FB_WIDTH*FB_HEIGHT, spectral_mode, features, ctx, FB_WIDTH*FB_HEIGHT);

    size_t gpuAuxMemSize = FB_WIDTH*FB_HEIGHT*FB_CHANNELS*sizeof(float) + 16 * 1024 * 1024; // reserve for frame buffer and other

    // advanced way, init device with features which is required by generated class
    //
    std::vector<const char*> requiredExtensions;
    auto deviceFeatures = Integrator_Generated::ListRequiredDeviceFeatures(requiredExtensions);
    auto ctx            = vk_utils::globalContextInit(requiredExtensions, true, a_preferredDeviceId, &deviceFeatures, gpuAuxMemSize, 1);

    // advanced way, you can disable some pipelines creation which you don't actually need;
    // this will make application start-up faster
    //
    Integrator_Generated::EnabledPipelines().enableRayTraceMega               = true;
    Integrator_Generated::EnabledPipelines().enableCastSingleRayMega          = false; // not used, for testing only
    Integrator_Generated::EnabledPipelines().enablePackXYMega                 = true;  // always true for this main.cpp;
    Integrator_Generated::EnabledPipelines().enablePathTraceFromInputRaysMega = false; // always false in this main.cpp; see cam_plugin main
    Integrator_Generated::EnabledPipelines().enablePathTraceMega              = true;
    Integrator_Generated::EnabledPipelines().enableNaivePathTraceMega         = false;

    // advanced way
    //
    auto pObj = std::make_shared<Integrator_Generated>(FB_WIDTH*FB_HEIGHT, features);
    pObj->SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
    pObj->SetResourcesDir("./dependencies/HydraCore3"); //directory to search for shaders
    pObj->SetVulkanContext(ctx);
    pObj->InitVulkanObjects(ctx.device, ctx.physicalDevice, FB_WIDTH*FB_HEIGHT);
    pImpl = pObj;
  }
  else
  #endif
  {
    pImpl = std::make_shared<Integrator>(FB_WIDTH*FB_HEIGHT,features);
    pImpl->SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
  }

  const int vpStartX = 0;
  const int vpStartY = 0;
  const int vpSizeX  = FB_WIDTH;
  const int vpSizeY  = FB_HEIGHT;

  pImpl->SetSpectralMode(false);
  pImpl->SetFrameBufferSize(FB_WIDTH, FB_HEIGHT);
  pImpl->SetViewport(vpStartX,vpStartY,vpSizeX,vpSizeY);
  std::cout << "[main]: Loading scene ... " << scenePath.c_str() << std::endl;
  pImpl->LoadScene(scenePath.c_str(), sceneDir.c_str());

  //if(override_camera_pos)
  //  pImpl->SetWorldView(look_at);

  pImpl->CommitDeviceData();

  std::cout << "[main]: PackXYBlock() ... " << std::endl;
  pImpl->PackXYBlock(vpSizeX, vpSizeY, 1);

  std::vector<float> directLightCopy;
  bool splitDirectAndIndirect = false;
  float timings[4] = {0,0,0,0};
  pImpl->SetFrameBufferLayer(Integrator::FB_COLOR);
  pImpl->SetCamId(0);
  pImpl->SetViewport(vpStartX,vpStartY,vpSizeX,vpSizeY);

  std::cout << "[main]: PathTraceBlock(MIS-PT) ... " << std::endl;
  std::fill(realColor.begin(), realColor.end(), 0.0f);
  
  pImpl->SetIntegratorType(Integrator::INTEGRATOR_MIS_PT);
  pImpl->UpdateMembersPlainData();

  pImpl->PathTraceBlock(vpSizeX*vpSizeY, FB_CHANNELS, realColor.data(), PASS_NUMBER);
    
  pImpl->GetExecutionTime("PathTraceBlock", timings);
  std::cout << "PathTraceBlock(exec) = " << timings[0]              << " ms " << std::endl;
  std::cout << "PathTraceBlock(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
  std::cout << "PathTraceBlock(ovrh) = " << timings[3]              << " ms " << std::endl;
  
  const float normConst = 1.0f/float(PASS_NUMBER);
  LiteImage::Image2D<uint32_t> tmp(FB_WIDTH, FB_HEIGHT);
  toLDRImage(realColor.data(), FB_WIDTH, FB_HEIGHT, normConst, gamma, tmp.data(), true);
  for (int y = 0; y < FB_HEIGHT; y++)
  {
    for (int x = 0; x < FB_WIDTH; x++)
    {
      //printf("%.2f %.2f %.2f %.2f\n", realColor[4 * (y*FB_WIDTH + x) + 0], realColor[4 * (y*FB_WIDTH + x) + 1], realColor[4 * (y*FB_WIDTH + x) + 2], realColor[4 * (y*FB_WIDTH + x) + 3]);
    }
  }
  LiteImage::SaveImage(imageOut.c_str(), tmp);
}