#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

#include "imageutils.h"
#include "integrator_pt.h"
#include "ArgParser.h"
#include "mi_materials.h"

float4x4 ReadMatrixFromString(const std::string& str);
std::shared_ptr<Integrator> CreateIntegratorQMC(int a_maxThreads = 1, std::vector<uint32_t> a_features = {});
std::shared_ptr<Integrator> CreateIntegratorKMLT(int a_maxThreads = 1, std::vector<uint32_t> a_features = {});

#ifdef USE_VULKAN
#include "vk_context.h"
#include "integrator_pt_generated.h" // advanced way
//std::shared_ptr<Integrator> CreateIntegrator_Generated(int a_maxThreads, std::vector<uint32_t> a_features, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated); // simple way
#endif

#ifdef USE_STB_IMAGE
  #define SaveLDRImageM SaveImage4fByExtension
#else
  #define SaveLDRImageM SaveImage4fToBMP
#endif

void SaveGBufferImages(const std::string& imageOutClean, const std::string& imageOutFiExt, 
                       const std::vector<Integrator::GBufferPixel>& gbuffer, std::vector<float>& tmp, uint width, uint height);

int main(int argc, const char** argv) // common hydra main
{
  #ifndef NDEBUG
  bool enableValidationLayers = true;
  #else
  bool enableValidationLayers = false;
  #endif

  //// test saving 3D image
  //{
  //  std::vector<float> data(640*480*8);
  //  for(int z=0;z<8;z++)
  //    for(int y=0;y<480;y++)
  //      for(int x=0;x<640;x++)
  //        data[z*(640*480) + y*640 + x] = 0.1f*float(z) + 0.05f*(float(x) + float(y) + 2.0f*float(z)*float(z)) / (std::sqrt(float(x*y)) + 0.1f);
  //  SaveFrameBufferToEXR(data.data(), 640, 480, 8, "z_test.exr", 1.0f);
  //  exit(0);
  //}

  int FB_WIDTH        = 1024;
  int FB_HEIGHT       = 1024;
  int FB_CHANNELS     = 4;

  int PASS_NUMBER     = 1024;
  int NAIVE_PT_REPEAT = 1; // make more samples for naivept which is quite useful for testing cases to get less noise for

  std::string scenePath      = "../resources/HydraCore/hydra_app/tests/test_42/statex_00001.xml";
  std::string sceneDir       = "";          // alternative path of scene library root folder (by default it is the folder where scene xml is located)
  std::string imageOut       = "z_out.bmp";
  std::string integratorType = "mispt";
  std::string fbLayer        = "color";
  std::string resourceDir    = ".";
  float gamma                = 2.4f; // out gamma, special value, see save image functions.

  std::vector<int> camsId;

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Integrator> pImpl = nullptr;
  ArgParser args(argc, argv);

  for(int argId = 1; argId < argc; argId++)
  {
    if(std::string(argv[argId]) == "-cam_id")
    {
      camsId.push_back(std::atoi(argv[argId+1]));
      argId++;
    }
  }

  if(camsId.size() == 0)
    camsId.push_back(0); // set first camera by default

  std::cout << "[main]: camsId.size() = " << camsId.size() << std::endl;

  if(args.hasOption("-in"))
    scenePath = args.getOptionValue<std::string>("-in");

  if(args.hasOption("-out"))
    imageOut = args.getOptionValue<std::string>("-out");

  std::filesystem::path out_path {imageOut};
  auto dir = out_path.parent_path();
  if(!dir.empty() && !std::filesystem::exists(dir))
    std::filesystem::create_directories(dir);

  if(args.hasOption("-scn_dir"))
    sceneDir = args.getOptionValue<std::string>("-scn_dir");

  if(args.hasOption("-resource_dir"))
    resourceDir = args.getOptionValue<std::string>("-resource_dir");

  const bool saveHDR = imageOut.find(".exr") != std::string::npos ||
                       imageOut.find(".image1f") != std::string::npos ||
                       imageOut.find(".image4f") != std::string::npos ||
                       imageOut.find(".image3d1f") != std::string::npos;

  const std::string imageOutClean = imageOut.substr(0, imageOut.find_last_of(".")); // "image.png" ==> "image"
  const std::string imageOutFiExt = imageOut.substr(imageOut.find_last_of(".")+1);  // "image.png" ==> ".png"

  if(args.hasOption("-integrator"))
    integratorType = args.getOptionValue<std::string>("-integrator");

  if(args.hasOption("-fb_layer"))
    fbLayer = args.getOptionValue<std::string>("-fb_layer");

  if(args.hasOption("-spp-naive-mul"))
    NAIVE_PT_REPEAT = std::max(args.getOptionValue<int>("-spp-naive-mul"),1);

  if(args.hasOption("-gamma")) {
    std::string gammaText = args.getOptionValue<std::string>("-gamma");
    if(gammaText == "srgb" || gammaText == "sSRGB")
      gamma = 2.4f;
    else
      gamma = args.getOptionValue<float>("-gamma");
  }

  bool evalGBuffer = false; 
  {
    if(args.hasOption("-evalgbuffer")) 
      evalGBuffer = (args.getOptionValue<int>("-evalgbuffer") != 0);
    if(fbLayer == "gbuffer")
      evalGBuffer = true;
  }
  
  int  spectral_mode = args.hasOption("--spectral") ? 1 : 0;
  bool qmcIsEnabled  = args.hasOption("--qmc");
  bool mltIsEnabled  = false;
  if(integratorType == "mlt" || integratorType == "kmlt" || integratorType == "kelemen_mlt")
  {
    mltIsEnabled   = true;
    integratorType = "mispt"; 
    if(fbLayer == "color")
      fbLayer = "split_direct_indirect";
  }

  float4x4 look_at;
  auto override_camera_pos = args.hasOption("-look_at");
  if(override_camera_pos)
  {
    auto str = args.getOptionValue<std::string>("-look_at");
    std::cout << str << std::endl;
    look_at = ReadMatrixFromString(str);
  }

  const bool enableNaivePT  = (integratorType == "naivept" || integratorType == "all");
  const bool enableShadowPT = (integratorType == "shadowpt" || integratorType == "all");
  const bool enableMISPT    = (integratorType == "mispt" || integratorType == "all");
  const bool enableRT       = (integratorType == "raytracing" || integratorType == "rt" || integratorType == "whitted_rt");
  const bool enablePRT      = (integratorType == "primary" || integratorType == "prt");

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  std::cout << "[main]: loading xml ... " << scenePath.c_str() << std::endl;

  SceneInfo sceneInfo = {};
  sceneInfo.spectral  = spectral_mode;
  auto features = Integrator::PreliminarySceneAnalysis(scenePath.c_str(), sceneDir.c_str(), &sceneInfo);
  FB_WIDTH      = sceneInfo.width;
  FB_HEIGHT     = sceneInfo.height;
  spectral_mode = sceneInfo.spectral;

  //// override parameters which are explicitly defined in command line
  //
  if(args.hasOption("-width"))
    FB_WIDTH = args.getOptionValue<int>("-width");
  if(args.hasOption("-height"))
    FB_HEIGHT = args.getOptionValue<int>("-height");
  if(args.hasOption("-channels"))
    FB_CHANNELS = args.getOptionValue<int>("-channels");
  if(args.hasOption("--spectral"))
    spectral_mode = 1;

  if(FB_CHANNELS == 2 || FB_CHANNELS == 3) // we don't support these values currently
    FB_CHANNELS = 4;
  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////

  std::vector<float> realColor(FB_WIDTH*FB_HEIGHT*FB_CHANNELS);

  bool onGPU = args.hasOption("--gpu") && !evalGBuffer;
  #ifdef USE_VULKAN
  if(onGPU)
  {
    unsigned int a_preferredDeviceId = args.getOptionValue<int>("-gpu_id", 0);

    // simple way
    //
    //auto ctx = vk_utils::globalContextGet(enableValidationLayers, a_preferredDeviceId);
    //pImpl = CreateIntegrator_Generated(FB_WIDTH*FB_HEIGHT, spectral_mode, features, ctx, FB_WIDTH*FB_HEIGHT);

    size_t gpuAuxMemSize = FB_WIDTH*FB_HEIGHT*FB_CHANNELS*sizeof(float) + 16 * 1024 * 1024; // reserve for frame buffer and other

    // advanced way, init device with features which is required by generated class
    //
    std::vector<const char*> requiredExtensions;
    auto deviceFeatures = Integrator_Generated::ListRequiredDeviceFeatures(requiredExtensions);
    auto ctx            = vk_utils::globalContextInit(requiredExtensions, enableValidationLayers, a_preferredDeviceId, &deviceFeatures, gpuAuxMemSize, 1);

    // advanced way, you can disable some pipelines creation which you don't actually need;
    // this will make application start-up faster
    //
    Integrator_Generated::EnabledPipelines().enableRayTraceMega               = enableRT;
    Integrator_Generated::EnabledPipelines().enableCastSingleRayMega          = false; // not used, for testing only
    Integrator_Generated::EnabledPipelines().enablePackXYMega                 = true;  // always true for this main.cpp;
    Integrator_Generated::EnabledPipelines().enablePathTraceFromInputRaysMega = false; // always false in this main.cpp; see cam_plugin main
    Integrator_Generated::EnabledPipelines().enablePathTraceMega              = enableShadowPT || enableMISPT;
    Integrator_Generated::EnabledPipelines().enableNaivePathTraceMega         = enableNaivePT;

    // advanced way
    //
    auto pObj = std::make_shared<Integrator_Generated>(FB_WIDTH*FB_HEIGHT, features);
    pObj->SetVulkanContext(ctx);
    pObj->InitVulkanObjects(ctx.device, ctx.physicalDevice, FB_WIDTH*FB_HEIGHT);
    pImpl = pObj;
  }
  else
  #endif
  {
    if(mltIsEnabled)
      pImpl = CreateIntegratorKMLT(FB_WIDTH*FB_HEIGHT, features);
    else if(qmcIsEnabled)
      pImpl = CreateIntegratorQMC(FB_WIDTH*FB_HEIGHT, features);
    else
      pImpl = std::make_shared<Integrator>(FB_WIDTH*FB_HEIGHT,features);
  }
  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////

  const int vpStartX = 0;
  const int vpStartY = 0;
  const int vpSizeX  = FB_WIDTH;
  const int vpSizeY  = FB_HEIGHT;

  pImpl->SetSpectralMode(spectral_mode);
  pImpl->SetFrameBufferSize(FB_WIDTH, FB_HEIGHT);
  pImpl->SetViewport(vpStartX,vpStartY,vpSizeX,vpSizeY);
  std::cout << "[main]: Loading scene ... " << scenePath.c_str() << std::endl;
  pImpl->LoadScene(scenePath.c_str(), sceneDir.c_str());

  if(override_camera_pos)
    pImpl->SetWorldView(look_at);

  pImpl->CommitDeviceData();

  PASS_NUMBER = pImpl->GetSPP();                     // read target spp from scene
  if(args.hasOption("-spp"))                         // override it if spp is specified via command line
    PASS_NUMBER = args.getOptionValue<int>("-spp");

  // remember (x,y) coords for each thread to make our threading 1D
  //
  std::cout << "[main]: PackXYBlock() ... " << std::endl;
  pImpl->PackXYBlock(vpSizeX, vpSizeY, 1);

  if(evalGBuffer)
  {
    std::cout << "[main]: EvalGBuffer() ... " << std::endl;

    std::vector<Integrator::GBufferPixel> gbuffer(FB_WIDTH*FB_HEIGHT);
    pImpl->EvalGBuffer(FB_WIDTH*FB_HEIGHT, gbuffer.data());
    
    SaveGBufferImages(imageOutClean, imageOutFiExt, gbuffer, realColor, FB_WIDTH, FB_HEIGHT);
    return 0;
  }

  std::vector<uint32_t> fbLayers = {Integrator::FB_COLOR};
  std::vector<float> directLightCopy;
  bool splitDirectAndIndirect = false;
  {
    if(fbLayer == "direct" || fbLayer == "direct_light")
      fbLayers = {Integrator::FB_DIRECT};
    else if(fbLayer == "indirect" || fbLayer == "indirect_light" || fbLayer == "secondary" || fbLayer == "secondary_light")
      fbLayers = {Integrator::FB_INDIRECT};
    else if(fbLayer == "all" || fbLayer == "both" || fbLayer == "split_direct_indirect") {
      fbLayers = {Integrator::FB_DIRECT, Integrator::FB_INDIRECT};
      splitDirectAndIndirect = true;
    }
  }
  
  //////////////////////////////////////////////////////////////////////////////////////
  //
  struct FrameData
  {
    int32_t     camId;
    uint32_t    fbId;
    std::string suffix;
  };
  
  std::vector<FrameData> frames;
  for(auto camId : camsId) {
    for(auto imLayer : fbLayers) {
      std::string suffix = splitDirectAndIndirect ? "_" + std::to_string(imLayer) : "";
      std::string suffixCam = "";
      if(camsId.size() > 1) {
        std::stringstream strOut;
        strOut << "_cam_" << std::setfill('0') << std::setw(3) << camId;
        suffixCam += strOut.str(); //std::string("_cam_") + std::to_string(camId);
      }
      suffix += suffixCam;

      FrameData frame;
      frame.camId  = camId;
      frame.fbId   = imLayer;
      frame.suffix = suffix;
      frames.push_back(frame);
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////

  for(size_t i=0;i<frames.size();i++) 
  {
    auto frame = frames[i];
    float timings[4] = {0,0,0,0};
    pImpl->SetFrameBufferLayer(frame.fbId);
    pImpl->SetCamId(frame.camId);
    pImpl->SetViewport(vpStartX,vpStartY,vpSizeX,vpSizeY);
    std::cout << "[main]: frame = " << i << " / "  << frames.size() << ", cam_id = " << frame.camId << ", fb_layer = " << frame.fbId  << std::endl;
    
    const auto& suffix = frame.suffix;

    if(enableNaivePT)
    {
      std::cout << "[main]: NaivePathTraceBlock() ... " << std::endl;
      std::fill(realColor.begin(), realColor.end(), 0.0f);
  
      pImpl->SetIntegratorType(Integrator::INTEGRATOR_STUPID_PT);
      pImpl->UpdateMembersPlainData();
      pImpl->NaivePathTraceBlock(FB_WIDTH*FB_HEIGHT, FB_CHANNELS, realColor.data(), PASS_NUMBER*NAIVE_PT_REPEAT);
  
      std::cout << std::endl;
      pImpl->GetExecutionTime("NaivePathTraceBlock", timings);
      std::cout << "NaivePathTraceBlock(exec)  = " << timings[0]              << " ms " << std::endl;
      std::cout << "NaivePathTraceBlock(copy)  = " << timings[1] + timings[2] << " ms " << std::endl;
      std::cout << "NaivePathTraceBlock(ovrh)  = " << timings[3]              << " ms " << std::endl;
      std::cout << std::endl;
  
      const float normConst = 1.0f/float(PASS_NUMBER*NAIVE_PT_REPEAT);
  
      if(saveHDR)
      {
        const std::string outName = (integratorType == "naivept" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_naivept" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst);
      }
      else
      {
        const std::string outName = (integratorType == "naivept" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_naivept" + suffix  + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveLDRImageM(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst, gamma);
      }
    } // end if enableNaivePT

    const float normConst = 1.0f/float(PASS_NUMBER);
    if(enableShadowPT)
    {
      std::cout << "[main]: PathTraceBlock(Shadow-PT) ... " << std::endl;
  
      std::fill(realColor.begin(), realColor.end(), 0.0f);
  
      pImpl->SetIntegratorType(Integrator::INTEGRATOR_SHADOW_PT);
      pImpl->UpdateMembersPlainData();
      pImpl->PathTraceBlock(FB_WIDTH*FB_HEIGHT, FB_CHANNELS, realColor.data(), PASS_NUMBER);
  
      if(saveHDR)
      {
        const std::string outName = (integratorType == "shadowpt" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_shadowpt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst);
      }
      else
      {
        const std::string outName = (integratorType == "shadowpt" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_shadowpt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveLDRImageM(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst, gamma);
      }
    } // end if enableShadowPT

    if(enableMISPT)
    {
      std::cout << "[main]: PathTraceBlock(MIS-PT) ... " << std::endl;
      std::fill(realColor.begin(), realColor.end(), 0.0f);
  
      pImpl->SetIntegratorType(Integrator::INTEGRATOR_MIS_PT);
      pImpl->UpdateMembersPlainData();
      
      //std::vector<float> tileData(vpSizeX*vpSizeY*FB_CHANNELS);
      //std::fill(tileData.begin(), tileData.end(), 0.0f);
      
      pImpl->PathTraceBlock(vpSizeX*vpSizeY, FB_CHANNELS, realColor.data(), PASS_NUMBER);
      
      //// copy data from tile memory to FB memory
      //for(int y=0;y<vpSizeY;y++){
      //  const int yOffset1 = FB_WIDTH*(y+vpStartY);
      //  const int yOffset2 = vpSizeX*y;
      //  for(int x=0;x<vpSizeX;x++) {
      //    for(int c=0;c<FB_CHANNELS;c++) {
      //      realColor[(yOffset1 + x + vpStartX)*FB_CHANNELS + c] = tileData[(yOffset2 + x)*FB_CHANNELS + c];
      //    }
      //  }
      //}
  
      pImpl->GetExecutionTime("PathTraceBlock", timings);
      std::cout << "PathTraceBlock(exec) = " << timings[0]              << " ms " << std::endl;
      std::cout << "PathTraceBlock(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
      std::cout << "PathTraceBlock(ovrh) = " << timings[3]              << " ms " << std::endl;
  
      if(saveHDR)
      {
        const std::string outName = (integratorType == "mispt" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_mispt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst);
      }
      else
      {
        const std::string outName = (integratorType == "mispt" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_mispt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveLDRImageM(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst, gamma);
      }
    } // end if (enableMISPT)

    if(enableRT || enablePRT)
    {
      const float normConstRT = 1.0f;  // must be always one for RT currently
      std::cout << "[main]: RayBlock ... " << std::endl;
  
      std::fill(realColor.begin(), realColor.end(), 0.0f);
  
      pImpl->UpdateMembersPlainData();
      if(enablePRT)
      {
        pImpl->CastSingleRayBlock(FB_WIDTH*FB_HEIGHT, realColor.data(), 1);
        pImpl->GetExecutionTime("CastSingleRayBlock", timings);
        std::cout << "CastSingleRayBlock(exec) = " << timings[0]              << " ms " << std::endl;
        std::cout << "CastSingleRayBlock(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
        std::cout << "CastSingleRayBlock(ovrh) = " << timings[3]              << " ms " << std::endl;
      }
      else
      {
        pImpl->RayTraceBlock(FB_WIDTH*FB_HEIGHT, FB_CHANNELS, realColor.data(), 1);
        pImpl->GetExecutionTime("RayTraceBlock", timings);
        std::cout << "RayTraceBlock(exec) = " << timings[0]              << " ms " << std::endl;
        std::cout << "RayTraceBlock(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
        std::cout << "RayTraceBlock(ovrh) = " << timings[3]              << " ms " << std::endl;
      }
  
      if(saveHDR)
      {
        const std::string outName = (integratorType == "raytracing" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_rt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst);
      }
      else
      {
        const std::string outName = (integratorType == "raytracing" && !splitDirectAndIndirect) ? imageOutClean + suffix + "." + imageOutFiExt : imageOutClean + "_rt" + suffix + "." + imageOutFiExt;
        std::cout << "[main]: save image to " << outName.c_str() << std::endl;
        SaveLDRImageM(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConstRT, gamma);
      }
    } // end if(enableRT || enablePRT)
    
    if(frame.fbId == Integrator::FB_DIRECT) // preserve direct light in separate image
      directLightCopy = realColor;
  }
  
  // save finale image as summ direct and indirect light; TODO: add correct support for multiple cameras here
  //
  if(splitDirectAndIndirect) 
  {
    std::cout << "[main]: save final image to " << imageOut.c_str() << std::endl;  
    for(size_t i=0; i<realColor.size(); i++)
      realColor[i] += directLightCopy[i];
    
    const float normConst = 1.0f/float(PASS_NUMBER);
    if(saveHDR)
      SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, imageOut.c_str(), normConst);
    else
      SaveLDRImageM(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, imageOut.c_str(), normConst, gamma);
  }

  return 0;
}
