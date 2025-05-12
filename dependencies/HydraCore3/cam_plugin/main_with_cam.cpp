#include <iostream>
#include <fstream>
#include <filesystem>

#include "integrator_pt.h"
#include "ArgParser.h"
#include "CamPluginAPI.h"
#include "CamPinHole.h"
#include "CamTableLens.h"

bool SaveImage4fToEXR(const float* rgb, int width, int height, const char* outfilename, float a_normConst = 1.0f, bool a_invertY = false);
bool SaveImage4fToBMP(const float* rgb, int width, int height, int channels, const char* outfilename, float a_normConst, float a_gamma);

int main(int argc, const char** argv)
{
  #ifndef NDEBUG
  bool enableValidationLayers = true;
  #else
  bool enableValidationLayers = false;
  #endif

  int WIN_WIDTH  = 1024;
  int WIN_HEIGHT = 1024;
  int SPP_TOTAL  = 1024;
  int CHANNELS   = 4;

  std::string scenePath      = "../resources/HydraCore/hydra_app/tests/test_42/statex_00001.xml"; 
  std::string sceneDir       = "";          // alternative path of scene library root folder (by default it is the folder where scene xml is located)
  std::string imageOut       = "z_out.bmp";
  std::string integratorType = "mispt";
  std::string opticFile      = "optics.dat";
  float gamma                = 2.4f; // out gamma, special value, see save image functions

  std::shared_ptr<Integrator>  pRender  = nullptr;
  std::shared_ptr<ICamRaysAPI2> pCamImpl = nullptr;

  ArgParser args(argc, argv);
  
  if(args.hasOption("-in"))
    scenePath = args.getOptionValue<std::string>("-in");

  if(args.hasOption("-out"))
    imageOut = args.getOptionValue<std::string>("-out");

  if(args.hasOption("-scn_dir"))
    sceneDir = args.getOptionValue<std::string>("-scn_dir");

  if(args.hasOption("-optics_file"))
    opticFile = args.getOptionValue<std::string>("-optics_file");

  int camType = 1;

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  int spectral_mode = args.hasOption("--spectral") ? 1 : 0;
 
  //// override parameters which are explicitly defined in command line
  //
  if(args.hasOption("-width"))
    WIN_WIDTH = args.getOptionValue<int>("-width");
  if(args.hasOption("-height"))
    WIN_HEIGHT = args.getOptionValue<int>("-height");
  if(args.hasOption("--spectral"))
    spectral_mode = 1;
  
  if(spectral_mode == 1) /////////////////////////////////////////////////////////////// (!!!) single wave per ray in spectral mode (!!!)
    CHANNELS = 1;
  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////

  std::filesystem::path out_path {imageOut};
  auto dir = out_path.parent_path();
  if(!dir.empty() && !std::filesystem::exists(dir))
    std::filesystem::create_directories(dir);

  const bool saveHDR = imageOut.find(".exr") != std::string::npos;
  const std::string imageOutClean = imageOut.substr(0, imageOut.find_last_of("."));
  
  if(args.hasOption("-gamma")) {
    std::string gammaText = args.getOptionValue<std::string>("-gamma");
    if(gammaText == "srgb" || gammaText == "sSRGB")
      gamma = 2.4f;
    else
      gamma = args.getOptionValue<float>("-gamma");
  }
  
  if(args.hasOption("-width"))
    WIN_WIDTH = args.getOptionValue<int>("-width");
  if(args.hasOption("-height"))
    WIN_HEIGHT = args.getOptionValue<int>("-height");
  
  
  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  
  const int MEGA_TILE_SIZE = 512*512;                      ///<! tile size

  std::vector<RayPosAndW> rayPos(MEGA_TILE_SIZE);          ///<! per tile data, input 
  std::vector<RayDirAndT> rayDir(MEGA_TILE_SIZE);          ///<! per tile data, input
  std::vector<float>      rayCol(MEGA_TILE_SIZE*CHANNELS); ///<! per tile data, output 

  std::vector<float4>   realColor(WIN_WIDTH*WIN_HEIGHT);             ///<! frame buffer, always float4 in this demo
  std::fill(realColor.begin(), realColor.end(), LiteMath::float4{}); // clear frame buffer
  
  SceneInfo sceneInfo = {};
  sceneInfo.spectral  = spectral_mode;
  auto features = Integrator::PreliminarySceneAnalysis(scenePath.c_str(), sceneDir.c_str(), &sceneInfo);

  {
    pRender = std::make_shared<Integrator>(MEGA_TILE_SIZE, features);

    if(camType == 0)
      pCamImpl = std::make_shared<CamPinHole>(); // (WIN_WIDTH*WIN_HEIGHT);
    else if(camType == 1)
      pCamImpl = std::make_shared<CamTableLens>(); // (WIN_WIDTH*WIN_HEIGHT);
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  
  std::cout << "[main_with_cam]: Loading scene ... " << scenePath.c_str() << std::endl;

  pCamImpl->SetParameters(WIN_WIDTH, WIN_HEIGHT, {45.0f, 1.0f, 0.01f, 100.0f, spectral_mode, opticFile});
  pCamImpl->SetBatchSize(MEGA_TILE_SIZE);

  pRender->LoadScene(scenePath.c_str(), sceneDir.c_str());
  pRender->SetIntegratorType(Integrator::INTEGRATOR_MIS_PT);

  pRender->CommitDeviceData();
  pCamImpl->CommitDeviceData();

  SPP_TOTAL = pRender->GetSPP();                     // read target spp from scene
  if(args.hasOption("-spp"))                         // override it if spp is specified via command line
    SPP_TOTAL = args.getOptionValue<int>("-spp");

  int SAMPLES_PER_RAY = 1;                          
  int CAM_PASSES_NUM  = SPP_TOTAL/SAMPLES_PER_RAY; 
  if(SPP_TOTAL == 1) {
    SAMPLES_PER_RAY = 1;
    CAM_PASSES_NUM = 1;
  }

  std::cout << "[main_with_cam]: spp     = " << SPP_TOTAL << std::endl;
  std::cout << "[main_with_cam]: passNum = " << CAM_PASSES_NUM << std::endl;

  float timings  [4] = {0,0,0,0};
  float timingSum[4] = {0,0,0,0};
  const float normConst = 1.0f/float(SPP_TOTAL);

  // do rendering
  //
  for(int passId = 0; passId < CAM_PASSES_NUM; passId++)
  {
    std::cout << "rendering, pass " << passId << " / " << CAM_PASSES_NUM  << "\r"; 
    std::cout.flush();
    
    const int passNum = (WIN_WIDTH*WIN_HEIGHT/MEGA_TILE_SIZE);
    for(int subPassId = 0; subPassId < passNum; subPassId++) 
    {
      std::fill(rayCol.begin(), rayCol.end(), 0.0f); // clear temp color buffer, gpu ver should do this automaticly, please check(!!!)
      
      pCamImpl->MakeRaysBlock(rayPos.data(), rayDir.data(), MEGA_TILE_SIZE, subPassId);
      pRender->PathTraceFromInputRaysBlock(MEGA_TILE_SIZE, CHANNELS, rayPos.data(), rayDir.data(), rayCol.data(), SAMPLES_PER_RAY);
      pCamImpl->AddSamplesContributionBlock((float*)realColor.data(), (const float*)rayCol.data(), MEGA_TILE_SIZE, WIN_WIDTH, WIN_HEIGHT, subPassId);

      pRender->GetExecutionTime("PathTraceFromInputRaysBlock", timings);
      for(int i=0;i<4;i++)
        timingSum[i] += timings[i];
    }
  }

  std::cout << std::endl << std::endl;
  std::cout << "PathTraceFromInputRays(exec, total) = " << timingSum[0]                << " ms " << std::endl;
  std::cout << "PathTraceFromInputRays(copy, total) = " << timingSum[1] + timingSum[2] << " ms " << std::endl;
  std::cout << "PathTraceFromInputRays(ovrh, total) = " << timingSum[3]                << " ms " << std::endl;

  if(saveHDR) 
    SaveImage4fToEXR((const float*)realColor.data(), WIN_WIDTH, WIN_HEIGHT, imageOut.c_str(), normConst, true);
  else
    SaveImage4fToBMP((const float*)realColor.data(), WIN_WIDTH, WIN_HEIGHT, 4, imageOut.c_str(), normConst, gamma);
  
  return 0;
}
