#include "hydra_integration.h"
#include "HydraCore3/integrator_pt.h"

void toLDRImage(const float *rgb, int width, int height, float a_normConst, float a_gamma, std::vector<uint32_t> &pixelData, bool a_flip)
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

std::shared_ptr<ISceneObject> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName);

void hydra_integration_example(std::string scene_filename)
{
  int FB_WIDTH        = 256;
  int FB_HEIGHT       = 256;
  int FB_CHANNELS     = 4;

  int PASS_NUMBER     = 16; //spp

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

  pImpl = std::make_shared<Integrator>(FB_WIDTH*FB_HEIGHT,features);
  pImpl->SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));

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
  toLDRImage(realColor.data(), FB_WIDTH, FB_HEIGHT, normConst, gamma, const_cast< std::vector<uint32_t>& >(tmp.vector()), true);
  for (int y = 0; y < FB_HEIGHT; y++)
  {
    for (int x = 0; x < FB_WIDTH; x++)
    {
      //printf("%.2f %.2f %.2f %.2f\n", realColor[4 * (y*FB_WIDTH + x) + 0], realColor[4 * (y*FB_WIDTH + x) + 1], realColor[4 * (y*FB_WIDTH + x) + 2], realColor[4 * (y*FB_WIDTH + x) + 3]);
    }
  }
  LiteImage::SaveImage(imageOut.c_str(), tmp);
}