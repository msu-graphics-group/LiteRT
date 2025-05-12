#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>

#include "integrator_pt.h"
#include "ArgParser.h"
#include "CamPluginAPI.h"
#include "CamPinHole.h"
#include "CamTableLens.h"

bool SaveImage4fToEXR(const float* rgb, int width, int height, const char* outfilename, float a_normConst = 1.0f, bool a_invertY = false);
bool SaveImage4fToBMP(const float* rgb, int width, int height, int channels, const char* outfilename, float a_normConst, float a_gamma);

#include "vk_context.h"
#include "vk_buffers.h"

#include "integrator_pt_generated.h"              // advanced way of woking with hydra
#include "cam_plugin/CamPinHole_pinhole_gpu.h"     // same way for camera plugins
#include "cam_plugin/CamTableLens_tablelens_gpu.h" // same way for camera plugins

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

  ArgParser args(argc, argv);
  
  if(args.hasOption("-in"))
    scenePath = args.getOptionValue<std::string>("-in");

  if(args.hasOption("-out"))
    imageOut = args.getOptionValue<std::string>("-out");

  if(args.hasOption("-scn_dir"))
    sceneDir = args.getOptionValue<std::string>("-scn_dir");

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
  if(args.hasOption("-optics_file"))
    opticFile = args.getOptionValue<std::string>("-optics_file");
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
  
  const int MEGA_TILE_SIZE = 512*512;  ///<! tile size
 
  // (1) advanced way, you may disable unused features in shader code via spec constants.
  //     To do this, you have to know what materials, lights and e.t.c. is actualle presented in scene 
  //
  std::cout << "[main]: loading xml ... " << scenePath.c_str() << std::endl;
  SceneInfo sceneInfo = {};
  sceneInfo.spectral  = spectral_mode;
  auto hydraFeatures = Integrator::PreliminarySceneAnalysis(scenePath.c_str(), sceneDir.c_str(), &sceneInfo); 
  WIN_WIDTH     = sceneInfo.width;
  WIN_HEIGHT    = sceneInfo.height;
  spectral_mode = sceneInfo.spectral;

  if(args.hasOption("-width"))
    WIN_WIDTH = args.getOptionValue<int>("-width");
  if(args.hasOption("-height"))
    WIN_HEIGHT = args.getOptionValue<int>("-height");
  spectral_mode = args.hasOption("--spectral") ? 1 : 0;
  
  // (2) init device with apropriate features for both hydra and camera plugin
  //
  unsigned int preferredDeviceId = args.getOptionValue<int>("-gpu_id", 0);
  std::vector<const char*> requiredExtensions;
    
  auto devFeaturesCam = (camType == 0) ? CamPinHole_PINHOLE_GPU::ListRequiredDeviceFeatures(requiredExtensions) :
                                         CamTableLens_TABLELENS_GPU::ListRequiredDeviceFeatures(requiredExtensions);
                                           
  auto devFeaturesHydra = Integrator_Generated::ListRequiredDeviceFeatures(requiredExtensions); 
    
  // TBD: you actually need to carefully join all required device features structures and Vulkan lists 
  //
  if(devFeaturesCam.features.shaderFloat64 == VK_TRUE) // in this example we know that hydra3 don't use double precition  
    devFeaturesHydra.features.shaderFloat64 = VK_TRUE; // while cam plugin probably uses it ... 

  sceneInfo.memGeom += MEGA_TILE_SIZE*CHANNELS*sizeof(float)*3 + WIN_WIDTH*WIN_HEIGHT*4*sizeof(float); // memory for our image data 

  auto ctx = vk_utils::globalContextInit(requiredExtensions, enableValidationLayers, preferredDeviceId, &devFeaturesHydra, sceneInfo.memGeom, sceneInfo.memTextures); 

  // (3) Explicitly disable all pipelines which you don't need.
  //     This will make application start-up faster.
  //
  Integrator_Generated::EnabledPipelines().enableRayTraceMega               = false;
  Integrator_Generated::EnabledPipelines().enableCastSingleRayMega          = false; 
  Integrator_Generated::EnabledPipelines().enablePackXYMega                 = false; 
  Integrator_Generated::EnabledPipelines().enablePathTraceFromInputRaysMega = true;  // you need only this pipeline!
  Integrator_Generated::EnabledPipelines().enablePathTraceMega              = false;
  Integrator_Generated::EnabledPipelines().enableNaivePathTraceMega         = false;

  // advanced way, init renderer
  //
  auto pRender = std::make_shared<Integrator_Generated>(MEGA_TILE_SIZE, spectral_mode, hydraFeatures); 
  auto pCamImpl = std::make_shared<CamTableLens_TABLELENS_GPU>();
  //auto pCamImpl = std::make_shared<CamPinHole_PINHOLE_GPU>();  

  pRender->InitVulkanObjects(ctx.device, ctx.physicalDevice, MEGA_TILE_SIZE); 
  pCamImpl->InitVulkanObjects(ctx.device, ctx.physicalDevice, MEGA_TILE_SIZE); 

  // alloc all reauired buffers on GPU
  // 
  VkBuffer rayPosGPU = vk_utils::createBuffer(ctx.device, MEGA_TILE_SIZE*4*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  VkBuffer rayDirGPU = vk_utils::createBuffer(ctx.device, MEGA_TILE_SIZE*4*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  VkBuffer rayColGPU = vk_utils::createBuffer(ctx.device, MEGA_TILE_SIZE*CHANNELS*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);

  VkBuffer frameBuferGPU = vk_utils::createBuffer(ctx.device, WIN_WIDTH*WIN_HEIGHT*4*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);

  auto memObject = vk_utils::allocateAndBindWithPadding(ctx.device, ctx.physicalDevice, {rayPosGPU,rayDirGPU,rayColGPU,frameBuferGPU});

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  
  std::cout << "[main_cam_gpu]: Loading scene ... " << scenePath.c_str() << std::endl;

  pCamImpl->SetParameters(WIN_WIDTH, WIN_HEIGHT, {45.0f, 1.0f, 0.01f, 100.0f, spectral_mode, opticFile});
  pCamImpl->SetBatchSize(MEGA_TILE_SIZE);

  pRender->LoadScene(scenePath.c_str(), sceneDir.c_str());
  pRender->SetIntegratorType(Integrator::INTEGRATOR_MIS_PT);

  pRender->CommitDeviceData(ctx.pCopyHelper);        // copy internal camera     data from CPU to GPU
  pCamImpl->CommitDeviceData(ctx.pCopyHelper);       // copy internal integrator data from CPU to GPU

  SPP_TOTAL = pRender->GetSPP();                     // read target spp from scene
  if(args.hasOption("-spp"))                         // override it if spp is specified via command line
    SPP_TOTAL = args.getOptionValue<int>("-spp");

  int SAMPLES_PER_RAY = 1;                          
  int CAM_PASSES_NUM  = SPP_TOTAL/SAMPLES_PER_RAY; 
  if(SPP_TOTAL == 1) {
    SAMPLES_PER_RAY = 1;
    CAM_PASSES_NUM = 1;
  }

  std::cout << "[main_cam_gpu]: spp     = " << SPP_TOTAL << std::endl;
  std::cout << "[main_cam_gpu]: passNum = " << CAM_PASSES_NUM << std::endl;

  float timings   [4] = {0,0,0,0};
  float timingSum[4] = {0,0,0,0};
  const float normConst = 1.0f/float(SPP_TOTAL);

  // bind buffers
  //
  pCamImpl->SetVulkanInOutFor_MakeRaysBlock(rayPosGPU, 0, 
                                            rayDirGPU, 0);

  pRender->SetVulkanInOutFor_PathTraceFromInputRays(rayPosGPU, 0, 
                                                    rayDirGPU, 0, 
                                                    rayColGPU, 0);

  pCamImpl->SetVulkanInOutFor_AddSamplesContributionBlock(frameBuferGPU, 0, 
                                                          rayColGPU, 0);

  // do rendering
  //
  VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(ctx.device, ctx.commandPool);
  VkCommandBufferBeginInfo beginCommandBufferInfo = {};
  beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
  
  // std::fill(realColor.begin(), realColor.end(), LiteMath::float4{});
  //
  std::cout << "[main_cam_gpu]: clear frame buffer" << std::endl;
  vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
  vkCmdFillBuffer(commandBuffer, frameBuferGPU, 0, VK_WHOLE_SIZE, 0);
  vkEndCommandBuffer(commandBuffer);  
  vk_utils::executeCommandBufferNow(commandBuffer, ctx.computeQueue, ctx.device); 
  
  vkResetCommandBuffer(commandBuffer, 0);
  vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
  
  // loop over big image in several passes
  //
  const int passNum = (WIN_WIDTH*WIN_HEIGHT/MEGA_TILE_SIZE);
  for(int subPassId = 0; subPassId < passNum; subPassId++) 
  {  
    // std::fill(rayCol.begin(), rayCol.end(), LiteMath::float4{});
    //
    vkCmdFillBuffer(commandBuffer, rayColGPU, 0, VK_WHOLE_SIZE, 0); 
    {
      VkBufferMemoryBarrier bar = {};
      bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
      bar.pNext               = NULL;
      bar.srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
      bar.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
      bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      bar.buffer              = rayColGPU;
      bar.offset              = 0;
      bar.size                = VK_WHOLE_SIZE;
      vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 1, &bar, 0, nullptr); 
    } 

    pCamImpl->MakeRaysBlockCmd(commandBuffer, nullptr, nullptr, MEGA_TILE_SIZE, subPassId);
    pRender->PathTraceFromInputRaysCmd(commandBuffer, MEGA_TILE_SIZE, CHANNELS, nullptr, nullptr, nullptr);
    pCamImpl->AddSamplesContributionBlockCmd(commandBuffer, nullptr, nullptr, MEGA_TILE_SIZE, WIN_WIDTH, WIN_HEIGHT, subPassId);      
  }

  vkEndCommandBuffer(commandBuffer);  
  
  std::cout << "[main_cam_gpu]: rendering ... " << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  for(int passId = 0; passId < CAM_PASSES_NUM; passId++) {
    vk_utils::executeCommandBufferNow(commandBuffer, ctx.computeQueue, ctx.device);
    if(passId != 0 && passId%16 == 0)
      std::cout << "[main_cam_gpu]: pass " <<  passId << "/" << CAM_PASSES_NUM << "\r";
      std::cout.flush();
  }
  auto stop = std::chrono::high_resolution_clock::now();
  std::cout << std::endl;
  std::cout.flush();

  std::cout << std::fixed << std::setprecision(2) \
            << "[main_cam_gpu]: render time = " << std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000000.f << " s" << std::endl;
  std::cout << "[main_cam_gpu]: save image to hard ... " << std::endl;

  std::vector<float> realColor(WIN_WIDTH*WIN_HEIGHT*4);            
  ctx.pCopyHelper->ReadBuffer(frameBuferGPU, 0, realColor.data(), WIN_WIDTH*WIN_HEIGHT*4*sizeof(float));

  vkFreeMemory(ctx.device, memObject, nullptr);


  if(saveHDR) 
    SaveImage4fToEXR(realColor.data(), WIN_WIDTH, WIN_HEIGHT, imageOut.c_str(), normConst, true);
  else
    SaveImage4fToBMP(realColor.data(), WIN_WIDTH, WIN_HEIGHT, 4, imageOut.c_str(), normConst, gamma);
  
  return 0;
}
