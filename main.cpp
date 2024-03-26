#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Image2d.h"
#include "Renderer/eye_ray.h"

using LiteImage::Image2D;

double   g_buildTime = 0; // used to get build time from deep underground of code
uint64_t g_buildTris = 0; // used to get total tris processed by builder

constexpr bool MEASURE_FRAMES = false;

IRenderer* MakeEyeRayShooterRenderer(const char* a_name);

ISceneObject* MakeBruteForceRT(const char* a_implName);
ISceneObject* MakeBVH2CommonRT(const char* a_implName, const char* a_buildName, const char* a_layoutName);

ISceneObject* CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName)
{
  const std::string className(a_implName); //
  if (className.find("BruteForce") != std::string::npos)
    return MakeBruteForceRT(a_implName);
  else if (className.find("BVH2Common") != std::string::npos || className.find("BVH2") != std::string::npos)
    return MakeBVH2CommonRT(a_implName, a_buildName, a_layoutName);
  return nullptr;
}

int main(int argc, const char** argv)
{
  uint32_t WIDTH  = 1024;
  uint32_t HEIGHT = 1024;
  
  const char* scenePath   = "scenes/02_sdf_scenes/csg_1.xml"; // bunny_cornell.xml, instanced_objects.xml
  const char* meshPath    = "scenes/meshes/bunny.vsgf";///helix_mid.vsgf";
  const char* renderName  = "eye"; // "RT", "RTAO" or just "AO"

  const char* accelStruct  = "BVH2Common"; // BruteForce BVH2Common
  const char* buildFormat  = "cbvh_embree2";///"NanoRT";  // BVH2Common
  const char* layout       = "SuperTreeletAlignedMerged4"; ///"opt";

  const char* outImageFile = "z_out.bmp";
  const char* outStatsFile = "z_stats.csv";
  
  bool onGPU          = false;
  bool testMode       = false;
  int  implId         = 0;
  int  NFrames        = 100;
  // how many times should we run
  //
  int NUM_LAUNCHES = 1;

  RenderPreset presets {};
  presets.isAORadiusInMeters = false;
  presets.aoRaysNum       = 16;
  presets.aoRayLength     = 0.1f; //
  presets.measureOverhead = false;
  presets.numBounces      = 3;

  //you may overrride some input parameters
  if(argc > 2)
  {
    for(int i=1;i<argc;i++) 
    {
      std::string currArg(argv[i]);
      if(currArg == "-scene" && i+1 < argc)
        scenePath = argv[i+1];
      else if(currArg == "-render" && i+1 < argc) 
        renderName = argv[i+1];
      else if(currArg == "-accel" && i+1 < argc) 
        accelStruct = argv[i+1];
      else if(currArg == "-build" && i+1 < argc) 
        buildFormat = argv[i+1];
      else if(currArg == "-out" && i+1 < argc) 
        outImageFile = argv[i+1];
      else if(currArg == "-layout" && i+1 < argc) 
        layout = argv[i+1];
      else if(currArg == "-stats" && i+1 < argc) 
        outStatsFile = argv[i+1];
      else if(currArg == "-width" && i+1 < argc)
        WIDTH = std::atoi(argv[i+1]);
      else if(currArg == "-height" && i+1 < argc)
        HEIGHT = std::atoi(argv[i+1]);
      else if((currArg == "-run_num" || currArg == "-num_launches")  && i+1 < argc)
        NUM_LAUNCHES = std::atoi(argv[i+1]);
      else if(currArg == "-aoraynum" && i+1 < argc)
        presets.aoRaysNum = std::atoi(argv[i+1]);
      else if(currArg == "-aoraylen" && i+1 < argc)
        presets.aoRayLength = std::atof(argv[i+1]);
      else if(currArg == "-ao_radius_in_meters")
        presets.isAORadiusInMeters = true;
      else if(currArg == "-gpu" && i+1 < argc)
        onGPU = (std::atoi(argv[i+1]) != 0);
      else if(currArg == "-test" && i+1 < argc)
        testMode = (std::atoi(argv[i+1]) != 0);
      else if(currArg == "-launch_id" && i+1 < argc)
        implId = std::atoi(argv[i+1]);
      else if(currArg == "-frames" && i+1 < argc)
        NFrames = std::atoi(argv[i+1]);
    }
  }

  if(std::string(accelStruct) == "overhead" || std::string(accelStruct) == "Overhead") //
  {
    accelStruct = "BVH2Fat32";
    presets.measureOverhead = true;
  }

  Image2D<uint32_t> image(WIDTH, HEIGHT);
  std::shared_ptr<IRenderer> pRender = nullptr;

  std::cout << "[main]: init renderer ..." << std::endl; 
  {
    pRender = std::shared_ptr<IRenderer>(MakeEyeRayShooterRenderer(renderName));  
    auto accelStructImpl = std::shared_ptr<ISceneObject>(CreateSceneRT(accelStruct, buildFormat, layout));
    pRender->SetAccelStruct(accelStructImpl);
  }

  pRender->SetViewport(0,0,WIDTH,HEIGHT);
  pRender->SetPresets(presets);
  
  g_buildTime = 0.0;
  bool loaded = false; 
  std::cout << "[main]: load scene '" << scenePath << "'" << std::endl;
  loaded = pRender->LoadScene(scenePath);

  std::cout << "Implementation:  " << pRender->Name() << "; builder = '" << buildFormat << "'" << std::endl;

  pRender->CommitDeviceData();
  pRender->Clear(WIDTH, HEIGHT, "color");

  if(!loaded) 
  {
    std::cout << "can't load scene '" << scenePath << "'" << std::endl; 
    return -1;
  }
  
  // you may update presents if you want, but don't forget to call 'UpdateMembersPlainData' after that
  //pImpl->SetPrestes(...);
  //pImpl->UpdateMembersPlainData();
  
  float timings[4] = {0,0,0,0}; 
  
  LiteImage::Image2D<uint32_t> imgRef;
  float psnr = 100.0f;
  if(implId != 0)                      // load reference image if we are not in reference impl.
  {
    std::string pathRef = std::string(outImageFile);
                pathRef = pathRef.substr(0, pathRef.size() - 6) + "00.bmp";
    imgRef = LiteImage::LoadImage<uint32_t>(pathRef.c_str());
  } 

  float timeAvg = 0.0f;
  float timeMin = 100000000000000.0f;
  
  std::vector<float> allTimes;
  const char* raysType = "eye";
  if(presets.aoRayLength >= 0.5f)
    raysType = "diffuse";
  else if(presets.aoRayLength >= 0.1f)
    raysType = "ao_long";
  else
    raysType = "ao_short";

  const int NUM_ITERS  = MEASURE_FRAMES ? 1 : NUM_LAUNCHES;
  const int NUM_FRAMES = MEASURE_FRAMES ? NUM_LAUNCHES : 1;
  
  for(int launchId = 0; launchId < NUM_ITERS; launchId++) 
  {
    std::cout << "[main]: do rendering ..." << std::endl;
    pRender->Render(image.data(), WIDTH, HEIGHT, "color", NUM_FRAMES); 
    std::cout << std::endl;

    if(implId != 0 && testMode) // estimate PSNR
    {
      auto mseVal = LiteImage::MSE(imgRef, image);
      psnr        = 20.0f * std::log10(255.0f / std::sqrt(mseVal));
    } 

    if(std::string(renderName) == "ao" || std::string(renderName) == "AO" || std::string(renderName) == "rtao" || std::string(renderName) == "RTAO") 
    {
      pRender->GetExecutionTime("CalcAOBlock", timings);
      std::cout << "CalcAOBlock(exec) = " << timings[0] << " ms; (runId = " << launchId << ", PSNR = " << psnr << ")" << std::endl;
    }
    else if(std::string(renderName) == "bfrt" || std::string(renderName) == "BFRT" || 
            std::string(renderName) == "bfrt32" || std::string(renderName) == "BFRT32")
    {
      pRender->GetExecutionTime("BFRT_ReadAndCompute32", timings);
      std::cout << "BFRT_ReadAndCompute32(exec) = " << timings[0] << " ms; (runId = " << launchId << ", PSNR = " << psnr << ")" << std::endl;
    }
    else if(std::string(renderName) == "bfrt16" || std::string(renderName) == "BFRT16")
    {
      pRender->GetExecutionTime("BFRT_ReadAndCompute16", timings);
      std::cout << "BFRT_ReadAndCompute16(exec) = " << timings[0] << " ms; (runId = " << launchId << ", PSNR = " << psnr << ")" << std::endl;
    }
    else if(std::string(renderName) == "BFRT_Compute" || std::string(renderName) == "BFRT_ComputeOnly")
    {
      pRender->GetExecutionTime("BFRT_ComputeBlock", timings);
      std::cout << "BFRT_ComputeBlock(exec) = " << timings[0] << " ms; (runId = " << launchId << ", PSNR = " << psnr << ")" << std::endl;
    }
    else
    {
      pRender->GetExecutionTime("CastRaySingleBlock", timings);
      std::cout << "CastRaySingleBlock(exec) = " << timings[0] << " ms; (runId = " << launchId << ", PSNR = " << psnr << ")" << std::endl;
    }
    
    allTimes.push_back(timings[0]); 
    timeAvg += timings[0];
    timeMin = std::min(timeMin, timings[0]);
  }
  
  timeAvg /= float(NUM_LAUNCHES);
  if(MEASURE_FRAMES)
    timeMin = timeAvg;

  std::cout << "[main]: save image to file ..." << std::endl;
  LiteImage::SaveImage(outImageFile, image);
  
  float summDiffSquare = 0.0f;
  for(float time : allTimes)
    summDiffSquare += (time - timeAvg)*(time - timeAvg);
  float timeErr = std::sqrt(summDiffSquare/float(allTimes.size()-1));
  if(MEASURE_FRAMES)
    timeErr = 0.0f;

  // save stats to file
  //
  const char* sceneName = "unknown";
  std::string scenePathStr = std::string(scenePath);
  auto posSlash = scenePathStr.find_last_of('\\');
  if (posSlash == std::string::npos)
    posSlash = scenePathStr.find_last_of('/');
  auto posDot = scenePathStr.find_last_of('.');
  std::string scenePathStr2 = scenePathStr.substr(posSlash + 1, posDot - posSlash - 1);
  sceneName = scenePathStr2.c_str();

  const char* timeFileName = "z_timings.csv";
  std::ofstream fout;
  std::ifstream fin(timeFileName);
  if (!fin.is_open())
  {
    fout.open(timeFileName);
    fout << "Render;Resolution;Scene;TrisNum(build);Build;Accel;Layout;TimeMin(ms);TimeAvg(ms);TimeErr(ms);TimeBuild(ms);MEM(MB);PSNR;Test;" << std::endl;
  }
  else
  {
    fin.close();
    fout.open(timeFileName, std::ios::app);
  }

  std::string acStr = std::string(accelStruct);
  if(onGPU)
    acStr += "(GPU)";

  std::string tstMsg = (psnr <= 40.0f) ? "FAILED" : "PASSED";
  fout << std::setprecision(4) << renderName << ";" << WIDTH << ";" << sceneName << ";" << g_buildTris << ";" << buildFormat << ";" << acStr.c_str() << ";" << layout << ";" << timeMin << ";" << timeAvg << ";" << timeErr << ";" << float(g_buildTime) << ";" << psnr << ";" << tstMsg.c_str() << ";" << std::endl;

  return 0;
}
