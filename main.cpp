#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Image2d.h"
#include "Renderer/eye_ray.h"

using LiteImage::Image2D;

extern double   g_buildTime; // used to get build time from deep underground of code
extern uint64_t g_buildTris; // used to get total tris processed by builder

constexpr bool MEASURE_FRAMES = false;

int main(int argc, const char** argv)
{
  uint32_t WIDTH  = 1024;
  uint32_t HEIGHT = 1024;
  
<<<<<<< HEAD
  const char* scenePath   = "scenes/01_simple_scenes/bunny_plane.xml"; // 02_sdf_scenes/csg_new.xml bunny_cornell.xml, instanced_objects.xml
=======
  const char* scenePath   = "scenes/02_sdf_scenes/relu_fields.xml"; // 02_sdf_scenes/csg_new.xml bunny_cornell.xml, instanced_objects.xml
>>>>>>> 83386d6 (Almost done)
  const char* accelStruct  = "BVH2Common"; // BruteForce BVH2Common
  const char* buildFormat  = "cbvh_embree2";///"NanoRT";  // BVH2Common
  const char* layout       = "SuperTreeletAlignedMerged4"; ///"opt";

  const char* outImageFile = "z_out.bmp";
  int NUM_LAUNCHES = 1;

  Image2D<uint32_t> image(WIDTH, HEIGHT);
  std::shared_ptr<IRenderer> pRender = nullptr;
  std::cout << "[main]: init renderer ..." << std::endl; 
  {
    pRender = CreateMultiRenderer("CPU");  
    auto accelStructImpl = CreateSceneRT(accelStruct, buildFormat, layout);
    pRender->SetAccelStruct(accelStructImpl);
  }

  pRender->SetViewport(0,0,WIDTH,HEIGHT);
  
  g_buildTime = 0.0;
  bool loaded = false; 
  std::cout << "[main]: load scene '" << scenePath << "'" << std::endl;
  loaded = pRender->LoadScene(scenePath);

  std::cout << "Implementation:  " << pRender->Name() << "; builder = '" << buildFormat << "'" << std::endl;

  MultiRenderPreset preset;
  preset.mode = MULTI_RENDER_MODE_RF;
  dynamic_cast<MultiRenderer*>(pRender.get())->SetPreset(preset);
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
  float timeAvg = 0.0f;
  float timeMin = 100000000000000.0f;
  std::vector<float> allTimes;

  const int NUM_ITERS  = MEASURE_FRAMES ? 1 : NUM_LAUNCHES;
  const int NUM_FRAMES = MEASURE_FRAMES ? NUM_LAUNCHES : 1;
  
  for(int launchId = 0; launchId < NUM_ITERS; launchId++) 
  {
    std::cout << "[main]: do rendering ..." << std::endl;
    pRender->Render(image.data(), WIDTH, HEIGHT, "color", NUM_FRAMES); 
    std::cout << std::endl;

    pRender->GetExecutionTime("CastRaySingleBlock", timings);
    std::cout << "CastRaySingleBlock(exec) = " << timings[0] << " ms" << std::endl;
    
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

  return 0;
}
