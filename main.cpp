#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Image2d.h"
#include "Renderer/eye_ray.h"
#include "tests/tests.h"
#include "utils/mesh.h"

using LiteImage::Image2D;

extern double   g_buildTime; // used to get build time from deep underground of code
extern uint64_t g_buildTris; // used to get total tris processed by builder

constexpr bool MEASURE_FRAMES = false;

int main(int argc, const char** argv)
{
  if (argc > 1)
  {
    if (std::string(argv[1]) == "-tests_litert")
    {
      perform_tests_litert({});
      return 0;
    }
    else if (std::string(argv[1]) == "-intersection_benchmark")
    {
      benchmark_framed_octree_intersection();
      return 0;
    }
    else if (std::string(argv[1]) == "-benchmark" && argc > 2)
    {
      std::string mesh_name = argv[2];
      std::string supported_type = argc == 3  ? "" : argv[3];
      unsigned flags = BENCHMARK_FLAG_RENDER_RT;
      if (supported_type == "build")
      {
        flags = BENCHMARK_FLAG_BUILD;
        supported_type = "";
      }
      main_benchmark("saves/"+mesh_name, mesh_name, flags, supported_type);

      return 0;
    }
    else if (std::string(argv[1]) == "-sbs_benchmark" && argc > 2)
    {
      std::string mesh_name = argv[2];
      unsigned flags = BENCHMARK_FLAG_RENDER_RT;
      SBS_benchmark("saves/"+mesh_name, mesh_name, flags);
      return 0;
    }
    else if (std::string(argv[1]) == "-rtx_benchmark" && argc > 2)
    {
      std::string mesh_name = argv[2];
      std::string supported_type = argc == 3  ? "" : argv[3];
      unsigned flags = BENCHMARK_FLAG_RENDER_RT;
      if (supported_type == "build")
      {
        flags = BENCHMARK_FLAG_BUILD;
        supported_type = "";
      }
      rtx_benchmark("saves/"+mesh_name, mesh_name, flags, supported_type);

      return 0;
    }
  }
  //auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  //cmesh4::create_triangle_list_grid(mesh, LiteMath::uint3(32,32,32));
  //return 0;
  // perform_tests_litert({9});
  //benchmark_framed_octree_intersection();
  // return 0;

  uint32_t WIDTH  = 800;
  uint32_t HEIGHT = 800;
  

  std::string scenePath   = "scenes/03_gs_scenes/" + std::string(argv[1]) + "_" + std::string(argv[2]) + ".xml";
  const char* accelStruct  = "BVH2Common";
  const char* buildFormat  = "cbvh_embree2";
  const char* layout       = "SuperTreeletAlignedMerged4"; ///"opt";

  std::string outImageFile = std::string(argv[1]) + "_" + std::string(argv[2]) + ".png";
  int NUM_LAUNCHES = 1;

  Image2D<uint32_t> image(WIDTH, HEIGHT);
  std::shared_ptr<IRenderer> pRender = nullptr;
  std::cout << "[main]: init renderer ..." << std::endl; 
  {
    pRender = CreateMultiRenderer("GPU");  
    auto accelStructImpl = CreateSceneRT(accelStruct, buildFormat, layout);
    pRender->SetAccelStruct(accelStructImpl);
  }

  pRender->SetViewport(0,0,WIDTH,HEIGHT);
  
  g_buildTime = 0.0;
  bool loaded = false; 
  std::cout << "[main]: load scene '" << scenePath << "'" << std::endl;
  loaded = pRender->LoadScene(scenePath.c_str());

  std::cout << "Implementation:  " << pRender->Name() << "; builder = '" << buildFormat << "'" << std::endl;

  MultiRenderPreset preset;
  preset.render_mode = MULTI_RENDER_MODE_GS;
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
  LiteImage::SaveImage(outImageFile.c_str(), image);
  
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
