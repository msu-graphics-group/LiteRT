#include <iostream>
#include <fstream>
#include <filesystem>
#include <memory>
#include <iomanip>

#include "integrator_pt.h"
#include "diff_render/integrator_dr.h"
#include "ArgParser.h"

#include "adam.h"

void SaveFrameBufferToEXR(float* data, int width, int height, int channels, const char* outfilename, float a_normConst = 1.0f);
bool SaveImage4fToBMP(const float* rgb, int width, int height, int channels, const char* outfilename, float a_normConst, float a_gamma);
std::vector<float> LoadImage4fFromEXR(const char* infilename, int* pW, int* pH);

float4x4 ReadMatrixFromString(const std::string& str);

void Image2D4fRegularizer(int w, int h, const float* data, float* grad);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, const char** argv)
{
  int FB_WIDTH        = 1024;
  int FB_HEIGHT       = 1024;
  int FB_CHANNELS     = 4;

  int PASS_NUMBER     = 1024;

  std::string scenePath      = "../resources/HydraCore/hydra_app/tests/test_42/statex_00001.xml"; 
  std::string sceneDir       = "";          // alternative path of scene library root folder (by default it is the folder where scene xml is located)
  std::string imageOut       = "z_out.bmp";
  std::string integratorType = "mispt";
  std::string refImgpath     = "z_ref.exr";
  float gamma                = 2.4f; // out gamma, special value, see save image functions

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  ArgParser args(argc, argv);
  
  if(args.hasOption("-in"))
    scenePath = args.getOptionValue<std::string>("-in");

  if(args.hasOption("-out"))
    imageOut = args.getOptionValue<std::string>("-out");

  if(args.hasOption("-ref"))
    refImgpath = args.getOptionValue<std::string>("-ref");

  std::filesystem::path out_path {imageOut};
  auto dir = out_path.parent_path();
  if(!dir.empty() && !std::filesystem::exists(dir))
    std::filesystem::create_directories(dir);
 
  if(args.hasOption("-scn_dir"))
    sceneDir = args.getOptionValue<std::string>("-scn_dir");

  const bool saveHDR = imageOut.find(".exr") != std::string::npos || 
                       imageOut.find(".image1f") != std::string::npos || 
                       imageOut.find(".image4f") != std::string::npos || 
                       imageOut.find(".image3d1f") != std::string::npos;
                       
  const std::string imageOutClean = imageOut.substr(0, imageOut.find_last_of("."));

  if(args.hasOption("-integrator"))
    integratorType = args.getOptionValue<std::string>("-integrator");
  
  if(args.hasOption("-gamma")) {
    std::string gammaText = args.getOptionValue<std::string>("-gamma");
    if(gammaText == "srgb" || gammaText == "sSRGB")
      gamma = 2.4f;
    else
      gamma = args.getOptionValue<float>("-gamma");
  }
  
  int spectral_mode = args.hasOption("--spectral") ? 1 : 0;

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
  
  int gradMode = 1;
  if(args.hasOption("-grad"))
    gradMode = args.getOptionValue<int>("-grad");

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////
  std::cout << "[drmain]: loading xml ... " << scenePath.c_str() << std::endl;
  
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
  auto pImpl = std::make_shared<IntegratorDR>(FB_WIDTH*FB_HEIGHT, spectral_mode, gradMode, features);
  
  std::cout << "[drmain]: Loading reference image ... " << refImgpath.c_str() << std::endl;

  int refW = 0, refH = 0;
  std::vector<float> refColor = LoadImage4fFromEXR(refImgpath.c_str(), &refW, &refH);
  
  if(refW  == 0 || refH == 0)
  {
    std::cout << "[drmain]: can't load reference image '" << refImgpath.c_str() << "'" << std::endl;
    exit(0);
  }
  else if(refW != FB_WIDTH || refH != FB_HEIGHT)
  {
    std::cout << "[drmain]: bad resolution of reference image, must   be " << FB_WIDTH << ", " << FB_HEIGHT << std::endl;
    std::cout << "[drmain]: bad resolution of reference image, actual is " << refW << ", " << refH << std::endl;
    exit(0);
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////

  pImpl->SetViewport(0,0,FB_WIDTH,FB_HEIGHT);
  std::cout << "[drmain]: Loading scene ... " << scenePath.c_str() << std::endl;
  pImpl->LoadScene(scenePath.c_str(), sceneDir.c_str());

  if(override_camera_pos)
  {
    pImpl->SetWorldView(look_at);
  }

  pImpl->CommitDeviceData();

  PASS_NUMBER = pImpl->GetSPP();                     // read target spp from scene
  if(args.hasOption("-spp"))                         // override it if spp is specified via command line
    PASS_NUMBER = args.getOptionValue<int>("-spp");

  // remember (x,y) coords for each thread to make our threading 1D
  //
  std::cout << "[drmain]: PackXYBlock() ... " << std::endl; 
  pImpl->PackXYBlock(FB_WIDTH, FB_HEIGHT, 1);

  pImpl->SetIntegratorType(Integrator::INTEGRATOR_MIS_PT);
  pImpl->UpdateMembersPlainData();

  float timings[4] = {0,0,0,0};
  float normConst = 1.0f/float(PASS_NUMBER);

  int wh[3] = {256,256,4};  

  std::vector<float> imgData(wh[0]*wh[1]*wh[2]); 
  std::vector<float> imgGrad(wh[0]*wh[1]*wh[2]); 

  std::cout << "(w,h) = " << wh[0] << ", " << wh[1] << std::endl;
  ///////////////////////////////////////////////////////////////////////////////// 

  std::fill(imgData.begin(), imgData.end(), 1.0f); // 
  std::fill(imgGrad.begin(), imgGrad.end(), 0.0f);

  auto [texOffset, texSize] = pImpl->PutDiffTex2D(1, wh[0], wh[1], wh[2]);

  pImpl->SetMaxThreadsAndBounces(32, 6);

  std::shared_ptr< IGradientOptimizer<float> > pOpt = std::make_shared< AdamOptimizer<float> >(imgGrad.size());

  // now run opt loop
  //
  for(int iter = 0; iter < 50; iter++) 
  {
    const int eachTen        = iter/20 + 1;
    const int currPassNumber = PASS_NUMBER; //std::min(PASS_NUMBER*eachTen, 64);
    normConst                = 1.0f/float(currPassNumber);

    std::cout << "[drmain]: Render(" << std::setfill('0') << std::setw(2) << iter << ", spp = " << currPassNumber << ") ..";
    std::cout.flush();
    
    std::fill(realColor.begin(), realColor.end(), 0.0f);

    //float loss = pImpl->RayTraceDR(FB_WIDTH*FB_HEIGHT, FB_CHANNELS, realColor.data(), 1,
    //                                refColor.data(), imgData.data(), imgGrad.data(), imgGrad.size());

    float loss = pImpl->PathTraceDR(FB_WIDTH*FB_HEIGHT, FB_CHANNELS, realColor.data(), currPassNumber,
                                    refColor.data(), imgData.data(), imgGrad.data(), imgGrad.size());                                
    //break;
    
    //std::vector<float> gradTest(imgGrad.size()); 
    //std::fill(gradTest.begin(), gradTest.end(), 0.0f);
    //Image2D4fRegularizer(wh[0], wh[1], imgData.data(), gradTest.data());
    //for(size_t i=0;i<imgGrad.size();i++)
    //  imgGrad[i] += gradTest[i]*0.5f;

    //if(iter == 30) 
    //{
    //  Image2D4fRegularizer(wh[0], wh[1], imgData.data(), gradTest.data());
    //  std::ofstream fout("grad_30.txt");
    //  for(int y=0;y<wh[1];y++) {
    //    for(int x=0;x<wh[0];x++) 
    //      fout << std::setfill('0') << std::setw(5) << gradTest[y*wh[0]+x] << " ";
    //    fout << std::endl;
    //  }
    //
    //  std::ofstream fout2("grad_30_rt.txt");
    //  for(int y=0;y<wh[1];y++) {
    //    for(int x=0;x<wh[0];x++) 
    //      fout2 << std::setfill('0') << std::setw(5) << imgGrad[y*wh[0]+x] << " ";
    //    fout2 << std::endl;
    //  }
    //}

    pImpl->GetExecutionTime("PathTraceDR", timings);  
  
    std::cout << ", loss = " << loss << ", time = " << timings[0] << " ms" << std::endl;
    std::cout.flush();
    
    //std::cout << "PathTraceBlock(exec) = " << timings[0] << " ms " << std::endl;
    
    pOpt->step(imgData.data(), imgGrad.data(), iter);

    // may apply some filters here (for example median filter)
    
    if(gradMode == 1)
    {
      std::stringstream strOut;
      strOut << imageOutClean << std::setfill('0') << std::setw(2) << iter << ".bmp";
      auto outName = strOut.str();
      SaveImage4fToBMP(realColor.data(), FB_WIDTH, FB_HEIGHT, 4, outName.c_str(), normConst, 2.4f);
    }
    else if(gradMode == 0)
    {
      const std::string outName = imageOutClean + ".exr";
      SaveFrameBufferToEXR(realColor.data(), FB_WIDTH, FB_HEIGHT, FB_CHANNELS, outName.c_str(), normConst);
      break;
    }
  }

  SaveImage4fToBMP(imgData.data(), 256, 256, 4, "z_opt_tex.bmp", 1.0f, 2.4f);

  return 0;
}
