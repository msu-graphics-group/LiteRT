#include <cfloat>
#include <cstring>
#include <sstream>

#include "eye_ray.h"
#include "hydraxml.h"
#include "cmesh4.h"
#include "Timer.h"

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;

using LiteMath::perspectiveMatrix;
using LiteMath::lookAt;
using LiteMath::inverse4x4;

EyeRayCaster::EyeRayCaster() 
{ 
  m_pAccelStruct = nullptr;
}

void EyeRayCaster::SetViewport(int a_xStart, int a_yStart, int a_width, int a_height)
{
  m_width  = a_width;
  m_height = a_height;
  m_packedXY.resize(m_width*m_height);
}

bool EyeRayCaster::LoadScene(const char* a_scenePath)
{
  m_pAccelStruct->ClearGeom();
  return LoadSceneHydra(std::string(a_scenePath));
}

bool EyeRayCaster::LoadSceneHydra(const std::string& a_path)
{
  hydra_xml::HydraScene scene;
  if(scene.LoadState(a_path) < 0)
    return false;

  for(auto cam : scene.Cameras())
  {
    float aspect   = float(m_width) / float(m_height);
    auto proj      = perspectiveMatrix(cam.fov, aspect, cam.nearPlane, cam.farPlane);
    auto worldView = lookAt(float3(cam.pos), float3(cam.lookAt), float3(cam.up));

    m_projInv      = inverse4x4(proj);
    m_worldViewInv = inverse4x4(worldView);

    break; // take first cam
  }

  std::vector<uint64_t> trisPerObject;
  trisPerObject.reserve(1000);
  m_totalTris = 0;
  m_pAccelStruct->ClearGeom();
  auto mIter = scene.GeomNodes().begin();
  size_t pos = a_path.find_last_of('/');
  std::string root_dir = a_path.substr(0, pos);
  while (mIter != scene.GeomNodes().end())
  {
    std::string dir = root_dir + "/" + hydra_xml::ws2s(std::wstring(mIter->attribute(L"loc").as_string()));
    std::string name = hydra_xml::ws2s(std::wstring(mIter->name()));
    if (name == "mesh")
    {
      std::cout << "[LoadScene]: mesh = " << dir.c_str() << std::endl;
      auto currMesh = cmesh4::LoadMeshFromVSGF(dir.c_str());
      m_pAccelStruct->AddGeom_Triangles3f((const float*)currMesh.vPos4f.data(), currMesh.vPos4f.size(),
                                          currMesh.indices.data(), currMesh.indices.size(), BUILD_HIGH, sizeof(float)*4);
      m_totalTris += currMesh.indices.size()/3;
      trisPerObject.push_back(currMesh.indices.size()/3);
    }
    else if (name == "sdf")
    {
      std::cout << "[LoadScene]: sdf = " << dir.c_str() << std::endl;
      SdfScene scene;
      load_sdf_scene(scene, dir);
      m_pAccelStruct->AddGeom_Sdf(scene);
    }
    else
    {
      std::cout << "[LoadScene]: unknown geometry node type: " << name.c_str() << std::endl;
    }
    mIter++;
  }
  
  m_totalTrisVisiable = 0;
  m_pAccelStruct->ClearScene();
  for(auto inst : scene.InstancesGeom())
  {
    m_pAccelStruct->AddInstance(inst.geomId, inst.matrix);
    m_totalTrisVisiable += trisPerObject[inst.geomId];
  }
  m_pAccelStruct->CommitScene();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void EyeRayCaster::Render(uint32_t* a_outColor, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum)
{
  CastRaySingleBlock(a_width*a_height, a_outColor, a_passNum);
}

void EyeRayCaster::CastRaySingleBlock(uint32_t tidX, uint32_t * out_color, uint32_t a_numPasses)
{
  profiling::Timer timer;
  
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int i=0;i<tidX;i++)
    CastRaySingle(i, out_color);

  timeDataByName["CastRaySingleBlock"] = timer.getElapsedTime().asMilliseconds();
}

const char* EyeRayCaster::Name() const
{
  std::stringstream strout;
  strout << "EyeRayCaster(" << m_pAccelStruct->Name() << ")";
  m_tempName = strout.str();
  return m_tempName.c_str();
}

void EyeRayCaster::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  auto p = timeDataByName.find(a_funcName);
  if(p == timeDataByName.end())
    return;
  a_out[0] = p->second;
}


IRenderer* MakeEyeRayShooterRenderer(const char* a_name) { return new EyeRayCaster; }

