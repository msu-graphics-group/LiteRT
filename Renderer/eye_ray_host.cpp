#include <cfloat>
#include <cstring>
#include <sstream>
#include <fstream>

#include "eye_ray.h"
#include "LiteScene/hydraxml.h"
#include "LiteScene/cmesh4.h"
#include "../Timer.h"
#include "../utils/mesh.h"
#include "../utils/mesh_bvh.h"
#include "../utils/sdf_converter.h"
#include "harmonic_function/any_polygon_common.h"

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;

using LiteMath::perspectiveMatrix;
using LiteMath::lookAt;
using LiteMath::inverse4x4;

MultiRenderer::MultiRenderer(uint32_t maxPrimitives) 
{ 
  m_maxPrimitives = maxPrimitives;
  SetAccelStruct(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
  m_preset = getDefaultPreset();

  m_textures.resize(MULTI_RENDER_MAX_TEXTURES);

  LiteImage::Image2D<float4> texture = LiteImage::Image2D<float4>(16, 16, float4(0,1,1,1)); //LiteImage::LoadImage<float4>("scenes/porcelain.png");
  for (int i=0;i<MULTI_RENDER_MAX_TEXTURES;i++)
    AddTexture(texture);
  AddMaterial({MULTI_RENDER_MATERIAL_TYPE_TEXTURED, DEFAULT_TEXTURE});
  active_textures_count = 1;

  m_matIdbyPrimId.push_back(DEFAULT_MATERIAL);

  m_seed = rand();

  m_lights = {create_direct_light(float3(1,1,1), float3(2.0f/3.0f)), create_ambient_light(float3(0.25, 0.25, 0.25))};
}

void MultiRenderer::SetViewport(int a_xStart, int a_yStart, int a_width, int a_height)
{
  m_width  = a_width;
  m_height = a_height;
  m_packedXY.resize(m_width*m_height);
}

bool MultiRenderer::LoadScene(const char* a_scenePath)
{
  m_seed = rand();
  m_pAccelStruct->ClearGeom();
  return LoadSceneHydra(std::string(a_scenePath));
}

void MultiRenderer::UpdateCamera(const LiteMath::float4x4& worldView, const LiteMath::float4x4& proj)
{
  m_proj = proj;
  m_worldView = worldView;
  m_projInv      = inverse4x4(proj);
  m_worldViewInv = inverse4x4(worldView);
}

bool MultiRenderer::LoadSceneHydra(const std::string& a_path)
{
  hydra_xml::HydraScene scene;
  if(scene.LoadState(a_path) < 0)
    return false;

  for(auto cam : scene.Cameras())
  {
    float aspect   = float(m_width) / float(m_height);
    auto proj      = perspectiveMatrix(cam.fov, aspect, cam.nearPlane, cam.farPlane);
    auto worldView = lookAt(float3(cam.pos), float3(cam.lookAt), float3(cam.up));
    UpdateCamera(worldView, proj);
    break; // take first cam
  }

  m_pAccelStruct->ClearGeom();
  auto mIter = scene.GeomNodes().begin();
  size_t pos = a_path.find_last_of('/');
  std::string root_dir = a_path.substr(0, pos);

  std::vector<LiteMath::float4x4> addGeomTransform;
  std::vector<unsigned> isCustom;

  while (mIter != scene.GeomNodes().end())
  {
    std::string dir = root_dir + "/" + hydra_xml::ws2s(std::wstring(mIter->attribute(L"loc").as_string()));
    std::string name = hydra_xml::ws2s(std::wstring(mIter->name()));
    addGeomTransform.push_back(float4x4());
    isCustom.push_back(1);
    if (name == "mesh")
    {
      auto currMesh = cmesh4::LoadMeshFromVSGF(dir.c_str());
      float4x4 trans = cmesh4::normalize_mesh(currMesh);
      addGeomTransform.back() = inverse4x4(trans);
      isCustom.back() = 0; 

      unsigned geomId = m_pAccelStruct->AddGeom_Triangles3f((const float *)currMesh.vPos4f.data(), currMesh.vPos4f.size(),
                                                            currMesh.indices.data(), currMesh.indices.size(), BUILD_HIGH, sizeof(float) * 4);
      add_mesh_internal(currMesh, geomId);
    }
    else
    {
      m_pAccelStruct->AddCustomGeom_FromFile(name.c_str(), dir.c_str(), m_pAccelStruct.get());
    }
    mIter++;
  }
  
  m_pAccelStruct->ClearScene();
  for(auto inst : scene.InstancesGeom())
    AddInstance(inst.geomId + isCustom[inst.geomId] * CRT_GEOM_MASK_AABB_BIT, 
                inst.matrix*addGeomTransform[inst.geomId]);
  m_pAccelStruct->CommitScene();

  return true;
}

bool MultiRenderer::CreateSceneFromHydra(const std::string& a_path, unsigned type, SparseOctreeSettings so_settings)
{
  hydra_xml::HydraScene scene;
  if(scene.LoadState(a_path) < 0)
    return false;

  for(auto cam : scene.Cameras())
  {
    float aspect   = float(m_width) / float(m_height);
    auto proj      = perspectiveMatrix(cam.fov, aspect, cam.nearPlane, cam.farPlane);
    auto worldView = lookAt(float3(cam.pos), float3(cam.lookAt), float3(cam.up));
    UpdateCamera(worldView, proj);
    break; // take first cam
  }

  m_pAccelStruct->ClearGeom();
  auto mIter = scene.GeomNodes().begin();
  size_t pos = a_path.find_last_of('/');
  std::string root_dir = a_path.substr(0, pos);

  std::vector<LiteMath::float4x4> addGeomTransform;
  std::vector<unsigned> isCustom;

  while (mIter != scene.GeomNodes().end())
  {
    std::string dir = root_dir + "/" + hydra_xml::ws2s(std::wstring(mIter->attribute(L"loc").as_string()));
    std::string name = hydra_xml::ws2s(std::wstring(mIter->name()));
    addGeomTransform.push_back(float4x4());
    isCustom.push_back(1);
    if (name == "mesh")
    {
      auto currMesh = cmesh4::LoadMeshFromVSGF(dir.c_str());
      float4x4 trans = cmesh4::normalize_mesh(currMesh);
      addGeomTransform.back() = inverse4x4(trans);

      switch (type)
      {
      case TYPE_MESH_TRIANGLE:
      {
        unsigned geomId = m_pAccelStruct->AddGeom_Triangles3f((const float *)currMesh.vPos4f.data(), currMesh.vPos4f.size(),
                                                              currMesh.indices.data(), currMesh.indices.size(), BUILD_HIGH, sizeof(float) * 4);
        add_mesh_internal(currMesh, geomId);
        isCustom.back() = 0; 
      }
      break;
      case TYPE_SDF_SVS:
      {
        auto *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
        assert(bvhrt);
        std::vector<SdfSVSNode> svs_nodes = sdf_converter::create_sdf_SVS(so_settings, currMesh);
        bvhrt->AddGeom_SdfSVS(svs_nodes, m_pAccelStruct.get());
      }
      break;
      case TYPE_SDF_GRID:
      {
        auto *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
        assert(bvhrt);

        MeshBVH mesh_bvh;
        mesh_bvh.init(currMesh);
        unsigned sz = pow(2, so_settings.depth);
        std::vector<float> data(sz * sz * sz, 0);

        for (int i = 0; i < sz; i++)
        {
          for (int j = 0; j < sz; j++)
          {
            for (int k = 0; k < sz; k++)
            {
              data[i * sz * sz + j * sz + k] = mesh_bvh.get_signed_distance(2.0f * (float3(k + 0.5, j + 0.5, i + 0.5) / float(sz)) - 1.0f);
            }
          }
        }

        bvhrt->AddGeom_SdfGrid(SdfGridView(uint3(sz, sz, sz), data), m_pAccelStruct.get());
      }
      break;
      default:
        printf("cannot transform meshes from Hydra scene to type %u\n", type);
        break;
      }
    }
    else
    {
      printf("[CreateSceneFromHydra]: Only meshes are supported. If you want to render existing SDFs, use LoadSceneHydra instead\n");
    }
    mIter++;
  }
  
  m_pAccelStruct->ClearScene();
  for(auto inst : scene.InstancesGeom())
    AddInstance(inst.geomId + isCustom[inst.geomId] * CRT_GEOM_MASK_AABB_BIT, 
                inst.matrix*addGeomTransform[inst.geomId]);
  m_pAccelStruct->CommitScene();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void MultiRenderer::Render(uint32_t* a_outColor, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum)
{
  profiling::Timer timer;
  for (int i=0;i<a_passNum;i++)
    CastRaySingleBlock(a_width*a_height, a_outColor, a_passNum);
  timeDataByName["CastRaySingleBlock"] = timer.getElapsedTime().asMilliseconds();
}

void MultiRenderer::CastRaySingleBlock(uint32_t tidX, uint32_t * out_color, uint32_t a_numPasses)
{
  //CPU version is mostly used by debug, so better make it single-threaded
  //also per-pixel debug does not work with multithreading
  //#ifndef _DEBUG
  //#pragma omp parallel for default(shared)
  //#endif
  for(int i=0;i<tidX;i++)
    CastRaySingle(i, out_color);
}

void MultiRenderer::RenderFloat(float4* a_outColor, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum)
{
  profiling::Timer timer;
  for (int i=0;i<a_passNum;i++)
    CastRayFloatSingleBlock(a_width*a_height, a_outColor, a_passNum);
  timeDataByName["CastRayFloatSingleBlock"] = timer.getElapsedTime().asMilliseconds();
}

void MultiRenderer::CastRayFloatSingleBlock(uint32_t tidX, float4 * out_color, uint32_t a_numPasses)
{
  //CPU version is mostly used by debug, so better make it single-threaded
  //also per-pixel debug does not work with multithreading
  //#ifndef _DEBUG
  #pragma omp parallel for default(shared)
  //#endif
  for(int i=0;i<tidX;i++)
    CastRayFloatSingle(i, out_color);
}

const char* MultiRenderer::Name() const
{
  return m_pAccelStruct->Name();
}

void MultiRenderer::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  auto p = timeDataByName.find(a_funcName);
  if(p == timeDataByName.end())
    return;
  a_out[0] = p->second;
}

void MultiRenderer::SetScene(const cmesh4::SimpleMesh &scene)
{
  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  unsigned geomId = m_pAccelStruct->AddGeom_Triangles3f((const float*)scene.vPos4f.data(), scene.vPos4f.size(), scene.indices.data(), scene.indices.size(), BUILD_HIGH, sizeof(float)*4);                                                          
  add_mesh_internal(scene, geomId);
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(SdfGridView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfGrid\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfGrid(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(SdfFrameOctreeView scene)
{ 
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfFrameOctree\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfFrameOctree(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(SdfSVSView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfSVS\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfSVS(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}


void MultiRenderer::SetScene(SdfSBSView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfSBS\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfSBS(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(SdfSBSAdaptView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfSBSAdapt\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfSBSAdapt(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(SdfFrameOctreeTexView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports SdfFrameOctreeTex\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_SdfFrameOctreeTex(scene, m_pAccelStruct.get());
  add_SdfFrameOctreeTex_internal(scene, geomId);
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(const RawNURBS &nurbs)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports NURBS\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_NURBS(nurbs, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(GraphicsPrimView scene)
{
  BVHRT *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  if (!bvhrt)
  {
    printf("only BVHRT supports Graphics primitives\n");
    return;
  }

  SetPreset(m_preset);
  m_pAccelStruct->ClearGeom();
  auto geomId = bvhrt->AddGeom_GraphicsPrim(scene, m_pAccelStruct.get());
  m_pAccelStruct->ClearScene();
  AddInstance(geomId, LiteMath::float4x4());
  m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetScene(AnyPolygon const& poly) {
    namespace lm = LiteMath;

    auto const bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));

    if (nullptr == bvhrt) {
        fprintf(stderr, "only BVHRT supports AnyPolygons");
        return;
    }

    SetPreset(m_preset);
    m_pAccelStruct->ClearGeom();

    auto const geom_id = bvhrt->AddGeom_AnyPolygon(poly, m_pAccelStruct.get());

    m_pAccelStruct->ClearScene();
    AddInstance(geom_id, lm::float4x4{});
    m_pAccelStruct->CommitScene();
}

void MultiRenderer::SetPreset(const MultiRenderPreset& a_preset)
{
  m_preset = a_preset;
  auto *bvhrt = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));

  if (bvhrt)
    bvhrt->SetPreset(a_preset);
}

void MultiRenderer::Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, 
                           const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj,
                           MultiRenderPreset preset, int a_passNum)
{
  SetViewport(0,0, a_width, a_height);
  UpdateCamera(a_worldView, a_proj);
  SetPreset(preset);
  CommitDeviceData();
  Clear(a_width, a_height, "color");
  Render(imageData, a_width, a_height, "color", a_passNum); 
}

void MultiRenderer::RenderFloat(float4* imageData, uint32_t a_width, uint32_t a_height, 
                                const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj,
                                MultiRenderPreset preset, int a_passNum)
{
  SetViewport(0,0, a_width, a_height);
  UpdateCamera(a_worldView, a_proj);
  SetPreset(preset);
  CommitDeviceData();
  Clear(a_width, a_height, "color");
  RenderFloat(imageData, a_width, a_height, "color", a_passNum); 
}

void MultiRenderer::add_mesh_internal(const cmesh4::SimpleMesh &scene, uint32_t geomId)
{
#ifndef DISABLE_MESH_TEX
  m_geomOffsets.resize(geomId + 1, uint2(0,0));
  m_geomOffsets[geomId].x = m_indices.size();
  m_geomOffsets[geomId].y = m_vertices.size();
  m_indices.insert(m_indices.end(), scene.indices.begin(), scene.indices.end());

  unsigned sz = scene.vPos4f.size();
  m_vertices.resize(m_vertices.size() + sz);
  m_normals.resize(m_normals.size() + sz);

  for (unsigned i = 0; i < sz; ++i)
  {
    m_vertices[m_geomOffsets[geomId].y + i] = scene.vPos4f[i];
    m_vertices[m_geomOffsets[geomId].y + i].w = scene.vTexCoord2f[i].x;

    m_normals[m_geomOffsets[geomId].y + i] = scene.vNorm4f[i];
    m_normals[m_geomOffsets[geomId].y + i].w = scene.vTexCoord2f[i].y;
  }

  //add material if it was not explicitly set before
  if (geomId >= m_matIdOffsets.size())
  {
    m_matIdOffsets.resize(geomId + 1, uint2(0,1));
    
    //no material, set default
    if (scene.matIndices.empty())
    {
      m_matIdbyPrimId.push_back(DEFAULT_MATERIAL);
      m_matIdOffsets[geomId] = uint2(m_matIdbyPrimId.size()-1, 1);
    }
    else
    {
      m_matIdbyPrimId.insert(m_matIdbyPrimId.end(), scene.matIndices.begin(), scene.matIndices.end());
      m_matIdOffsets[geomId] = uint2(m_matIdbyPrimId.size()-scene.matIndices.size(), scene.matIndices.size());
    }
  }
#endif
}

void MultiRenderer::add_SdfFrameOctreeTex_internal(SdfFrameOctreeTexView scene, unsigned geomId)
{
  //add material if it was not explicitly set before
  if (geomId >= m_matIdOffsets.size())
  {
    m_matIdOffsets.resize(geomId + 1, uint2(0,1));
    unsigned start_idx = m_matIdbyPrimId.size();
    
    m_matIdbyPrimId.resize(start_idx + scene.size);
    for (unsigned i = 0; i < scene.size; ++i)
      m_matIdbyPrimId[start_idx + i] = scene.nodes[i].material_id;
    
    m_matIdOffsets[geomId] = uint2(start_idx, scene.size);
  }
}

uint32_t MultiRenderer::AddTexture(const Image2D<LiteMath::float4> &image)
{
  assert(active_textures_count < m_textures.size());
  std::shared_ptr<Image2D<LiteMath::float4>> pTexture1 = std::make_shared<Image2D<LiteMath::float4>>(image.width(), image.height(), image.data());
  Sampler sampler;
  sampler.filter   = Sampler::Filter::LINEAR; 
  sampler.addressU = Sampler::AddressMode::CLAMP;
  sampler.addressV = Sampler::AddressMode::CLAMP;
  m_textures[active_textures_count] = MakeCombinedTexture2D(pTexture1, sampler);
  active_textures_count++;

  return active_textures_count - 1;
}
uint32_t MultiRenderer::AddMaterial(const MultiRendererMaterial &material)
{
  m_materials.push_back(material);
  return m_materials.size() - 1;
}
void MultiRenderer::SetMaterial(uint32_t matId, uint32_t geomId)
{
  m_matIdbyPrimId.push_back(matId);
  if (geomId >= m_matIdOffsets.size())
    m_matIdOffsets.resize(geomId + 1, uint2(0,1));
  m_matIdOffsets[geomId] = uint2(m_matIdbyPrimId.size()-1, 1);
}

void MultiRenderer::SetLights(const std::vector<Light>& lights)
{
  m_lights = lights;
}

uint32_t MultiRenderer::AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix)
{
  m_instanceTransformInvTransposed.push_back(transpose(inverse4x4(a_matrix)));
  return m_pAccelStruct->AddInstance(a_geomId, a_matrix);
}

#if defined(USE_GPU)
  #if defined(USE_RTX)
    #include "eye_ray_rtx.h"
    #include "vk_context.h"
    std::shared_ptr<MultiRenderer> CreateMultiRenderer_RTX(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated);
    std::shared_ptr<MultiRenderer> CreateMultiRenderer(const char* a_name, uint32_t maxPrimitives) 
    {
      static vk_utils::VulkanContext context;
      if (std::string(a_name) == "RTX" || std::string(a_name) == "GPU")
      {
        if(context.instance == VK_NULL_HANDLE)
        {
          std::vector<const char*> requiredExtensions;
          auto deviceFeatures = MultiRenderer_RTX::ListRequiredDeviceFeatures(requiredExtensions);
          context = vk_utils::globalContextInit(requiredExtensions, true, 0, &deviceFeatures);
        }
        return CreateMultiRenderer_RTX(maxPrimitives, context, 256);
      }
      else
        return std::shared_ptr<MultiRenderer>(new MultiRenderer(maxPrimitives));
    }
  #else
    #include "eye_ray_gpu.h"
    #include "vk_context.h"
    std::shared_ptr<MultiRenderer> CreateMultiRenderer_GPU(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated);
    std::shared_ptr<MultiRenderer> CreateMultiRenderer(const char* a_name, uint32_t maxPrimitives) 
    {
      static vk_utils::VulkanContext context;

      if (std::string(a_name) == "RTX" || std::string(a_name) == "GPU")
      {
        if(context.instance == VK_NULL_HANDLE)
        {
          std::vector<const char*> requiredExtensions;
          auto deviceFeatures = MultiRenderer_GPU::ListRequiredDeviceFeatures(requiredExtensions);
          context = vk_utils::globalContextInit(requiredExtensions, true, 0, &deviceFeatures);
        }

        return CreateMultiRenderer_GPU(maxPrimitives, context, 256);
      }
      else
        return std::shared_ptr<MultiRenderer>(new MultiRenderer(maxPrimitives));
    }
  #endif
#else
std::shared_ptr<MultiRenderer> CreateMultiRenderer(const char* a_name, uint32_t maxPrimitives) 
{ 
  return std::shared_ptr<MultiRenderer>(new MultiRenderer(maxPrimitives)); 
}
#endif
