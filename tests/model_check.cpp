#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
#include "../utils/image_metrics.h"
#include "LiteScene/hydraxml.h"
#include "../utils/sparse_octree_builder.h"
#include "../utils/similarity_compression.h"
#include "LiteMath/Image2d.h"

#include <functional>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <map>
namespace cmesh4
{
  std::vector<std::string> explode(std::string aStr, char aDelim);
};
void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum);

void check_model(const std::string &path)
{
  printf("[check_model::INFO] Checking model %s\n", path.c_str());

  auto mesh = cmesh4::LoadMesh(path.c_str(), true, true);

  if (mesh.indices.size() == 0)
  {
    printf("[check_model::ERROR] Failed to load mesh %s\n", path.c_str());
    return;
  }

  printf("[check_model::INFO] Loaded mesh\n");

  std::string model_name = cmesh4::explode(cmesh4::explode(path, '/').back(), '.')[0];
  printf("[check_model::INFO] Model name: %s\n", model_name.c_str());

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.spp = 4;

  unsigned W = 4096, H = 4096;
  LiteImage::Image2D<uint32_t> image_ref(W, H);
  LiteImage::Image2D<uint32_t> image(W, H);

  {
    auto pRender = CreateMultiRenderer(DEVICE_GPU);
    preset.normal_mode = NORMAL_MODE_VERTEX;
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    //render(image, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza
    //render(image_ref, pRender, float3(-0.75,-0.75,1.25), float3(-0.75,-0.75,0), float3(0,1,0), preset, 1); //for HMS_Daring_Type_45.obj
    render(image_ref, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/check_" + model_name + "_ref.png").c_str(), image_ref); 

    long mesh_total_bytes = 0;
    BVHRT *bvh = dynamic_cast<BVHRT*>(pRender->GetAccelStruct()->UnderlyingImpl(0));
    mesh_total_bytes = bvh->m_allNodePairs.size()*sizeof(BVHNodePair) + 
                       bvh->m_primIdCount.size()* sizeof(uint32_t) +
                       bvh->m_vertPos.size()* sizeof(float4) +
                       bvh->m_vertNorm.size()* sizeof(float4) +
                       bvh->m_indices.size()* sizeof(uint32_t) +
                       bvh->m_primIndices.size()* sizeof(uint32_t);
    printf("mesh %6.1f Mb\n", mesh_total_bytes/(1024.0f*1024.0f));
  }

  printf("[check_model::INFO] Rendered mesh\n");

  constexpr int max_depth = 9;
  std::array<float, max_depth> PSNRs = {0, 0, 0, 0, 0, 0, 0, 0}; 
  float max_psnr = 0;

  for (int depth = 3; depth < max_depth; depth++)
  {
    printf("[check_model::INFO] Building SDF with depth %d\n", depth);
    SparseOctreeSettings settings = SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, depth);
    sdf_converter::GlobalOctree g;
    g.header.brick_size = 4;
    g.header.brick_pad = 0;
    
    COctreeV3 coctree;
    coctree.header.bits_per_value = 8;
    coctree.header.brick_size = g.header.brick_size;
    coctree.header.brick_pad = g.header.brick_pad;
    coctree.header.uv_size = 0;
    coctree.header.sim_compression = 1;
    
    scom::Settings scom_settings;
    scom_settings.similarity_threshold = 0.075f;
    scom_settings.search_algorithm = scom::SearchAlgorithm::BALL_TREE;
    scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::REPLACEMENT;

    auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
    printf("[check_model::INFO]   Built TLO\n");
    sdf_converter::mesh_octree_to_global_octree(mesh, tlo, g);
    printf("[check_model::INFO]   Built Global Octree\n");
    sdf_converter::global_octree_to_compact_octree_v3(g, coctree, 8, scom_settings);
    printf("[check_model::INFO]   Built Compact Octree\n");

    auto pRender = CreateMultiRenderer(DEVICE_GPU);
    preset.normal_mode = g.header.brick_pad == 1 ? NORMAL_MODE_SDF_SMOOTHED : NORMAL_MODE_VERTEX;
    pRender->SetPreset(preset);
    pRender->SetScene(coctree, 0);
    //render(image, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza
    //render(image, pRender, float3(-0.75,-0.75,1.25), float3(-0.75,-0.75,0), float3(0,1,0), preset, 1); //for HMS_Daring_Type_45.obj
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/check_" + model_name + "_depth_" + std::to_string(depth) + ".png").c_str(), image); 

    float psnr = image_metrics::PSNR(image, image_ref);
    max_psnr = std::max(max_psnr, psnr);
    PSNRs[depth] = psnr;
    printf("[check_model::INFO] Rendered Compact Octree. PSNR = %.2f\n", psnr);
  }

  printf("[check_model::INFO] ==================================================================\n");
  if (max_psnr < 30)
    printf("[check_model::INFO] OUR ALGORITHM FAILED TO CONVERT THIS MODEL. DO NOT USE IT\n");
  else if (max_psnr < 40)
    printf("[check_model::INFO] OUR ALGORITHM CONVERTED WITH MODEL WITH FLAWS. USE IT WITH CAUTION\n");
  else if (max_psnr < 50)
    printf("[check_model::INFO] OUR ALGORITHM SUCCEEDED TO CONVERT THIS MODEL\n");
  else
    printf("[check_model::INFO] PSNR IS TOO HIGH. CHECK THE IMAGES\n");
  printf("[check_model::INFO] ==================================================================\n");
}