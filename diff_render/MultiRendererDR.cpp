#include "MultiRendererDR.h"
#include "BVH2DR.h"
#include <omp.h>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;

using LiteMath::inverse4x4;
using LiteMath::lookAt;
using LiteMath::perspectiveMatrix;

namespace dr
{
  static float Loss(unsigned loss_function, float3 color, float3 ref_color)
  {
    switch (loss_function)
    {
    case DR_LOSS_FUNCTION_MSE:
      {
       float3 diff = color - ref_color;
       return sqrt(dot(diff, diff));
      }
      break;
    
    default:
      break;
    }
    return 0;
  }

  static float3 LossGrad(unsigned loss_function, float3 color, float3 ref_color)
  {
    switch (loss_function)
    {
    case DR_LOSS_FUNCTION_MSE:
    {
      float3 diff = color - ref_color;
      return 2.0f * diff;
    }
      break;
    
    default:
      break;
    }
    return float3(0,0,0);
  }

  MultiRendererDR::MultiRendererDR()
  {
    m_pAccelStruct = std::shared_ptr<ISceneObject>(new BVHDR());
    m_preset = getDefaultPreset();
    m_preset_dr = getDefaultPresetDR();
    m_mainLightDir = normalize3(float4(1, 0.5, 0.5, 1));
    m_mainLightColor = 1.0f * normalize3(float4(1, 1, 0.98, 1));
    m_seed = rand();
  }

  void MultiRendererDR::SetReference(const std::vector<LiteImage::Image2D<float4>> &images,
                                     const std::vector<LiteMath::float4x4> &worldView,
                                     const std::vector<LiteMath::float4x4> &proj)
  {
    m_imagesRef = images;
    m_worldViewRef = worldView;
    m_projRef = proj;

  }

  void MultiRendererDR::OptimizeColor(MultiRendererDRPreset preset, SdfSBS &sbs)
  {
    assert(sbs.nodes.size() > 0);
    assert(sbs.values.size() > 0);
    assert(sbs.values_f.size() > 0);
    assert(sbs.header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F);
    assert(preset.opt_iterations > 0);
    assert(preset.spp > 0);
    assert(m_imagesRef.size() > 0);
    assert(m_worldViewRef.size() == m_imagesRef.size());
    assert(m_projRef.size() == m_imagesRef.size());
    for (int i = 1; i < m_imagesRef.size(); i++)
    {
      assert(m_imagesRef[i].width() == m_imagesRef[0].width());
      assert(m_imagesRef[i].height() == m_imagesRef[0].height());
    }

    unsigned max_threads = omp_get_max_threads();
    unsigned params_count = sbs.values_f.size();

    std::vector<float> m_dLoss_dS_tmp = std::vector<float>(params_count*max_threads, 0);
    std::vector<float> m_Opt_tmp = std::vector<float>(2*params_count, 0);

    m_preset_dr = preset;
    m_preset.spp = preset.spp;
    m_preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;

    m_width = m_imagesRef[0].width();
    m_height = m_imagesRef[0].height();
    m_images.resize(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));

    SetScene(sbs, true);
    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();

    //SetViewport(0,0, a_width, a_height);
    //UpdateCamera(a_worldView, a_proj);
    for (int iter = 0; iter < preset.opt_iterations; iter++)
    {
      //render (with multithreading)
      for (int image_id = 0; image_id < m_imagesRef.size(); image_id++)
      {
        SetViewport(0,0, m_width, m_height);
        UpdateCamera(m_worldViewRef[image_id], m_projRef[image_id]);
        Clear(m_width, m_height, "color");
        std::fill(m_dLoss_dS_tmp.begin(), m_dLoss_dS_tmp.end(), 0.0f);

        RenderDR(m_imagesRef[image_id].data(), m_images[image_id].data(), m_dLoss_dS_tmp.data(), params_count);
      }

      //accumulate
      for (int i=1; i< max_threads; i++)
        for (int j = 0; j < params_count; j++)
          m_dLoss_dS_tmp[j] += m_dLoss_dS_tmp[j + params_count * i];

      OptimizeStepAdam(iter, m_dLoss_dS_tmp.data(), params, m_Opt_tmp.data(), params_count, preset);

      if (true && iter % 10 == 0)
      {
        LiteImage::SaveImage<float4>(("saves/iter_"+std::to_string(iter)+".bmp").c_str(), m_images[0]); 
      }
    } 
  }

  void MultiRendererDR::RenderDR(const float4 *image_ref, LiteMath::float4 *out_image,
                                 float *out_dLoss_dS, unsigned params_count)
  {
    #pragma omp parallel for
    for (int i = 0; i < m_width * m_height; i++)
      CastRayWithGrad(i, image_ref, out_image, out_dLoss_dS + (params_count * omp_get_thread_num()));
  }

  void MultiRendererDR::OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset)
  {
    float lr = preset.opt_lr;
    float beta_1 = preset.opt_beta_1;
    float beta_2 = preset.opt_beta_2;
    float eps = preset.opt_eps;

    float *V = tmp;
    float *S = tmp + size;

    for (int i = 0; i < size; i++)
    {
      float g = dX[i];
      V[i] = beta_1 * V[i] + (1 - beta_1) * g;
      float Vh = V[i] / (1 - pow(beta_1, iter + 1));
      S[i] = beta_2 * S[i] + (1 - beta_2) * g * g;
      float Sh = S[i] / (1 - pow(beta_2, iter + 1));
      X[i] = X[i] - lr * Vh / (sqrt(Sh) + eps);
    }
  }

  float MultiRendererDR::CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4 *out_image, float *out_dLoss_dS)
  {
    if (tidX >= m_packedXY.size())
      return 0.0;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;
    
    float4 res_color = float4(0,0,0,0);
    float res_loss = 0.0f;
    uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
    float i_spp_sqrt = 1.0f/spp_sqrt;

    for (uint32_t i = 0; i < m_preset.spp; i++)
    {
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
      
      float4 color = float4(0,0,0,1);
      CRT_HitDR hit = ((BVHDR*)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(rayPosAndNear, rayDirAndFar);
      if (hit.primId == 0xFFFFFFFF) //no hit
      {
        res_color += color / m_preset.spp;
        continue;
      }
      unsigned type = hit.geomId >> SH_TYPE;
      unsigned geomId = hit.geomId & 0x0FFFFFFF;

      float4 diffuse = float4(1,0,0,1);
      if (type == TYPE_SDF_SBS_COL)
      {
        diffuse.x = std::round(hit.coords[0])/255.0f;
        diffuse.y = LiteMath::fract(hit.coords[0]);
        diffuse.z = std::round(hit.coords[1])/255.0f;
      }

      color = float4(1,0,1,1); //if pixel is purple at the end, then something gone wrong!
      LiteMath::float3x3 dColor_dDiffuse = LiteMath::float3x3();
      //printf("m_preset_dr.dr_render_mode = %u\n", m_preset_dr.dr_render_mode);

      switch (m_preset_dr.dr_render_mode)
      {
        case DR_RENDER_MODE_DIFFUSE:
          dColor_dDiffuse = LiteMath::make_float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
          color = diffuse;
          break;
        case DR_RENDER_MODE_LAMBERT:
          /* code */
          break;      
        default:
          break;
      }

      res_color += color / m_preset.spp;

      float loss          = Loss(m_preset_dr.dr_loss_function, to_float3(color), to_float3(image_ref[y * m_width + x]));
      res_loss += loss / m_preset.spp;

      float3 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, to_float3(color), to_float3(image_ref[y * m_width + x]));

      for (PD &pd : hit.dDiffuse_dS)
      {
        if (pd.index == PD::INVALID_INDEX)
          continue;
        
        float3 diff = dLoss_dColor * (dColor_dDiffuse * float3(pd.value));
        out_dLoss_dS[pd.index + 0] += diff.x / m_preset.spp;
        out_dLoss_dS[pd.index + 1] += diff.y / m_preset.spp;
        out_dLoss_dS[pd.index + 2] += diff.z / m_preset.spp;
      }

    }
    out_image[y * m_width + x] = res_color;

    return res_loss;
  }
}