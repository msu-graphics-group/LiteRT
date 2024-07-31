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
      return dot(diff, diff);
    }
    break;
    case DR_LOSS_FUNCTION_MAE:
    {
      float3 diff = abs(color - ref_color);
      return diff.x + diff.y + diff.z;
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
    case DR_LOSS_FUNCTION_MAE:
    {
      float3 diff = color - ref_color;
      return sign(diff);
    }
    break;
    default:
      break;
    }
    return float3(0,0,0);
  }

  static uint32_t diff_render_mode_to_multi_render_mode(uint32_t diff_render_mode)
  {
    switch (diff_render_mode)
    {
    case DR_RENDER_MODE_DIFFUSE:
      return MULTI_RENDER_MODE_DIFFUSE;
    case DR_RENDER_MODE_LAMBERT:
      return MULTI_RENDER_MODE_LAMBERT;    
    case DR_RENDER_MODE_MASK:
      return MULTI_RENDER_MODE_MASK;
    default:
      printf("Unknown diff_render_mode: %u\n", diff_render_mode);
      return MULTI_RENDER_MODE_DIFFUSE;
    }
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

  void MultiRendererDR::OptimizeColor(MultiRendererDRPreset preset, SdfSBS &sbs, bool verbose)
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

    m_dLoss_dS_tmp = std::vector<float>(params_count*max_threads, 0);
    m_Opt_tmp = std::vector<float>(2*params_count, 0);

    m_preset_dr = preset;
    m_preset.spp = preset.spp;
    m_preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON; //we need newton to minimize calculations for border integral
    m_preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
    m_preset.render_mode = diff_render_mode_to_multi_render_mode(preset.dr_render_mode);

    m_width = m_imagesRef[0].width();
    m_height = m_imagesRef[0].height();
    m_images.resize(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));

    SetScene(sbs, true);
    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();
    unsigned images_count = m_imagesRef.size();

    for (int iter = 0; iter < preset.opt_iterations; iter++)
    {
      float loss_sum = 0;
      float loss_max = -1e6;
      float loss_min = 1e6;

      //render (with multithreading)
      for (int image_iter = 0; image_iter < preset.image_batch_size; image_iter++)
      {
        unsigned image_id = rand() % images_count;
        SetViewport(0,0, m_width, m_height);
        UpdateCamera(m_worldViewRef[image_id], m_projRef[image_id]);
        Clear(m_width, m_height, "color");
        std::fill(m_dLoss_dS_tmp.begin(), m_dLoss_dS_tmp.end(), 0.0f);

        float loss = 1e6f;
        if (preset.dr_diff_mode == DR_DIFF_MODE_DEFAULT)
        {
          loss = RenderDR(m_imagesRef[image_id].data(), m_images[image_id].data(), m_dLoss_dS_tmp.data(), params_count);
        }
        else if (preset.dr_diff_mode == DR_DIFF_MODE_FINITE_DIFF)
        {
          bool is_geometry = preset.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_GEOMETRY;
          unsigned color_params = 3*8*sbs.nodes.size();
          unsigned active_params_start = is_geometry ? 0 : params_count - color_params;
          unsigned active_params_end   = is_geometry ? params_count - color_params : params_count;

          loss = RenderDRFiniteDiff(m_imagesRef[image_id].data(), m_images[image_id].data(), m_dLoss_dS_tmp.data(), params_count,
                                    active_params_start, active_params_end, 0.001f);
        }

        loss_sum += loss;
        loss_max = std::max(loss_max, loss);
        loss_min = std::min(loss_min, loss);

        //if (verbose && iter % 10 == 0)
        //  printf("%f\n", loss);
        if (verbose && image_id == 0)
        {
          printf("Iter:%4d, loss: %f (%f-%f)\n", iter, loss_sum/preset.image_batch_size, loss_min, loss_max);
          LiteImage::SaveImage<float4>(("saves/iter_"+std::to_string(iter)+"_"+std::to_string(image_id)+".bmp").c_str(), m_images[image_id]);
        }
      }

      //accumulate
      for (int i=1; i< max_threads; i++)
        for (int j = 0; j < params_count; j++)
          m_dLoss_dS_tmp[j] += m_dLoss_dS_tmp[j + params_count * i];

      for (int j = 0; j < params_count; j++)
        m_dLoss_dS_tmp[j] /= preset.image_batch_size;

      OptimizeStepAdam(iter, m_dLoss_dS_tmp.data(), params, m_Opt_tmp.data(), params_count, preset);
    } 
  }

  float MultiRendererDR::RenderDR(const float4 *image_ref, LiteMath::float4 *out_image,
                                  float *out_dLoss_dS, unsigned params_count)
  {
    unsigned max_threads = omp_get_max_threads();
    unsigned steps = (m_width * m_height + max_threads - 1)/max_threads;
    std::vector<double> loss_v(max_threads, 0.0f);
    #pragma omp parallel for
    for (int thread_id = 0; thread_id < max_threads; thread_id++)
    {
      unsigned start = thread_id * steps;
      unsigned end = std::min((thread_id + 1) * steps, m_width * m_height);
      for (int i = start; i < end; i++)
        loss_v[thread_id] += CastRayWithGrad(i, image_ref, out_image, out_dLoss_dS + (params_count * thread_id));
    }
    double loss = 0.0f;
    for (auto &l : loss_v)
      loss += l;
    
    return loss/(m_width * m_height);
  }

  float MultiRendererDR::RenderDRFiniteDiff(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                                            unsigned start_index, unsigned end_index, float delta)
  {
    assert(end_index > start_index);
    assert(end_index <= params_count);
    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();

    for (unsigned i = start_index; i < end_index; i++)
    {
      float p0 = params[i];
      double loss_plus = 0.0f;
      double loss_minus = 0.0f;

      params[i] = p0 + delta;
      RenderFloat(out_image, m_width, m_height, "color");
      for (int j = 0; j < m_width * m_height; j++)
        loss_plus += Loss(m_preset_dr.dr_loss_function, to_float3(out_image[j]), to_float3(image_ref[j]));
    
      params[i] = p0 - delta;
      RenderFloat(out_image, m_width, m_height, "color");
      for (int j = 0; j < m_width * m_height; j++)
        loss_minus += Loss(m_preset_dr.dr_loss_function, to_float3(out_image[j]), to_float3(image_ref[j]));
      
      params[i] = p0;

      //it is the same way as loss is accumulated in RenderDR
      //loss_plus /= m_width * m_height;
      //loss_minus /= m_width * m_height;
      //printf("loss_plus = %f, loss_minus = %f\n", loss_plus, loss_minus);

      out_dLoss_dS[i] = (loss_plus - loss_minus) / (2 * delta);
    }

    double loss = 0.0f;
    //RenderFloat(out_image, m_width, m_height, "color");
    for (int j = 0; j < m_width * m_height; j++)
      loss += Loss(m_preset_dr.dr_loss_function, to_float3(out_image[j]), to_float3(image_ref[j]));
    
    return loss / (m_width * m_height);
  }

  void MultiRendererDR::OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset)
  {
    float lr = preset.opt_lr;
    float beta_1 = preset.opt_beta_1;
    float beta_2 = preset.opt_beta_2;
    float eps = preset.opt_eps;

    float *V = tmp;
    float *S = tmp + size;

    std::vector<uint2> borders{uint2(0, size)};
    std::vector<float2> limits{float2(-1000.0f, 1000.0f)};

    for (int border_id = 0; border_id < borders.size(); border_id++)
    {
      for (int i = borders[border_id].x; i < borders[border_id].y; i++)
      {
        float g = dX[i];
        V[i] = beta_1 * V[i] + (1 - beta_1) * g;
        float Vh = V[i] / (1 - pow(beta_1, iter + 1));
        S[i] = beta_2 * S[i] + (1 - beta_2) * g * g;
        float Sh = S[i] / (1 - pow(beta_2, iter + 1));
        X[i] = LiteMath::clamp(X[i] - lr * Vh / (sqrt(Sh) + eps), limits[border_id].x, limits[border_id].y);
      }
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
    float4 color_min = float4(1e10f, 1e10f, 1e10f, 1e10f);
    float4 color_max = float4(-1e10f, -1e10f, -1e10f, -1e10f);
    float res_loss = 0.0f;
    uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
    float i_spp_sqrt = 1.0f/spp_sqrt;

    const float relax_eps = 1e-4f;
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);

    uint32_t ray_flags, border_ray_flags;

    if (m_preset_dr.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_COLOR)
    {
      ray_flags = DR_RAY_FLAG_DDIFFUSE_DCOLOR;
    }
    else if (m_preset_dr.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_GEOMETRY)
    {
      if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_MASK)
        ray_flags = DR_RAY_FLAG_NO_DIFF;
      else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_DIFFUSE)
        ray_flags = DR_RAY_FLAG_DDIFFUSE_DPOS;
      else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_LAMBERT)
        ray_flags = DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS;
    }

    border_ray_flags = DR_RAY_FLAG_BORDER;

    for (uint32_t i = 0; i < m_preset.spp; i++)
    {
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
      
      float4 color = float4(0,0,0,1);
      LiteMath::float3x3 dColor_dDiffuse = LiteMath::make_float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
      LiteMath::float3x3 dColor_dNorm    = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
      
      CRT_HitDR hit = ((BVHDR*)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(ray_flags, rayPosAndNear, rayDirAndFar, nullptr);
      
      if (hit.primId == 0xFFFFFFFF) //no hit
      {
        color = to_float4(background_color, 1.0f);
        res_color += color / m_preset.spp;
        //continue;
      }
      else
      {
        unsigned type = hit.geomId >> SH_TYPE;
        unsigned geomId = hit.geomId & 0x0FFFFFFF;

        float4 diffuse = float4(1,0,0,1);
        if (type == TYPE_SDF_SBS_COL)
        {
          diffuse = float4(hit.color.x, hit.color.y, hit.color.z, 1.0f);
        }

        color = float4(1,0,1,1); //if pixel is purple at the end, then something gone wrong!


        /*
        color = color(pos, diffuse, norm)
        dcolor/dS = dcolor/dpos * dpos/dS + dcolor/ddiffuse * ddiffuse/dS + dcolor/dnorm * dnorm/dS
        dcolor/dpos == 0 for DIFFUSE and LAMBERT render modes
        */
        switch (m_preset_dr.dr_render_mode)
        {
          case DR_RENDER_MODE_DIFFUSE:
            color = diffuse;

            dColor_dDiffuse = LiteMath::make_float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
            dColor_dNorm    = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
          break;
          case DR_RENDER_MODE_LAMBERT:
          {
            float3 light_dir = normalize(float3(1, 1, 1));
            float3 norm = hit.normal;
            float q0 = dot(norm, light_dir); // = norm.x*light_dir.x + norm.y*light_dir.y + norm.z*light_dir.z
            float q = max(0.1f, q0);
            color = to_float4(q * to_float3(diffuse), 1);

            dColor_dDiffuse = LiteMath::make_float3x3(float3(q,0,0), float3(0,q,0), float3(0,0,q));
            if (q0 > 0.1f)
            {
              float3 dq_dnorm = light_dir;
              dColor_dNorm = LiteMath::make_float3x3(diffuse.x*light_dir, 
                                                    diffuse.y*light_dir, 
                                                    diffuse.z*light_dir);
            }
            else
              dColor_dNorm = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
          }
          break; 
          case DR_RENDER_MODE_MASK:
            color = float4(1,1,1,1);

            dColor_dDiffuse = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
            dColor_dNorm    = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
          break;     
          default:
          break;
        }

        float3 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, to_float3(color), to_float3(image_ref[y * m_width + x]));

        if (m_preset_dr.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_COLOR)
        {
          for (PDColor &pd : hit.dDiffuse_dSc)
          {
            float3 diff = dLoss_dColor * (dColor_dDiffuse * float3(pd.value));
            out_dLoss_dS[pd.index + 0] += diff.x / m_preset.spp;
            out_dLoss_dS[pd.index + 1] += diff.y / m_preset.spp;
            out_dLoss_dS[pd.index + 2] += diff.z / m_preset.spp;
          }
        }
        else if (m_preset_dr.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_GEOMETRY &&
                 ray_flags & (DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS))
        {
          for (PDDist &pd : hit.dDiffuseNormal_dSd)
          {
            float diff = dot(dLoss_dColor, dColor_dDiffuse * pd.dDiffuse + dColor_dNorm * pd.dNorm);
            out_dLoss_dS[pd.index] += diff / m_preset.spp;
          }
        }
      }

      res_color += color / m_preset.spp;
      color_min = min(color_min, color);
      color_max = max(color_max, color);

      float loss          = Loss(m_preset_dr.dr_loss_function, to_float3(color), to_float3(image_ref[y * m_width + x]));
      res_loss += loss / m_preset.spp;
    }
    out_image[y * m_width + x] = res_color;

    if (m_preset_dr.dr_reconstruction_type == DR_RECONSTRUCTION_TYPE_GEOMETRY)
    {
      unsigned max_border_spp = m_preset_dr.border_spp;
      unsigned border_spp = max_border_spp * LiteMath::clamp(length(color_min - color_max) - 0.05f, 0.0f, 1.0f);
      uint32_t spp_sqrt = uint32_t(sqrt(border_spp));
      float i_spp_sqrt = 1.0f/spp_sqrt;
      const float debias_mult = 1.0f;

      for (int sample_id = 0; sample_id < border_spp; sample_id++)
      {
        float3 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, to_float3(out_image[y * m_width + x]), to_float3(image_ref[y * m_width + x]));
        PDShape relax_pt{
                        .f_in  = background_color,
                        .t = 0.f,
                        .f_out = background_color,
                        .sdf = relax_eps, // only choose points with SDF() less than this, no need to pass relax_eps
                        .indices = { 0, 0, 0, 0, 0, 0, 0, 0 },
                        .dSDF_dtheta = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f }
                      };
        float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, sample_id) : i_spp_sqrt*float2(sample_id/spp_sqrt+0.5, sample_id%spp_sqrt+0.5);
        float4 rayPosAndNear, rayDirAndFar;
        kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);        
        CRT_HitDR hit = ((BVHDR*)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(border_ray_flags, rayPosAndNear, rayDirAndFar, &relax_pt);
        
        if (relax_pt.sdf < relax_eps)
        {
          float ad = length(relax_pt.f_in - relax_pt.f_out);
          for (int j = 0; j < 8; j++)
          {
            float diff = dot(dLoss_dColor, (1.0f/relax_eps)*relax_pt.dSDF_dtheta[j]*(relax_pt.f_in - relax_pt.f_out));
            out_dLoss_dS[relax_pt.indices[j]] += debias_mult * diff / border_spp;
          }
        }
      }
    }

    return res_loss;
  }
}