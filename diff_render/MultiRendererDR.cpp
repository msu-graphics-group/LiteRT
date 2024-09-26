#include "MultiRendererDR.h"
#include "BVH2DR.h"
#include "utils/points_visualizer.h"

#include <omp.h>
#include <chrono>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;

using LiteMath::inverse4x4;
using LiteMath::lookAt;
using LiteMath::perspectiveMatrix;

//extern float __delta;

namespace dr
{
  static constexpr float z_near = 0.1;
  static constexpr float z_far = 10;

  static float3 visualize_value_debug(float val)
  {
    float diff_log = log10(abs(val) + 1.0f);
    float q_red = std::max(0.0f, 1.f - abs(diff_log - 3.0f));
    float q_green = std::max(0.0f, 1.f - abs(diff_log - 2.0f));
    float q_blue = std::max(0.0f, 1.f - abs(diff_log - 1.0f));

    return float3(q_red, q_green, q_blue);
  }


  static float urand(float from=0, float to=1)
  {
    return ((double)rand() / RAND_MAX) * (to - from) + from;
  }

  static float linear_to_absolute_depth(float lz)
  {
    return lz * (z_far - z_near) + z_near;
  }

  static float absolute_to_linear_depth(float z)
  {
    return (z - z_near) / (z_far - z_near);
  }

  static float Loss(unsigned loss_function, float4 color, float4 ref_color)
  {
    float4 mult = float4(1,1,1,1);
    switch (loss_function)
    {
    case DR_LOSS_FUNCTION_MSE:
    {
      float4 diff = mult*(color - ref_color);
      //printf("color ref_color, diff = (%f %f %f %f) (%f %f %f %f)\n", color.x, color.y, color.z, color.w, ref_color.x, ref_color.y, ref_color.z, ref_color.w);
      return dot(diff, diff);
    }
    break;
    case DR_LOSS_FUNCTION_MAE:
    {
      float4 diff = mult*abs(color - ref_color);
      return diff.x + diff.y + diff.z + diff.w;
    }
    break;    
    default:
      break;
    }
    return 0;
  }

  static float4 LossGrad(unsigned loss_function, float4 color, float4 ref_color)
  {
    float4 mult = float4(1,1,1,1);
    switch (loss_function)
    {
    case DR_LOSS_FUNCTION_MSE:
    {
      float4 diff = (color - ref_color);
      return 2.0f * mult * diff;
    }
    break;
    case DR_LOSS_FUNCTION_MAE:
    {
      float4 diff = (color - ref_color);
      return mult*sign(diff);
    }
    break;
    default:
      break;
    }
    return float4(0,0,0,0);
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
    case DR_RENDER_MODE_LINEAR_DEPTH:
      return MULTI_RENDER_MODE_LINEAR_DEPTH;
    default:
      printf("Unknown diff_render_mode: %u\n", diff_render_mode);
      return MULTI_RENDER_MODE_DIFFUSE;
    }
  }

  MultiRendererDR::MultiRendererDR(uint32_t maxPrimitives):
  MultiRenderer(maxPrimitives)
  {
    m_preset = getDefaultPreset();
    m_preset_dr = getDefaultPresetDR();
    m_mainLightDir = normalize3(float4(1, 0.5, 0.5, 1));
    m_mainLightColor = 1.0f * normalize3(float4(1, 1, 0.98, 1));
    m_seed = rand();

    //  extend object frame on k pixels to cast rays right in object and not in empty space 
    add_border = 0;
  }

  void MultiRendererDR::cleanMasks()
  {
    for (auto &mask: masks)
    {
      mask.clear();
    }

    masks.clear();
    process_mask.clear();
  }

  void 
  MultiRendererDR::setBorderThickness(uint32_t thickness)
  {
    this->add_border = thickness;
  }

  void MultiRendererDR::SetReference(const std::vector<LiteImage::Image2D<float4>> &images,
                                     const std::vector<LiteMath::float4x4> &worldView,
                                     const std::vector<LiteMath::float4x4> &proj)
  {
    m_imagesRefOriginal = images;
    m_worldViewRef = worldView;
    m_projRef = proj;

    CreateRefImageMasks();
  }

  void MultiRendererDR::SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                                     const std::vector<LiteImage::Image2D<float4>>& masks, 
                                     const std::vector<LiteMath::float4x4>& worldView, 
                                     const std::vector<LiteMath::float4x4>& proj)
  {
    m_imagesRefOriginal = images;
    m_imagesRefMask = masks;
    m_worldViewRef = worldView;
    m_projRef = proj;
  }

  void MultiRendererDR::CreateRefImageMasks()
  {
    assert(m_imagesRefOriginal.size() > 0);
    
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);

    m_imagesRefMask = std::vector<LiteImage::Image2D<float4>>(m_imagesRefOriginal.size());
    for (unsigned image_n = 0; image_n < m_imagesRefOriginal.size(); image_n++)
    {
      m_imagesRefMask[image_n] = LiteImage::Image2D<float4>(m_imagesRefOriginal[image_n].width(), m_imagesRefOriginal[image_n].height());
      for (int i=0;i<m_imagesRefOriginal[image_n].width()*m_imagesRefOriginal[image_n].height();i++)
      {
        float3 color = to_float3(m_imagesRefOriginal[image_n].data()[i]);
        m_imagesRefMask[image_n].data()[i] = length(color - background_color) < 0.001f ? float4(0,0,0,0) : float4(1,1,1,1);
      }
    }
  }

  void MultiRendererDR::PreprocessRefImages(unsigned width, unsigned height, bool to_mask, float3 background_color)
  {
    //preprocess reference images (m_imagesRefOriginal) to m_imagesRef, that will be used for optimization
    m_imagesRef = std::vector<LiteImage::Image2D<float4>>(m_imagesRefOriginal.size());
    for (unsigned image_n = 0; image_n < m_imagesRefOriginal.size(); image_n++)
    {
      //get reference image by sampling original (or mask) with default bilinear sampler
      LiteImage::Sampler sampler = LiteImage::Sampler();
      sampler.filter = LiteImage::Sampler::Filter::LINEAR;
      unsigned spp_x = ceil(float(m_imagesRefOriginal[image_n].width()) / width);
      unsigned spp_y = ceil(float(m_imagesRefOriginal[image_n].height()) / height);

      m_imagesRef[image_n] = LiteImage::Image2D<float4>(width, height);
      for (unsigned y = 0; y < height; y++)
      {
        for (unsigned x = 0; x < width; x++)
        {
          for (unsigned dx = 0; dx < spp_x; dx++)
          {
            for (unsigned dy = 0; dy < spp_y; dy++)
            {
              float2 uv = float2((spp_x*x + dx + 0.5f) / (spp_x*width), (spp_y*y + dy + 0.5f) / (spp_y*height));
              float4 color(0,0,0,0);
              if (to_mask)
                color = m_imagesRefMask[image_n].sample(sampler, uv);
              else
              {
                color   = m_imagesRefOriginal[image_n].sample(sampler, uv);
                color.w = m_imagesRefMask[image_n].sample(sampler, uv).w;
                //printf("color = %f %f %f %f\n", color.x, color.y, color.z, color.w);
              }
              m_imagesRef[image_n].data()[y*width + x] += color / (spp_x*spp_y);
            }
          }
        }
      }
      LiteImage::SaveImage<float4>(("saves/ref_" + std::to_string(image_n) + ".png").c_str(), m_imagesRef[image_n]);
    }
  }

  void MultiRendererDR::OptimizeGrid(unsigned start_grid_size, bool no_last_step_resize, std::vector<MultiRendererDRPreset> presets)
  {
    unsigned grid_size = start_grid_size;
    unsigned grid_steps = presets.size();
    unsigned brick_size = 1;

    auto grid =  create_grid_sbs(grid_size, brick_size, 
                                 [&](float3 p){return circle_sdf(float3(0,-0.15f,0), 0.7f, p);}, 
                                 gradient_color);
    
    for (unsigned i = 0; i < grid_steps; i++)
    {
      OptimizeFixedStructure(presets[i], grid);

      auto grid_sampler = [&](float3 p){
        unsigned p_count = grid_size*brick_size + 1u;
        float3 idx_f = clamp(0.5f*(p + 1.0f), 0.0f, 1.0f + 1e-6f) * grid_size*brick_size;
        uint3 idx = uint3(idx_f + 1e-6f);
        float3 dp = idx_f - float3(idx);
        float dist = 1000;
        if (idx.x < p_count-1 && idx.y < p_count-1 && idx.z < p_count-1)
        {
          dist = (1 - dp.x) * (1 - dp.y) * (1 - dp.z) * grid.values_f[idx.x * p_count*p_count + idx.y * p_count + idx.z] +
                 (1 - dp.x) * (1 - dp.y) * dp.z * grid.values_f[idx.x * p_count*p_count + idx.y * p_count + idx.z+1] +
                 (1 - dp.x) * dp.y * (1 - dp.z) * grid.values_f[idx.x * p_count*p_count + (idx.y+1) * p_count + idx.z] +
                 (1 - dp.x) * dp.y * dp.z * grid.values_f[idx.x * p_count*p_count + (idx.y+1) * p_count + idx.z+1] +
                 dp.x * (1 - dp.y) * (1 - dp.z) * grid.values_f[(idx.x+1) * p_count*p_count + idx.y * p_count + idx.z] +
                 dp.x * (1 - dp.y) * dp.z * grid.values_f[(idx.x+1) * p_count*p_count + idx.y * p_count + idx.z+1] +
                 dp.x * dp.y * (1 - dp.z) * grid.values_f[(idx.x+1) * p_count*p_count + (idx.y+1) * p_count + idx.z] +
                 dp.x * dp.y * dp.z * grid.values_f[(idx.x+1) * p_count*p_count + (idx.y+1) * p_count + idx.z+1];
        }
        else 
        {
          dist = grid.values_f[idx.x*p_count*p_count + idx.y*p_count + idx.z];
        }
        return dist;

      };

      unsigned new_brick_size;
      if (i == grid_steps-1 && no_last_step_resize) 
        new_brick_size = brick_size; 
      else 
        new_brick_size = 2*brick_size;
      auto new_grid = create_grid_sbs(grid_size, new_brick_size, grid_sampler, gradient_color);
      brick_size = new_brick_size;
      grid = new_grid;
    }
  }

  void MultiRendererDR::OptimizeFixedStructure(MultiRendererDRPreset preset, SdfSBS &sbs)
  {
    assert(sbs.nodes.size() > 0);
    assert(sbs.values.size() > 0);
    assert(sbs.values_f.size() > 0);
    assert(sbs.header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F);
    assert(preset.opt_iterations > 0);
    assert(preset.spp > 0);
    assert(m_imagesRefOriginal.size() > 0);
    assert(m_worldViewRef.size() == m_imagesRefOriginal.size());
    assert(m_projRef.size() == m_imagesRefOriginal.size());
    if (preset.dr_input_type == DR_INPUT_TYPE_LINEAR_DEPTH)
    {
      assert(preset.dr_render_mode == DR_RENDER_MODE_LINEAR_DEPTH);
      assert((preset.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_COLOR) == 0);
    }

    unsigned max_threads = omp_get_max_threads();
    unsigned params_count = sbs.values_f.size();

    m_dLoss_dS_tmp = std::vector<float>(params_count*max_threads, 0);
    m_Opt_tmp = std::vector<float>(2*params_count, 0);
    m_PD_tmp = std::vector<PDFinalColor>(2*8*preset.spp*max_threads);

    m_preset_dr = preset;
    m_preset.spp = preset.spp;
    m_preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON; //we need newton to minimize calculations for border integral
    m_preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
    if (preset.debug_render_mode == DR_DEBUG_RENDER_MODE_PRIMITIVE)
      m_preset.render_mode = MULTI_RENDER_MODE_PRIMITIVE;
    else
      m_preset.render_mode = diff_render_mode_to_multi_render_mode(preset.dr_render_mode);

    if (preset.render_width == 0 || preset.render_height == 0)
    {
      for (int i = 1; i < m_imagesRefOriginal.size(); i++)
      {
        assert(m_imagesRefOriginal[i].width() == m_imagesRefOriginal[0].width());
        assert(m_imagesRefOriginal[i].height() == m_imagesRefOriginal[0].height());
      }

      m_width = m_imagesRefOriginal[0].width();
      m_height = m_imagesRefOriginal[0].height();
    }
    else
    {
      m_width = preset.render_width;
      m_height = preset.render_height;
    }

    PreprocessRefImages(m_width, m_height, preset.dr_render_mode == DR_RENDER_MODE_MASK);

    m_images = std::vector<LiteImage::Image2D<float4>>(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));
    m_imagesDepth = std::vector<LiteImage::Image2D<float4>>(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));
    SetAccelStruct(std::shared_ptr<ISceneObject>(new BVHDR()));
    SetPreset(m_preset);
    SetScene(sbs, true);

    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();
    unsigned images_count = m_imagesRef.size();

    if (m_preset_dr.debug_pd_images)
      m_imagesDebugPD = std::vector<LiteImage::Image2D<float4>>(params_count, LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));
    
    if (m_preset_dr.debug_render_mode != DR_DEBUG_RENDER_MODE_NONE)
      m_imagesDebug = std::vector<LiteImage::Image2D<float4>>(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));

    if (preset.debug_border_samples_mega_image)
      samples_mega_image = LiteImage::Image2D<float4>(m_width*MEGA_PIXEL_SIZE, m_height*MEGA_PIXEL_SIZE);

    if (m_preset_dr.dr_raycasting_mask == DR_RENDER_MASK_CAST_OPT)
    { 
      masks.resize(images_count);

      for (auto &mask: masks)
      {
        mask.resize(m_width * m_height);
        std::fill(mask.begin(), mask.end(), 1);
      }

      process_mask.resize(m_width * m_height);
      std::fill(process_mask.begin(), process_mask.end(), 0);

      //  Need to get right mask in ray tracing
      mask_ind = 0;
    }

    float timeAvg = 0.0f;

    for (int iter = 0; iter < preset.opt_iterations; iter++)
    {
      float loss_sum = 0;
      float loss_max = -1e6;
      float loss_min = 1e6;

      auto t1 = std::chrono::high_resolution_clock::now();

      //render (with multithreading)
      for (int image_iter = 0; image_iter < preset.image_batch_size; image_iter++)
      {
        bool base_debug_pd_images = m_preset_dr.debug_pd_images;
        m_preset_dr.debug_pd_images = base_debug_pd_images && (image_iter == 0);

        unsigned image_id = preset.image_batch_size == images_count ? image_iter : rand() % images_count;
        mask_ind = image_id;

        SetViewport(0,0, m_width, m_height);
        UpdateCamera(m_worldViewRef[image_id], m_projRef[image_id]);
        Clear(m_width, m_height, "color");
        std::fill(m_dLoss_dS_tmp.begin(), m_dLoss_dS_tmp.end(), 0.0f);

        if (m_preset_dr.debug_render_mode != DR_DEBUG_RENDER_MODE_NONE)
          m_imagesDebug[image_id].clear(float4(0,0,0,1));

        float loss = 1e6f;
        if (preset.dr_diff_mode == DR_DIFF_MODE_DEFAULT)
        {
          loss = RenderDR(m_imagesRef[image_id].data(), m_images[image_id].data(), 
                          m_dLoss_dS_tmp.data(), params_count, m_imagesDepth[image_id].data(),
                          m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_NONE ? nullptr : m_imagesDebug[image_id].data());
        }
        else if (preset.dr_diff_mode == DR_DIFF_MODE_FINITE_DIFF)
        {
          bool is_geometry = (preset.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_GEOMETRY);
          unsigned color_params = 3*8*sbs.nodes.size();
          unsigned active_params_start = is_geometry ? 0 : params_count - color_params;
          unsigned active_params_end   = is_geometry ? params_count - color_params : params_count;

          loss = RenderDRFiniteDiff(m_imagesRef[image_id].data(), m_images[image_id].data(), m_dLoss_dS_tmp.data(), params_count,
                                    active_params_start, active_params_end, 0.01f);
        }

        loss_sum += loss;
        loss_max = std::max(loss_max, loss);
        loss_min = std::min(loss_min, loss);

        m_preset_dr.debug_pd_images = base_debug_pd_images;

        if (m_preset_dr.dr_raycasting_mask == DR_RENDER_MASK_CAST_OPT)
        {
          // masks[mask_ind] = process_mask;
          std::copy(process_mask.begin(), process_mask.end(), masks[mask_ind].begin());
          std::fill(process_mask.begin(), process_mask.end(), 0);
        }
      }

      auto t2 = std::chrono::high_resolution_clock::now();

      //regualization (if needed)
      if (preset.reg_function != DR_REG_FUNCTION_NONE)
        Regularization(m_dLoss_dS_tmp.data(), params_count);

      auto t3 = std::chrono::high_resolution_clock::now();

      // Redistancing every N iterations
      if (preset.redistancing_enable && iter % preset.redistancing_interval == 0)
      {
        dr::BVHDR* bvhdr_tree = dynamic_cast<dr::BVHDR*>(GetAccelStruct().get());
        uint32_t brick_count = std::cbrt(bvhdr_tree->m_SdfSBSNodes.size());
        uint32_t p_count = brick_count * bvhdr_tree->m_SdfSBSHeaders[0].brick_size + 1;
        Redistance(bvhdr_tree->m_SdfSBSDataF.data(), {p_count, p_count, p_count}, 2.f / p_count, p_count);
      }

      auto t4 = std::chrono::high_resolution_clock::now();

      //accumulate
      for (int i=1; i< max_threads; i++)
        for (int j = 0; j < params_count; j++)
          m_dLoss_dS_tmp[j] += m_dLoss_dS_tmp[j + params_count * i];

      for (int j = 0; j < params_count; j++)
        m_dLoss_dS_tmp[j] /= preset.image_batch_size;

      auto t5 = std::chrono::high_resolution_clock::now();

      OptimizeStepAdam(iter, m_dLoss_dS_tmp.data(), params, m_Opt_tmp.data(), params_count, preset);

      auto t6 = std::chrono::high_resolution_clock::now();
      float time_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      float time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
      float time_3 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
      float time_4 = std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
      float time_5 = std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
      float time = time_1 + time_2 + time_3 + time_4 + time_5;
      if (iter == 0)
        timeAvg = time;
      else
        timeAvg = 0.97f * timeAvg + 0.03f * time;

      if (preset.debug_print && iter % preset.debug_print_interval == 0)
      {
        printf("\r");
        printf("Iter:%4d, loss: %f (%f-%f) ", iter, loss_sum/preset.image_batch_size, loss_min, loss_max);
        printf("%7.1f ms/iter (%7.1f + %.1f + %.1f + %.1f + %.1f) ", time, time_1, time_2, time_3, time_4, time_5);
        printf("ETA %.1f s", (timeAvg * (preset.opt_iterations - iter - 1)) / 1000.0f);
        printf("                  ");
        fflush(stdout);
      }
      
      if (preset.debug_print && 
          preset.debug_progress_images != DEBUG_PROGRESS_NONE && 
          iter % preset.debug_progress_interval == 0)
      {
        for (int image_id = 0; image_id < images_count; image_id++)
        {
          if (preset.debug_progress_images != DEBUG_PROGRESS_RAW)
          {
            auto original_mode = m_preset.render_mode;
            m_preset.render_mode = preset.debug_progress_images;
            SetPreset(m_preset);

            UpdateCamera(m_worldViewRef[image_id], m_projRef[image_id]);
            RenderFloat(m_images[image_id].data(), m_width, m_height, "color");

            m_preset.render_mode = original_mode;
            SetPreset(m_preset);
          }
          
          LiteImage::SaveImage<float4>(("saves/iter_"+std::to_string(iter)+"_"+std::to_string(image_id)+".png").c_str(), m_images[image_id]);

          if (preset.debug_render_mode != DR_DEBUG_RENDER_MODE_NONE)
            LiteImage::SaveImage<float4>(("saves/debug_iter_"+std::to_string(iter)+"_"+std::to_string(image_id)+".png").c_str(), m_imagesDebug[image_id]);

        }
      }

      if  (m_preset_dr.debug_pd_images && preset.dr_diff_mode == DR_DIFF_MODE_DEFAULT)
      {
        for (int i = 0; i < params_count; i++)
        {
          for (int j = 0; j < m_width * m_height; j++)
          {
            float l = m_imagesDebugPD[i].data()[j].x;
            m_imagesDebugPD[i].data()[j] = float4(std::max(0.0f, l/20), std::max(0.0f, -l/20),0,1);
          }
          LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"a.png").c_str(), m_imagesDebugPD[i]);
        }
      }
    }

    if (preset.debug_print)
      printf("\n");
    sbs.values_f = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF;
  }

  float MultiRendererDR::RenderDR(const float4 *image_ref, LiteMath::float4 *out_image,
                                  float *out_dLoss_dS, unsigned params_count, 
                                  LiteMath::float4* out_image_depth, LiteMath::float4* out_image_debug)
  {
    bool use_multithreading = !(m_preset_dr.debug_border_samples || 
                                m_preset_dr.debug_pd_images ||
                                m_preset_dr.debug_border_samples_mega_image);

    unsigned max_threads = use_multithreading ? omp_get_max_threads() : 1;
    unsigned steps = (m_width * m_height + max_threads - 1)/max_threads;
    std::vector<double> loss_v(max_threads, 0.0f);
    //std::vector<float> dloss_1(params_count, 0.0f);
    //std::vector<float> dloss_2(params_count, 0.0f);

    omp_set_num_threads(max_threads);

    //I - render image, calculate internal derivatives
    #pragma omp parallel for
    for (int thread_id = 0; thread_id < max_threads; thread_id++)
    {
      unsigned start = thread_id * steps;
      unsigned end = std::min((thread_id + 1) * steps, m_width * m_height);
      for (int i = start; i < end; i++)
      {
        loss_v[thread_id] += CastRayWithGrad(i, image_ref, out_image, out_dLoss_dS + (params_count * thread_id), 
                                            out_image_depth, out_image_debug, m_PD_tmp.data() + 2*8*m_preset_dr.spp * thread_id);
      }
    }

    //II - find border pixels
    m_borderPixels.resize(0);
    const int search_radius = 1;

    if (m_preset_dr.dr_diff_mode == DR_DIFF_MODE_DEFAULT)
    {
      for (int i = 0; i < m_width * m_height; i++)
      {
        if (i >= m_packedXY.size())
          continue;

        const uint XY = m_packedXY[i];
        const uint x  = (XY & 0x0000FFFF);
        const uint y  = (XY & 0xFFFF0000) >> 16;

        if (x < search_radius || x >= m_width - search_radius || y <= search_radius || y >= m_height - search_radius)
          continue;
        
        //finding external borders nearby (borders with background)
        //TODO: find internal borders
        float d0 = out_image[y*m_width + x].w;
        float max_diff = 0.0f;
        float  max_depth_thr = 0.0f;

        for (int dx = -search_radius; dx <= search_radius; dx++)
        {
          for (int dy = -search_radius; dy <= search_radius; dy++)
          {
            float d = out_image[(y + dy) * m_width + x + dx].w;

            max_diff      = std::max(max_diff,      std::abs(d0 - out_image[(y + dy) * m_width + x + dx].w));
            max_depth_thr = std::max(max_depth_thr, out_image_depth[(y + dy) * m_width + x + dx].w);
          }
        }

        bool is_border = false;
        switch (m_preset_dr.dr_render_mode)
        {
          case DR_RENDER_MODE_MASK:
            is_border = max_diff > 0;
          break;
          case DR_RENDER_MODE_LINEAR_DEPTH:
          case DR_RENDER_MODE_DIFFUSE:
          case DR_RENDER_MODE_LAMBERT:
            is_border = max_diff > 0 || max_depth_thr > 0;
          break;
          default:
            is_border = false;
          break;
        }

        if (is_border || m_preset_dr.debug_forced_border)
        {
          m_borderPixels.push_back(i);

          if (m_preset_dr.dr_raycasting_mask == DR_RENDER_MASK_CAST_OPT)
          {
            int y0 = std::max(0, (int)y - (int)add_border), y1 = std::min(m_height - 1, y + add_border);
            int x0 = std::max(0, (int)x - (int)add_border), x1 = std::min(m_width - 1, x + add_border);
            
            for (int h = y0; h < y1; ++h)
            {
              process_mask[m_width * h + x] = 1;
            }

            // for (int w = x0; w < x1; ++w)
            // {
            //   process_mask[m_width * y + w] = 1;
            // }

            std::fill(process_mask.begin() + x0, process_mask.begin() + x1, 1);
          }
        }
      }
    }

    if (m_preset_dr.debug_border_samples_mega_image)
    {
      unsigned wmult = MEGA_PIXEL_SIZE, hmult = MEGA_PIXEL_SIZE;
      unsigned sw = m_width * wmult, sh = m_height * hmult;
      for (int h = 0; h < m_height; h++)
      {
        for (int w = 0; w < m_width; w++)
        {
          for (int y = 0; y < hmult; y++)
            for (int x = 0; x < wmult; x++)
              samples_mega_image.data()[(h*hmult + y)*sw + w*wmult + x] = out_image[h*m_width + w];
        }
      }
    }

    //III - calculate border intergral on border pixels
    if (m_borderPixels.size() > 0 && m_preset_dr.dr_diff_mode == DR_DIFF_MODE_DEFAULT &&
        (m_preset_dr.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_GEOMETRY))
    {
      unsigned border_steps = (m_borderPixels.size() + max_threads - 1)/max_threads;

      if (m_preset_dr.dr_border_sampling == DR_BORDER_SAMPLING_RANDOM)
      {
        #pragma omp parallel for
        for (int thread_id = 0; thread_id < max_threads; thread_id++)
        {
          unsigned start = thread_id * border_steps;
          unsigned end = std::min((thread_id + 1) * border_steps, (unsigned)m_borderPixels.size());
          for (int i = start; i < end; i++)
            CastBorderRay(m_borderPixels[i], image_ref, out_image, out_dLoss_dS + (params_count * thread_id), out_image_debug);
        }
      }
      else if (m_preset_dr.dr_border_sampling == DR_BORDER_SAMPLING_SVM)
      {
        #pragma omp parallel for
        for (int thread_id = 0; thread_id < max_threads; thread_id++)
        {
          unsigned start = thread_id * border_steps;
          unsigned end = std::min((thread_id + 1) * border_steps, (unsigned)m_borderPixels.size());
          for (int i = start; i < end; i++)
            CastBorderRaySVM(m_borderPixels[i], image_ref, out_image, out_dLoss_dS + (params_count * thread_id), out_image_debug);
        }        
      }
    }
    if (m_preset_dr.debug_border_samples_mega_image)
    {
      for (int h = 0; h < samples_mega_image.width()*samples_mega_image.height(); h++)
        samples_mega_image.data()[h].w = 1;
      LiteImage::SaveImage<float4>("saves/debug_mega_image.png", samples_mega_image);
    }

    //IV - calculate loss
    double loss = 0.0f;
    for (auto &l : loss_v)
      loss += l;

    omp_set_num_threads(omp_get_max_threads());

    return loss/(m_width * m_height);
  }

  float MultiRendererDR::RenderDRFiniteDiff(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                                            unsigned start_index, unsigned end_index, float delta)
  {
    assert(end_index > start_index);
    assert(end_index <= params_count);
    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();

    std::vector<float> m_dLoss_dS_tmp_2(m_dLoss_dS_tmp.size(), 0.0f);

    LiteImage::Image2D<float4> image_1(m_width, m_height, float4(0,0,0,1)), image_2(m_width, m_height, float4(0,0,0,1));

    for (unsigned i = start_index; i < end_index; i++)
    {
      float p0 = params[i];
      double loss_plus = 0.0f;
      double loss_minus = 0.0f;

      params[i] = p0 + delta;
      //RenderFloat(out_image, m_width, m_height, "color");
      RenderDR(image_ref, out_image, m_dLoss_dS_tmp_2.data(), params_count, nullptr, nullptr);
      for (int j = 0; j < m_width * m_height; j++)
      {
        float l = Loss(m_preset_dr.dr_loss_function, out_image[j], image_ref[j]);
        image_1.data()[j] = out_image[j];//float4(l,l,0,1);
        loss_plus += l;
      }
    
      params[i] = p0 - delta;
      //RenderFloat(out_image, m_width, m_height, "color");
      RenderDR(image_ref, out_image, m_dLoss_dS_tmp_2.data(), params_count, nullptr, nullptr);
      for (int j = 0; j < m_width * m_height; j++)
      {
        float l = Loss(m_preset_dr.dr_loss_function, out_image[j], image_ref[j]);
        image_2.data()[j] = out_image[j];//float4(l,l,0,1);
        loss_minus += l;
      }
      
      params[i] = p0;

      //it is the same way as loss is accumulated in RenderDR
      //loss_plus /= m_width * m_height;
      //loss_minus /= m_width * m_height;
      //printf("loss_plus = %f, loss_minus = %f\n", loss_plus, loss_minus);

      if (m_preset_dr.debug_pd_images)
      {
        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"_1.png").c_str(), image_1);
        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"_2.png").c_str(), image_2);

        for (int j = 0; j < m_width * m_height; j++)
        {
          float l1 = Loss(m_preset_dr.dr_loss_function, image_1.data()[j], image_ref[j]);
          float l2 = Loss(m_preset_dr.dr_loss_function, image_2.data()[j], image_ref[j]);
          float l = (l1 - l2) / (2 * delta);
          image_1.data()[j] = float4(std::max(0.0f, l/20), std::max(0.0f, -l/20),0,1);
        }

        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"b.png").c_str(), image_1);
      }

      out_dLoss_dS[i] = (loss_plus - loss_minus) / (2 * delta);
    }

    double loss = 0.0f;
    //RenderFloat(out_image, m_width, m_height, "color");
    for (int j = 0; j < m_width * m_height; j++)
      loss += Loss(m_preset_dr.dr_loss_function, out_image[j], image_ref[j]);
    
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

  float3 MultiRendererDR::CalculateColor(const CRT_HitDR &hit)
  {
    float3 color = float3(1, 0, 1);

    switch (m_preset_dr.dr_render_mode)
    {
    case DR_RENDER_MODE_DIFFUSE:
      color = hit.color;
      break;
    case DR_RENDER_MODE_LAMBERT:
    {
      float3 light_dir = normalize(float3(1, 1, 1));
      float3 norm = hit.normal;
      float q0 = dot(norm, light_dir); // = norm.x*light_dir.x + norm.y*light_dir.y + norm.z*light_dir.z
      float q = max(0.1f, q0);
      color = q * hit.color;
    }
    break;
    case DR_RENDER_MODE_MASK:
      color = float3(1, 1, 1);
      break;
    case DR_RENDER_MODE_LINEAR_DEPTH:
    {
      float d = absolute_to_linear_depth(hit.t);
      color = float3(d, d, d);
    }
    break;
    default:
      break;
    }

    return color;
  }

  float3 MultiRendererDR::CalculateColorWithGrad(const CRT_HitDR &hit,
                                                 LiteMath::float3x3 &dColor_dDiffuse,
                                                 LiteMath::float3x3 &dColor_dNorm)
  {
    float3 color = float3(1, 0, 1);

    switch (m_preset_dr.dr_render_mode)
    {
    case DR_RENDER_MODE_DIFFUSE:
      color = hit.color;

      dColor_dDiffuse = LiteMath::make_float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
      dColor_dNorm = LiteMath::make_float3x3(float3(0, 0, 0), float3(0, 0, 0), float3(0, 0, 0));
      break;
    case DR_RENDER_MODE_LAMBERT:
    {
      float3 light_dir = normalize(float3(1, 1, 1));
      float3 norm = hit.normal;
      float q0 = dot(norm, light_dir); // = norm.x*light_dir.x + norm.y*light_dir.y + norm.z*light_dir.z
      float q = max(0.1f, q0);
      color = q * hit.color;

      dColor_dDiffuse = LiteMath::make_float3x3(float3(q, 0, 0), float3(0, q, 0), float3(0, 0, q));
      if (q0 > 0.1f)
      {
        float3 dq_dnorm = light_dir;
        dColor_dNorm = LiteMath::make_float3x3(hit.color.x * light_dir,
                                               hit.color.y * light_dir,
                                               hit.color.z * light_dir);
      }
      else
        dColor_dNorm = LiteMath::make_float3x3(float3(0, 0, 0), float3(0, 0, 0), float3(0, 0, 0));
    }
    break;
    case DR_RENDER_MODE_MASK:
      color = float3(1, 1, 1);

      dColor_dDiffuse = LiteMath::make_float3x3(float3(0, 0, 0), float3(0, 0, 0), float3(0, 0, 0));
      dColor_dNorm = LiteMath::make_float3x3(float3(0, 0, 0), float3(0, 0, 0), float3(0, 0, 0));
      break;
    case DR_RENDER_MODE_LINEAR_DEPTH:
    {
      float d = absolute_to_linear_depth(hit.t);
      color = float3(d, d, d);
    }
    break;
    default:
      break;
    }

    return color;
  }

  float3 MultiRendererDR::ApplyDebugColor(float3 original_color, const CRT_HitDR &hit)
  {
    float3 color = original_color;
    switch (m_preset_dr.debug_render_mode)
    {
      case DR_DEBUG_RENDER_MODE_PRIMITIVE:
        color = to_float3(decode_RGBA8(m_palette[(hit.primId) % palette_size]));
      break;
      default:
      break;
    }

    return color;
  }

  // if returned threshold <= 0, it indicates smooth surface without depth discontinuity
  float t_samples_to_threshold(const float *samples, unsigned count, float default_value, unsigned default_count)
  {
    constexpr int GISTO_SIZE = 10;
    constexpr int GISTO_DISC_GAP = 4;

    float depth_min = samples[0];
    float depth_max = samples[0];
    for (unsigned i = 1; i < count; i++)
    {
      depth_min = std::min(depth_min, samples[i]);
      depth_max = std::max(depth_max, samples[i]);
    }

    if (default_count > 0)
    {
      depth_min = std::min(depth_min, default_value);
      depth_max = std::max(depth_max, default_value);
    }
  
    float depth_diff = std::max(1e-9f, depth_max - depth_min);

    unsigned gisto[GISTO_SIZE] = {0};
    for (int i = 0; i < count; i++)
    {
      float d = (samples[i] - depth_min) / depth_diff;
      gisto[std::clamp(int(d * GISTO_SIZE), 0, GISTO_SIZE - 1)]++;
    }

    // add default values to the histogram
    {
      float default_d = (default_value - depth_min) / depth_diff;
      gisto[std::clamp(int(default_d * GISTO_SIZE), 0, GISTO_SIZE - 1)] += default_count;
    }

    // find the longest gap in the histogram (gisto[0] and gisto[GISTO_SIZE-1] are always >0)
    // if it long enough, we assume that there is depth discontinuity in this pixel
    // we save assumed discontinuity threshold and gap length
    int max_gap_length = 0;
    int max_gap_start = -1;
    int gap_start = -1;
    for (int i = 1; i < GISTO_SIZE; i++)
    {
      if (gap_start != -1)
      {
        if (gisto[i] != 0) // gap end
        {
          int gap_length = i - gap_start;
          if (gap_length > max_gap_length)
          {
            max_gap_length = gap_length;
            max_gap_start = gap_start;
          }
          gap_start = -1;
        }
      }
      else
      {
        if (gisto[i] == 0) // gap start
          gap_start = i;
      }
    }

    if (max_gap_length > GISTO_DISC_GAP)
      return depth_min + depth_diff * (max_gap_start + 0.5f * max_gap_length) / GISTO_SIZE;
    else
      return 0.0f;
  }

  // bool 
  // MultiRendererDR::hasNearIntersection(const int &x, const int &y) const
  // {
  //   bool has_near = false;
  //   int i0 = std::max(y - add_border, 0), i1 = std::min(y + add_border, (int)m_height);
  //   int j0 = std::max(x - add_border, 0), j1 = std::min(x + add_border, (int)m_width);

  //   #pragma omp parallel for reduction(+:has_near)
  //   for (int i = i0; i < i1; ++i)
  //   {
  //     for (int j = j0; j < j1; ++j)
  //     {
  //       has_near = has_near || mask[i * m_width + j]; 
  //     }
  //   }

  //   return has_near;
  // }

  float MultiRendererDR::CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4 *out_image, float *out_dLoss_dS,
                                         LiteMath::float4* out_image_depth, LiteMath::float4* out_image_debug, PDFinalColor *out_pd_tmp)
  {
    if (tidX >= m_packedXY.size())
      return 0.0;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;

    if (m_preset_dr.dr_raycasting_mask == DR_RENDER_MASK_CAST_OPT && !masks[mask_ind][y * m_width + x])
    {
      return 0.f;
    }
    
    float3 res_color = float3(0,0,0);
    float3 debug_color = float3(0,0,0);
    float res_loss = 0.0f;
    int hit_count = 0;
    std::vector<float> samples(m_preset.spp, 0.0f);

    uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
    float i_spp_sqrt = 1.0f/spp_sqrt;
    uint32_t color_pd_offset = 8*m_preset.spp;
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);

    uint32_t ray_flags;
    if (m_preset_dr.dr_diff_mode == DR_DIFF_MODE_FINITE_DIFF)
    {
      ray_flags = DR_RAY_FLAG_NO_DIFF;
    }
    else if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_LINEAR_DEPTH)
    {
      ray_flags = DR_RAY_FLAG_DDIST_DPOS;
    }
    else
    {
      ray_flags = DR_RAY_FLAG_NO_DIFF;

      if (m_preset_dr.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_COLOR)
      {
        ray_flags |= DR_RAY_FLAG_DDIFFUSE_DCOLOR;
      }
      if (m_preset_dr.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_GEOMETRY)
      {
        if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_MASK)
          ray_flags |= DR_RAY_FLAG_NO_DIFF;
        else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_DIFFUSE)
          ray_flags |= DR_RAY_FLAG_DDIFFUSE_DPOS;
        else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_LAMBERT)
          ray_flags |= DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS;
      }
    }

    float total_diff = 0;
    for (uint32_t i = 0; i < m_preset.spp; i++)
    {
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
      
      float3 color = float3(0,0,0);
      float3 dLoss_dColor = float3(0,0,0);
      LiteMath::float3x3 dColor_dDiffuse = LiteMath::make_float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
      LiteMath::float3x3 dColor_dNorm    = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
      
      RayDiffPayload payload;
      CRT_HitDR hit = ((BVHDR*)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(ray_flags, rayPosAndNear, rayDirAndFar, &payload);
      
      if (hit.primId == 0xFFFFFFFF) //no hit
      {
        color = background_color;
      }
      else
      {
        if (m_preset_dr.dr_raycasting_mask == DR_RENDER_MASK_CAST_OPT)
        {
          process_mask[y * m_width + x] = 1;
        }
        
        color = CalculateColorWithGrad(hit, dColor_dDiffuse, dColor_dNorm);

        if (m_preset_dr.debug_render_mode != DR_DEBUG_RENDER_MODE_NONE)
          debug_color += ApplyDebugColor(color, hit) / m_preset.spp;

        res_color += color / m_preset.spp;

        //fill buffer with final color partial derivatives w.r.t. voxel color
        if (ray_flags & DR_RAY_FLAG_DDIFFUSE_DCOLOR)
        {
          for (int i = 0; i < 8; i++)
          {
            float dDiffuse_dSc = payload.dDiffuse_dSc[i].value;
            out_pd_tmp[color_pd_offset + 8*hit_count + i].index = payload.dDiffuse_dSc[i].index;
            out_pd_tmp[color_pd_offset + 8*hit_count + i].dFinalColor = dColor_dDiffuse * float3(dDiffuse_dSc);
          }
        }

        //fill buffer with final color partial derivatives w.r.t. voxel distances
        //in default mode it is done via dDiffuse and dNorm partial derivatives
        //when input data is depth it is done via dDist partial derivative
        if (ray_flags & (DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS))
        {
          for (int i = 0; i < 8; i++)
          {
            PDDist &pd = payload.dDiffuseNormal_dSd[i];
            out_pd_tmp[8*hit_count + i].index = pd.index;
            out_pd_tmp[8*hit_count + i].dFinalColor = dColor_dDiffuse*pd.dDiffuse + dColor_dNorm*pd.dNorm;
          }
        }
        else if (ray_flags & DR_RAY_FLAG_DDIST_DPOS)
        {
          for (int i = 0; i < 8; i++)
          {
            PDDist &pd = payload.dDiffuseNormal_dSd[i];
            out_pd_tmp[8*hit_count + i].index = pd.index;
            out_pd_tmp[8*hit_count + i].dFinalColor = float3(pd.dDist) / (z_far - z_near);
          }
        }

        samples[hit_count] = hit.t;
        hit_count++;
      }
    }

    float4 color_mask = float4(res_color.x, res_color.y, res_color.z, (float)hit_count/m_preset.spp);
    out_image[y * m_width + x] = float4(res_color.x, res_color.y, res_color.z, (float)hit_count/m_preset.spp);

    if (out_image_depth)
    {
      if (hit_count == 0)
      {
        out_image_depth[y * m_width + x] = float4(0, 0, 0, 0);
      }
      else
      {
        float depth_thr = t_samples_to_threshold(samples.data(), hit_count, 1e9f, m_preset.spp - hit_count);
        out_image_depth[y * m_width + x] = float4(0, 0, 0, depth_thr);
      }
    }

    res_loss = Loss(m_preset_dr.dr_loss_function, color_mask, image_ref[y * m_width + x]);
    float3 dLoss_dColor = to_float3(LossGrad(m_preset_dr.dr_loss_function, color_mask, image_ref[y * m_width + x]));
    float ref_mask = 1.0f;//image_ref[y * m_width + x].w;
    
    dLoss_dColor = dLoss_dColor * ref_mask / m_preset.spp;

    if (ray_flags & DR_RAY_FLAG_DDIFFUSE_DCOLOR)
    {
      for (int i = color_pd_offset; i < color_pd_offset + 8 * hit_count; i++)
      {
        float3 diff = dLoss_dColor * out_pd_tmp[i].dFinalColor;
        out_dLoss_dS[out_pd_tmp[i].index + 0] += diff.x;
        out_dLoss_dS[out_pd_tmp[i].index + 1] += diff.y;
        out_dLoss_dS[out_pd_tmp[i].index + 2] += diff.z;

        if (m_preset_dr.debug_pd_images)
        {
          m_imagesDebugPD[out_pd_tmp[i].index + 0].data()[y * m_width + x] += diff.x * float4(1, 1, 1, 0);
          m_imagesDebugPD[out_pd_tmp[i].index + 1].data()[y * m_width + x] += diff.y * float4(1, 1, 1, 0);
          m_imagesDebugPD[out_pd_tmp[i].index + 2].data()[y * m_width + x] += diff.z * float4(1, 1, 1, 0);
        }
      }
    }

    if (ray_flags & (DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS | DR_RAY_FLAG_DDIST_DPOS))
    {
      for (int i = 0; i < 8 * hit_count; i++)
      {
        float diff = dot(dLoss_dColor, out_pd_tmp[i].dFinalColor);
        out_dLoss_dS[out_pd_tmp[i].index] += diff;
        total_diff += diff;

        if (m_preset_dr.debug_pd_images)
          m_imagesDebugPD[out_pd_tmp[i].index].data()[y * m_width + x] += diff * float4(1, 1, 1, 0);
      }
    }

    if (out_image_debug)
    {
      if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_AREA_INTEGRAL)
        out_image_debug[y * m_width + x] = to_float4(visualize_value_debug(total_diff), 1.0f);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_PRIMITIVE)
        out_image_debug[y * m_width + x] = to_float4(debug_color, 1.0f);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_DEPTH_STAT && out_image_depth)
      {
        out_image_debug[y * m_width + x] = float4(absolute_to_linear_depth(out_image_depth[y * m_width + x].x), 
                                                  absolute_to_linear_depth(out_image_depth[y * m_width + x].y), 
                                                  absolute_to_linear_depth(out_image_depth[y * m_width + x].z), 
                                                  1.0f);
      }
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_DEPTH_DIFF && out_image_depth)
      {
        float diff = out_image_depth[y * m_width + x].z - out_image_depth[y * m_width + x].y;
        out_image_debug[y * m_width + x] = float4(out_image_depth[y * m_width + x].w, 0, 0, 1.0f);
      }
    }

    return res_loss;
  }

  void MultiRendererDR::CastBorderRay(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS,
                                      LiteMath::float4* out_image_debug)
  {
    if (tidX >= m_packedXY.size())
      return;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;

    const unsigned border_ray_flags = DR_RAY_FLAG_BORDER;
    const float relax_eps = m_preset_dr.border_relax_eps;
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);
    const float  background_depth = 0.0f;

    const unsigned border_spp = m_preset_dr.border_spp;
    const uint32_t spp_sqrt = uint32_t(sqrt(border_spp));
    const float i_spp_sqrt = 1.0f / spp_sqrt;
    const float4 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, out_image[y * m_width + x], image_ref[y * m_width + x]);
    
    unsigned border_points = 0;
    float total_diff = 0.0f;

    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      samples_debug_color.resize(border_spp, float4(0,0,0,0));
      samples_debug_pos_size.resize(border_spp, float4(0,0,0,0));
    }

    for (int sample_id = 0; sample_id < border_spp; sample_id++)
    {
      float pixel_diff = 0.0f;
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, sample_id) : i_spp_sqrt * float2(sample_id / spp_sqrt + 0.5, sample_id % spp_sqrt + 0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);

      RayDiffPayload payload;
      CRT_HitDR hit = ((BVHDR *)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(border_ray_flags, rayPosAndNear, rayDirAndFar, &payload);

      bool is_border_ray = payload.missed_hit.sdf < relax_eps;

      if (is_border_ray)
      {
        border_points++;

        float4 color_delta = float4(0,0,0,0);
        if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_COLOR)
        {
          float3 f_in = CalculateColor(payload.missed_hit);
          float3 f_out = hit.primId == 0xFFFFFFFF ? background_color : CalculateColor(hit);
          color_delta = to_float4(f_in - f_out, hit.primId == 0xFFFFFFFF ? 1.0f : 0.0f);
        }
        else if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_LINEAR_DEPTH)
        {
          float d_in = payload.missed_hit.t;
          float d_out = hit.primId == 0xFFFFFFFF ? background_depth : hit.t;
          color_delta = float4(d_in - d_out, d_in - d_out, d_in - d_out, hit.primId == 0xFFFFFFFF ? 1.0f : 0.0f);
        }

        for (int j = 0; j < 8; j++)
        {
          if (payload.missed_indices[j] == INVALID_INDEX)
            continue;
          float diff = m_preset_dr.border_integral_mult * (1.0f / border_spp) *dot(dLoss_dColor, (1.0f / relax_eps) * payload.missed_dSDF_dtheta[j] * color_delta);
          out_dLoss_dS[payload.missed_indices[j]] += diff;

          total_diff += abs(diff);
          pixel_diff += length((1.0f / relax_eps) * payload.missed_dSDF_dtheta[j] * color_delta);
          if (m_preset_dr.debug_pd_images)
            m_imagesDebugPD[payload.missed_indices[j]].data()[y * m_width + x] += diff * float4(1, 1, 1, 0);
        }
      }


      if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
      {
        samples_debug_pos_size[sample_id] = float4(d.x, d.y, 0, 1.0f/MEGA_PIXEL_SIZE);
        if (is_border_ray) //border
          samples_debug_color[sample_id] = float4(0, 0, 0.1*abs(pixel_diff), 1);
        else if (hit.primId == 0xFFFFFFFF) //background
        {
#ifdef DEBUG_PAYLOAD_STORE_SDF
          bool neg_flag = false;
          for (auto sdf : payload.sdf_i)
          {
            if (sdf < 0) // or any other condition
            {
              neg_flag = true;
              break;
            }
          }
          if (true)
          {
            printf("Pts %ld:\n", payload.sdf_i.size());
            for (int i = 0; i < payload.sdf_i.size(); ++i) {
              printf("%f, ", payload.sdf_i[i]);
              if ((i % 11) == 10)
                printf("\n");
            }
          }
#endif
          samples_debug_color[sample_id] = float4(0.05, 0.05, 0.05, 1);
        }
        else //hit
          samples_debug_color[sample_id] = float4(1, 0, 0, 1);
      }
    }

    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      unsigned wmult = MEGA_PIXEL_SIZE, hmult = MEGA_PIXEL_SIZE;
      unsigned sw = m_width * wmult, sh = m_height * hmult;
      LiteImage::Image2D<float4> debug_image(wmult, hmult);
      draw_points(samples_debug_pos_size, samples_debug_color, debug_image);
      if (tidX % 10 == 0)
        LiteImage::SaveImage(("saves/debug_border"+std::to_string(x)+"_"+std::to_string(y)+".png").c_str(), debug_image);
      
      if (m_preset_dr.debug_border_samples_mega_image)
      {
        for (int dy = 0; dy < hmult; dy++)
          for (int dx = 0; dx < wmult; dx++)
            samples_mega_image.data()[(y*hmult + dy) * sw + (x*wmult + dx)] = debug_image.data()[dy * wmult + dx];
      }
    }

    if (out_image_debug)
    {
      if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_DETECTION)
        out_image_debug[y * m_width + x] = float4(1, 1, 1, 1);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_FOUND)
        out_image_debug[y * m_width + x] = border_points > 0 ? float4(1, 1, 1, 1) : float4(0, 0, 0, 1);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL)
        out_image_debug[y * m_width + x] = to_float4(visualize_value_debug(total_diff), 1.0f);
    }
  }

  // This is an implementation of the SVM classification algorithm
  // Note that it works only for binary classification (-1, +1)

  // ######################   PARAMETERS    ######################
  // X_mod: vector<float3>, will be modified in this function
  // training data in the form (x_1*y, x_2*y, y)

  // etha: float(default - 0.01)
  //     Learning rate, gradient step

  // alpha: float, (default - 0.1)
  //     Regularization parameter in 0.5*alpha*||w||^2

  // epochs: int, (default - 200)
  //     Number of epochs of training

  //return W = (a, b, c, error_rate) division line ax + by + c = 0 and error rate
  float4 SVM_fit(std::vector<float3>& X_mod, float etha = 0.01f, float alpha = 0.1f, int epochs = 200)
  {
    const unsigned N = X_mod.size();

    //X_mod = X_mod * etha
    for (int i = 0; i < N; i++)
      X_mod[i] = X_mod[i] * etha;
    
    uint32_t errors = N;
    float3 W = float3(-1,1,0); //y = x
    const float weightMult = 1.0f - etha * alpha / epochs;
    for (int i = 0; i < epochs && errors > 0; i++)
    {
      errors = 0;
      for (int j = 0; j < N; j++)
      {
        float margin = dot(W, X_mod[j]);
        W = W*weightMult;
        if (margin < 0)
        {
          W = W + X_mod[j];
          errors++;
        }
      }
    }
    //printf("errors %d, W= %f %f %f\n", errors, W.x, W.y, W.z);

    return float4(W.x, W.y, W.z, float(errors) / float(N));
  }

  // Cohen–Sutherland algorithm for fast line clipping
  // https://en.wikipedia.org/wiki/Cohen%E2%80%93Sutherland_algorithm
  // clipping plane is [0,0] - [1,1]
  uint32_t ComputeOutCode(float x, float y)
  {
    constexpr uint32_t INSIDE = 0b0000;
    constexpr uint32_t LEFT   = 0b0001;
    constexpr uint32_t RIGHT  = 0b0010;
    constexpr uint32_t BOTTOM = 0b0100;
    constexpr uint32_t TOP    = 0b1000;
    constexpr uint32_t xmin   = 0;
    constexpr uint32_t xmax   = 1;
    constexpr uint32_t ymin   = 0;
    constexpr uint32_t ymax   = 1;

    uint32_t code = INSIDE;  // initialised as being inside of clip window

    if (x < xmin)           // to the left of clip window
      code |= LEFT;
    else if (x > xmax)      // to the right of clip window
      code |= RIGHT;
    if (y < ymin)           // below the clip window
      code |= BOTTOM;
    else if (y > ymax)      // above the clip window
      code |= TOP;

    return code;
  }

  // Cohen–Sutherland clipping algorithm clips a line from
  // P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with 
  // diagonal from (xmin, ymin) to (xmax, ymax).
  bool CohenSutherlandLineClip(float& x0, float& y0, float& x1, float& y1)
  {
    constexpr uint32_t INSIDE = 0b0000;
    constexpr uint32_t LEFT   = 0b0001;
    constexpr uint32_t RIGHT  = 0b0010;
    constexpr uint32_t BOTTOM = 0b0100;
    constexpr uint32_t TOP    = 0b1000;
    constexpr uint32_t xmin   = 0;
    constexpr uint32_t xmax   = 1;
    constexpr uint32_t ymin   = 0;
    constexpr uint32_t ymax   = 1;

    // compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
    uint32_t outcode0 = ComputeOutCode(x0, y0);
    uint32_t outcode1 = ComputeOutCode(x1, y1);
    bool accept = false;

    while (true) {
      if (!(outcode0 | outcode1)) {
        // bitwise OR is 0: both points inside window; trivially accept and exit loop
        accept = true;
        break;
      } else if (outcode0 & outcode1) {
        // bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
        // or BOTTOM), so both must be outside window; exit loop (accept is false)
        break;
      } else {
        // failed both tests, so calculate the line segment to clip
        // from an outside point to an intersection with clip edge
        float x=-1, y=-1;

        // At least one endpoint is outside the clip rectangle; pick it.
        uint32_t outcodeOut = outcode1 > outcode0 ? outcode1 : outcode0;

        // Now find the intersection point;
        // use formulas:
        //   slope = (y1 - y0) / (x1 - x0)
        //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
        //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
        // No need to worry about divide-by-zero because, in each case, the
        // outcode bit being tested guarantees the denominator is non-zero
        if (outcodeOut & TOP) {           // point is above the clip window
          x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
          y = ymax;
        } else if (outcodeOut & BOTTOM) { // point is below the clip window
          x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
          y = ymin;
        } else if (outcodeOut & RIGHT) {  // point is to the right of clip window
          y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
          x = xmax;
        } else if (outcodeOut & LEFT) {   // point is to the left of clip window
          y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
          x = xmin;
        }

        // Now we move outside point to intersection point to clip
        // and get ready for next pass.
        if (outcodeOut == outcode0) {
          x0 = x;
          y0 = y;
          outcode0 = ComputeOutCode(x0, y0);
        } else {
          x1 = x;
          y1 = y;
          outcode1 = ComputeOutCode(x1, y1);
        }
      }
    }
    return accept;
  }

  void MultiRendererDR::CastBorderRaySVM(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS,
                                         LiteMath::float4* out_image_debug)
  {
    const float min_sample_radius = 0.1f;
    const float sample_radius_mult = 2.0f;
    const float max_error_rate = 0.1f;
    const float min_stripe_area = 0.005f;
    const float svm_samples_budget = 0.25f;

    if (tidX >= m_packedXY.size())
      return;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;

    const unsigned border_ray_flags = DR_RAY_FLAG_BORDER;
    const float relax_eps = m_preset_dr.border_relax_eps;
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);
    const float  background_depth = 0.0f;

    const unsigned svm_spp = std::max(1u, (unsigned)(svm_samples_budget*m_preset_dr.border_spp));
    const float4 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, out_image[y * m_width + x], image_ref[y * m_width + x]);
    
    float total_diff = 0.0f;
    unsigned border_points = 0;

    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      samples_debug_color.resize(m_preset_dr.border_spp, float4(0,0,0,0));
      samples_debug_pos_size.resize(m_preset_dr.border_spp, float4(0,0,0,0));
    }

    //step 1: cast some rays to provide data for SVM

    std::vector<float3> X_mod(svm_spp);
    std::vector<float> t_samples(svm_spp);
    for (int sample_id = 0; sample_id < svm_spp; sample_id++)
    {
      float2 d = rand2(x, y, sample_id);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);

      RayDiffPayload payload; //TODO remove it, as we dont want calculate any differences here
      CRT_HitDR hit = ((BVHDR *)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(DR_RAY_FLAG_NO_DIFF, rayPosAndNear, rayDirAndFar, &payload);

      X_mod[sample_id] = float3(d.x, d.y, 1);
      t_samples[sample_id] = hit.t;
    }

    const float t_threshold = t_samples_to_threshold(t_samples.data(), svm_spp, 0, 0);
    unsigned type_0_samples_count = 0;
    for (int sample_id = 0; sample_id < svm_spp; sample_id++)
    {
      float t = t_samples[sample_id];
      float y = t <= t_threshold ? -1 : 1;
      type_0_samples_count += (t <= t_threshold ? 1 : 0);
      X_mod[sample_id] *= y;

      if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
      {
        samples_debug_pos_size[sample_id] = float4(std::abs(X_mod[sample_id].x), std::abs(X_mod[sample_id].y), 0, 1.0f/MEGA_PIXEL_SIZE);
        if (t_samples[sample_id] >= 1e6f) //background
          samples_debug_color[sample_id] = float4(0.05, 0.05, 0.05, 1);
        else //hit
          samples_debug_color[sample_id] = float4(t_samples[sample_id] <= t_threshold ? 0 : 1, t_samples[sample_id] <= t_threshold ? 1 : 0, 0, 1);
      }
    }

    //if all samples are of one type, return
    if (type_0_samples_count == svm_spp || type_0_samples_count == 0)
      return;

    //step 2: train SVM
    //printf("x, y = %u %u\n", x, y);
    float4 W = SVM_fit(X_mod);
    float error_rate = W.w;

    float2 p0, p1, r, n;
    float sample_radius, stripe_area;
    bool force_random_ray_cast = false;
    //if error rate is too high, force random ray cast
    if (error_rate > max_error_rate)
    {
      force_random_ray_cast = true;
      stripe_area = 1.0f;
    }
    else
    {
      if (abs(W.y) > 1e-6f && abs(W.x) > 1e-6f)
      {
        p0 = float2(0, -W.z / W.y);
        p1 = float2(1, (-W.z - W.x) / W.y);
        float ry = p1.y - p0.y;
        //printf("BEFORE p0 = %f %f, p1 = %f %f,r = %f %f, n = %f %f\n", p0.x, p0.y, p1.x, p1.y, r.x, r.y, n.x, n.y);

        if (p0.y < -1e-6f || p0.y > 1+1e-6f)
        {
          float k1 = -p0.y/ry;
          float k2 = (1-p0.y)/ry;
          float k = (abs(k1) < abs(k2)) ? k1 : k2;
          p0 = p0 + k*float2(1, ry);
        }

        if (p1.y < -1e-6f || p1.y > 1+1e-6f)
        {
          float k1 = -p1.y/ry;
          float k2 = (1-p1.y)/ry;
          float k = (abs(k1) < abs(k2)) ? k1 : k2;
          p1 = p1 + k*float2(1, ry);
        }
      }
      else if (abs(W.x) > 1e-6f)
      {
        p0 = float2(-W.z / W.x, 0);
        p1 = float2(-W.z / W.x, 1);
      }
      else
      {
        force_random_ray_cast = true;
      }

      //force_random_ray_cast = true;
          
      bool line_clipped = CohenSutherlandLineClip(p0.x, p0.y, p1.x, p1.y);

      //line is ouside the pixel border
      if (!line_clipped)
        return;

      r = p1 - p0;
      n = float2(-r.y, r.x);
      sample_radius = min_sample_radius + sample_radius_mult*error_rate;
      stripe_area = std::min(1.0f, 2*sample_radius*length(r));

      //if stripe area is too small, there is no actual border here
      if (!force_random_ray_cast && stripe_area < min_stripe_area)
        return;
    }
    
    //number of border samples is proportional to stripe area
    float area_mult = std::min(1.0f, 0.05f + 2.0f*stripe_area);
    const unsigned border_spp = std::max(1u, (unsigned)(area_mult*(m_preset_dr.border_spp - svm_spp)));

    //step 3: cast rays alongside border
    for (int sample_id = svm_spp; sample_id < svm_spp + border_spp; sample_id++)
    {
      float pixel_diff = 0.0f;
      float2 d = float2(1000,1000);
      float sampling_pdf;
      if (force_random_ray_cast)
      {
        d = rand2(x, y, sample_id);
        sampling_pdf = 1.0f / border_spp;
      }
      else
      {
        while (d.x < 0 || d.x > 1 || d.y < 0 || d.y > 1)
        {
          d = p0 + urand(0, 1) * r + urand(-1, 1) * sample_radius * n;
          //printf("p0 = %f %f, p1 = %f %f,r = %f %f, n = %f %f, d = %f %f\n", p0.x, p0.y, p1.x, p1.x, r.x, r.y, n.x, n.y, d.x, d.y);
        }
        sampling_pdf = stripe_area / border_spp;
      }

      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);

      RayDiffPayload payload;
      CRT_HitDR hit = ((BVHDR *)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(border_ray_flags, rayPosAndNear, rayDirAndFar, &payload);

      bool is_border_ray = payload.missed_hit.sdf < relax_eps;

      if (is_border_ray)
      {
        border_points++;

        float4 color_delta = float4(0,0,0,0);
        if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_COLOR)
        {
          float3 f_in = CalculateColor(payload.missed_hit);
          float3 f_out = hit.primId == 0xFFFFFFFF ? background_color : CalculateColor(hit);
          color_delta = to_float4(f_in - f_out, hit.primId == 0xFFFFFFFF ? 1.0f : 0.0f);
        }
        else if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_LINEAR_DEPTH)
        {
          float d_in = payload.missed_hit.t;
          float d_out = hit.primId == 0xFFFFFFFF ? background_depth : hit.t;
          color_delta = float4(d_in - d_out, d_in - d_out, d_in - d_out, hit.primId == 0xFFFFFFFF ? 1.0f : 0.0f);
        }

        for (int j = 0; j < 8; j++)
        {
          float diff = m_preset_dr.border_integral_mult * sampling_pdf * dot(dLoss_dColor, (1.0f / relax_eps) * payload.missed_dSDF_dtheta[j] * color_delta);
          out_dLoss_dS[payload.missed_indices[j]] += diff;

          total_diff += abs(diff);
          pixel_diff += length((1.0f / relax_eps) * payload.missed_dSDF_dtheta[j] * color_delta);
          if (m_preset_dr.debug_pd_images)
            m_imagesDebugPD[payload.missed_indices[j]].data()[y * m_width + x] += diff * float4(1, 1, 1, 0);
        }
      }

      if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
      {
        samples_debug_pos_size[sample_id] = float4(d.x, d.y, 0, 1.0f / MEGA_PIXEL_SIZE);
        if (is_border_ray) //border
          samples_debug_color[sample_id] = float4(0, 0, 0.1 * abs(pixel_diff), 1);
        else if (hit.primId == 0xFFFFFFFF) // background
          samples_debug_color[sample_id] = float4(0.05, 0.05, 0.05, 1);
        else // hit
          samples_debug_color[sample_id] = float4(hit.t <= t_threshold ? 0 : 1, hit.t <= t_threshold ? 1 : 0, 0, 1);
      }
    }

    //debug stuff
    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      for (int sample_id = svm_spp + border_spp; sample_id < samples_debug_pos_size.size(); sample_id++)
      {
        samples_debug_pos_size[sample_id] = float4(0, 0, 0, 0);
        samples_debug_color[sample_id] = float4(0, 0, 0, 0);
      }
      unsigned wmult = MEGA_PIXEL_SIZE, hmult = MEGA_PIXEL_SIZE;
      unsigned sw = m_width * wmult, sh = m_height * hmult;
      LiteImage::Image2D<float4> debug_image(wmult, hmult, float4(0,0,0,1));
      draw_points(samples_debug_pos_size, samples_debug_color, debug_image);

      draw_line(p0, p1, 1.0f/MEGA_PIXEL_SIZE, float4(1,1,1,1), debug_image);
      //if (tidX % 10 == 0)
        LiteImage::SaveImage(("saves/debug_border"+std::to_string(x)+"_"+std::to_string(y)+".png").c_str(), debug_image);
      
      if (m_preset_dr.debug_border_samples_mega_image)
      {
        for (int dy = 0; dy < hmult; dy++)
          for (int dx = 0; dx < wmult; dx++)
            samples_mega_image.data()[(y*hmult + dy) * sw + (x*wmult + dx)] = debug_image.data()[dy * wmult + dx];
      }
    }

    if (out_image_debug)
    {
      if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_DETECTION)
        out_image_debug[y * m_width + x] = float4(1, 1, 1, 1);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_FOUND)
        out_image_debug[y * m_width + x] = border_points > 0 ? float4(1, 1, 1, 1) : float4(0, 0, 0, 1);
      else if (m_preset_dr.debug_render_mode == DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL)
        out_image_debug[y * m_width + x] = to_float4(visualize_value_debug(total_diff), 1.0f);
    }
  }

  void MultiRendererDR::Regularization(float *out_dLoss_dS, unsigned params_count)
  {
    SdfSBSHeader header = ((BVHDR *)m_pAccelStruct.get())->m_SdfSBSHeaders[0];
    unsigned node_count = ((BVHDR *)m_pAccelStruct.get())->m_SdfSBSNodes.size();
    SdfSBSNode *  nodes = ((BVHDR *)m_pAccelStruct.get())->m_SdfSBSNodes.data();
    uint32_t *     data = ((BVHDR *)m_pAccelStruct.get())->m_SdfSBSData.data();
    float *      data_f = ((BVHDR *)m_pAccelStruct.get())->m_SdfSBSDataF.data();
    unsigned v_size = header.brick_size + 1;
    float power = m_preset_dr.reg_power;
    float lambda = m_preset_dr.reg_lambda;

    bool use_multithreading = !(m_preset_dr.debug_border_samples || 
                                m_preset_dr.debug_pd_images ||
                                m_preset_dr.debug_border_samples_mega_image);

    unsigned max_threads = use_multithreading ? omp_get_max_threads() : 1;
    unsigned steps = (node_count + max_threads - 1)/max_threads;
    //std::vector<float> out_dLoss_dS(((BVHDR *)m_pAccelStruct.get())->m_SdfSBSDataF.size(), 0.0f);
    double total_reg_loss = 0.0;

    omp_set_num_threads(max_threads);

    #pragma omp parallel for
    for (int thread_id = 0; thread_id < max_threads; thread_id++)
    {
      unsigned start = thread_id * steps;
      unsigned end = std::min((thread_id + 1) * steps, m_width * m_height);
      for (int node_id = start; node_id < end; node_id++)
      {
        unsigned offset = nodes[node_id].data_offset;
        for (int i0 = 0; i0 < v_size; i0++)
        {
          for (int j0 = 0; j0 < v_size; j0++)
          {
            for (int k0 = 0; k0 < v_size; k0++)
            {
              unsigned idx = offset + i0 * v_size * v_size + j0 * v_size + k0;              
              if (i0 > 0)
              {
                float d = data_f[data[idx]] - data_f[data[idx - v_size * v_size]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              if (i0 < v_size - 1)
              {
                float d = data_f[data[idx]] - data_f[data[idx + v_size * v_size]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              if (j0 > 0)
              {
                float d = data_f[data[idx]] - data_f[data[idx - v_size]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              } 

              if (j0 < v_size - 1)
              {
                float d = data_f[data[idx]] - data_f[data[idx + v_size]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              if (k0 > 0)
              {
                float d = data_f[data[idx]] - data_f[data[idx - 1]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              if (k0 < v_size - 1)
              {
                float d = data_f[data[idx]] - data_f[data[idx + 1]];
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);  

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                out_dLoss_dS[thread_id*params_count + data[idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }
            }
          }
        }
      }
    }

    //for (int i = 0; i < out_dLoss_dS.size(); i++)
    //  printf("%f \n", out_dLoss_dS[i]);
    //printf("\n total reg loss = %f\n", total_reg_loss);

    omp_set_num_threads(omp_get_max_threads());
  }


  float MultiRendererDR::SolveEikonal(float3 axes_mins, float grid_spacing)
  {
    float3 m = axes_mins;
    float  h = grid_spacing;

    // Sort m in ascending order
    if (m[0] > m[1])
      std::swap(m[0], m[1]);
    if (m[0] > m[2])
      std::swap(m[0], m[2]);
    if (m[1] > m[2])
      std::swap(m[1], m[2]);

    // From: https://github.com/scikit-fmm/scikit-fmm/blob/master/skfmm/distance_marcher.cpp
    // Exact same results, but for different h's
    // float3 h{grid_spacing};
    // float m2_0     = m[0] * m[0];
    // float m2_1     = m[1] * m[1];
    // float m2_2     = m[2] * m[2];
    // float d2_0     = h[0] * h[0];
    // float d2_1     = h[1] * h[1];
    // float d2_2     = h[2] * h[2];
    // float dist_new = m[0] + h[0];

    // if (dist_new > m[1]) {
    //     float s = std::sqrt(-m2_0 + 2 * m[0] * m[1] - m2_1 + d2_0 + d2_1);
    //     dist_new = (m[1] * d2_0 + m[0] * d2_1 + h[0] * h[1] * s) / (d2_0 + d2_1);

    //     if (dist_new > m[2]) {
    //         float a = std::sqrt(-m2_0 * d2_1 - m2_0 * d2_2 + 2 * m[0] * m[1] * d2_2 - m2_1 * d2_0 -
    //                              m2_1 * d2_2 + 2 * m[0] * m[2] * d2_1 - m2_2 * d2_0 - m2_2 * d2_1 +
    //                              2 * m[1] * m[2] * d2_0 + d2_0 * d2_1 + d2_0 * d2_2 + d2_1 * d2_2);
    //         dist_new =
    //             (m[2] * d2_0 * d2_1 + m[1] * d2_0 * d2_2 + m[0] * d2_1 * d2_2 + h[0] * h[1] * h[2] * a) /
    //             (d2_0 * d2_1 + d2_0 * d2_2 + d2_1 * d2_2);
    //     }
    // }
    // return dist_new;

    // // f == 1

    float dist_new = m[0] + h;
    if (dist_new > m[1])
    {
      float h_2 = h * h;
      float m_sum = m[0] + m[1], c_2 = (m[0] - m[1]) * (m[0] - m[1]);

      float x2 = (m_sum + std::sqrt(2 * h_2 - c_2)) * 0.5f;
      dist_new = x2;

      if (dist_new > m[2])
      {
        const float one_third = 1.f / 3;
        m_sum += m[2];
        c_2 = 3.f * (m[0] * m[0] + m[1] * m[1] + m[2] * m[2] - h_2);

        x2 = (m_sum + std::sqrt(m_sum * m_sum - c_2)) * one_third;
        dist_new = x2;
      }
    }
    return dist_new;
  }


  void MultiRendererDR::Redistance(float *dist_in, uint3 size_in, float grid_spacing, uint32_t num_iters)
  {
    const uint32_t N_in = size_in.x * size_in.y * size_in.z;
    const int3 axes_offsets{(int) (size_in.y * size_in.z), (int) size_in.z, 1};

    float *dist_bord[2] = { new float[N_in], new float[N_in] };
    bool *frozen = new bool[N_in];


    // Print input array's eikonal check values to a file

    // FILE* FOUT = fopen("./saves/log.txt", "a+");
    // fprintf(FOUT, "Array IN_CHECK:\n");
    // double eikonal_mean = 0.f;

    // for (int i = 0; i < size_in.x; ++i) {
    //   for (int j = 0; j < size_in.y; ++j) {
    //     for (int k = 0; k < size_in.z; ++k) {
    //       int3 ind_in{ i, j, k };
    //       uint32_t ind_in_lin = ind_in.x * axes_offsets.x + ind_in.y * axes_offsets.y + ind_in.z;

    //       float3 axes_mins{99999, 99999, 99999};

    //       for (int dim = 0; dim < 3; ++dim)
    //       {
    //         if (ind_in[dim] > 0 && ind_in[dim] < size_in[dim] - 1)
    //         {
    //           axes_mins[dim] = dist_in[ind_in_lin - axes_offsets[dim]];
    //           float dist_tmp = dist_in[ind_in_lin + axes_offsets[dim]];
    //           if (std::abs(dist_tmp) < std::abs(axes_mins[dim]))
    //             axes_mins[dim] = dist_tmp;
    //         }
    //         else if (ind_in[dim] >  0 && ind_in[dim] == size_in[dim] - 1)
    //           axes_mins[dim] = dist_in[ind_in_lin - axes_offsets[dim]];
    //         else if (ind_in[dim] == 0 && ind_in[dim] <  size_in[dim] - 1)
    //           axes_mins[dim] = dist_in[ind_in_lin + axes_offsets[dim]];
    //       }

    //       float eik_val = ((dist_in[ind_in_lin] - axes_mins.x) * (dist_in[ind_in_lin] - axes_mins.x) +
    //                        (dist_in[ind_in_lin] - axes_mins.y) * (dist_in[ind_in_lin] - axes_mins.y) +
    //                        (dist_in[ind_in_lin] - axes_mins.z) * (dist_in[ind_in_lin] - axes_mins.z)) / (grid_spacing * grid_spacing);
    //       fprintf(FOUT, "%12.6f, ", eik_val);
    //       eikonal_mean += eik_val;
    //     }
    //     fprintf(FOUT, "\n");
    //   }
    //   fprintf(FOUT, "\n");
    // }
    // printf("Input eikonal mean = %f\n", eikonal_mean / (size_in.x * size_in.y * size_in.z));


    // Init

    const float big_active_const = 99999.f;
    const float Infin = 1e38f;
    for (int i = 0; i < size_in.x; ++i)
      for (int j = 0; j < size_in.y; ++j)
        for (int k = 0; k < size_in.z; ++k)
        {
          int3 ind_in = { i, j, k };
          uint32_t ind_in_lin = ind_in.x * axes_offsets.x + ind_in.y * axes_offsets.y + ind_in.z;

          frozen[ind_in_lin] = false;

          // Initialize with a big value
          dist_bord[0][ind_in_lin] = big_active_const;
          dist_bord[1][ind_in_lin] = big_active_const;

          float dist_val = dist_in[ind_in_lin];

          // Surface itself is not moved
          if (dist_val == 0.f)
          {
            frozen[ind_in_lin] = true;
            dist_bord[0][ind_in_lin] = 0.f;
            dist_bord[1][ind_in_lin] = 0.f;
          }

          // Also freeze if a surface is between this node and any of its neighbours
          if (!frozen[ind_in_lin])
          {
            for (int dim = 0; dim < 3; ++dim)
              for (int offset_sign = -1; offset_sign < 2; offset_sign += 2)
              {
                int3 neighbour = ind_in;
                neighbour[dim] += offset_sign;

                if (neighbour.x >= 0 && neighbour.x < size_in.x &&
                    neighbour.y >= 0 && neighbour.y < size_in.y &&
                    neighbour.z >= 0 && neighbour.z < size_in.z)
                {
                  float n_val = dist_in[neighbour.x * axes_offsets.x +
                                        neighbour.y * axes_offsets.y +
                                        neighbour.z];
                  if (n_val * dist_val < 0)
                  {
                    // near surface SDF = 0 - initialize with the exact values
                    dist_bord[0][ind_in_lin] = dist_val;
                    dist_bord[1][ind_in_lin] = dist_val;

                    frozen[ind_in_lin] = true;
                  }
                }
              }
          }
        }
    

    // Update loop

    int curr_arr_ind = 0;
    const int max_level = size_in.x + size_in.y + size_in.z - 3;

    for (uint32_t iter = 0u; iter < num_iters; ++iter)
    {
      for (int ordering = 0; ordering < 8; ++ordering)
      {
        int start, end, step;
        if (ordering == 1 || ordering == 4 || ordering == 6 || ordering == 7)
        {
          start = max_level;
          end   = -1;
          step  = -1;
        }
        else
        {
          start = 0;
          end   = max_level + 1;
          step  = 1;
        }


        for (int level = start; level != end; level += step)
        {
          int xs = std::max(0, level - int(size_in.y + size_in.z - 2));
          int ys = std::max(0, level - int(size_in.x + size_in.z - 2));
          int xe = std::min((int) size_in.x - 1, level);
          int ye = std::min((int) size_in.y - 1, level);
          int xr = xe - xs + 1;
          int yr = ye - ys + 1;

          // pragma parallel for
          for (int i = xs; i < xs + xr; ++i)
            for (int j = ys; j < ys + yr; ++j)
            {
              int k = level - i - j;
              if (k < 0 || k >= size_in.z) continue; // only k check is needed

              int3 ind_in{ i, j, k };
              uint32_t ind_in_lin = ind_in.x * axes_offsets.x + ind_in.y * axes_offsets.y + ind_in.z;

              dist_bord[curr_arr_ind][ind_in_lin] = dist_bord[1 - curr_arr_ind][ind_in_lin];
              if (frozen[ind_in_lin]) continue;

              float3 axes_mins{big_active_const, big_active_const, big_active_const};

              for (int dim = 0; dim < 3; ++dim)
              {
                if (ind_in[dim] > 0 && ind_in[dim] < size_in[dim] - 1)
                {
                  axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin - axes_offsets[dim]];
                  float dist_tmp = dist_bord[1 - curr_arr_ind][ind_in_lin + axes_offsets[dim]];
                  if (std::abs(dist_tmp) < std::abs(axes_mins[dim]))
                    axes_mins[dim] = dist_tmp;
                }
                else if (ind_in[dim] >  0 && ind_in[dim] == size_in[dim] - 1)
                  axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin - axes_offsets[dim]];
                else if (ind_in[dim] == 0 && ind_in[dim] <  size_in[dim] - 1)
                  axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin + axes_offsets[dim]];
              }

              if (axes_mins.x < big_active_const || axes_mins.y < big_active_const || axes_mins.z < big_active_const)
              {
                bool pos_part = axes_mins.x >= 0.f && axes_mins.y >= 0.f && axes_mins.z >= 0.f;

                if (pos_part)
                {
                  dist_bord[curr_arr_ind][ind_in_lin] = std::min( SolveEikonal(axes_mins, grid_spacing),
                                                                  dist_bord[1 - curr_arr_ind][ind_in_lin]);
                }
                else
                {
                  if (axes_mins.x < 0.f) axes_mins.x = -axes_mins.x;
                  if (axes_mins.y < 0.f) axes_mins.y = -axes_mins.y;
                  if (axes_mins.z < 0.f) axes_mins.z = -axes_mins.z;
                  dist_bord[curr_arr_ind][ind_in_lin] = std::max(-SolveEikonal(axes_mins, grid_spacing),
                                                                 -std::abs(dist_bord[1 - curr_arr_ind][ind_in_lin]));
                }
              }
            }
        }
      }
      curr_arr_ind = 1 - curr_arr_ind; // swap
    }


    // dist_bord[1 - curr_arr_ind] has the result
    // copy dist_bord[1 - curr_arr_ind] to dist_in (input array)
    // Save eikonal check values to the now unused dist_bord[curr_arr_ind]

    // eikonal_mean = 0.;

    // pragma parallel for
    for (int i = 0; i < size_in.x; ++i)
      for (int j = 0; j < size_in.y; ++j)
        for (int k = 0; k < size_in.z; ++k)
        {
          uint32_t ind_in_lin = i * axes_offsets.x + j * axes_offsets.y + k;
          dist_in[ind_in_lin] = dist_bord[1 - curr_arr_ind][ind_in_lin];

          int3 ind_in{i,j,k};
          // float3 axes_mins{big_active_const, big_active_const, big_active_const};

          // for (int dim = 0; dim < 3; ++dim)
          // {
          //   if (ind_in[dim] > 0 && ind_in[dim] < size_in[dim] - 1)
          //   {
          //     axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin - axes_offsets[dim]];
          //     float dist_tmp = dist_bord[1 - curr_arr_ind][ind_in_lin + axes_offsets[dim]];
          //     if (std::abs(dist_tmp) < std::abs(axes_mins[dim]))
          //       axes_mins[dim] = dist_tmp;
          //   }
          //   else if (ind_in[dim] >  0 && ind_in[dim] == size_in[dim] - 1)
          //     axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin - axes_offsets[dim]];
          //   else if (ind_in[dim] == 0 && ind_in[dim] <  size_in[dim] - 1)
          //     axes_mins[dim] = dist_bord[1 - curr_arr_ind][ind_in_lin + axes_offsets[dim]];
          // }
          // dist_bord[curr_arr_ind][ind_in_lin] = ((dist_in[ind_in_lin] - axes_mins.x) * (dist_in[ind_in_lin] - axes_mins.x) +
          //                                        (dist_in[ind_in_lin] - axes_mins.y) * (dist_in[ind_in_lin] - axes_mins.y) +
          //                                        (dist_in[ind_in_lin] - axes_mins.z) * (dist_in[ind_in_lin] - axes_mins.z)) / (grid_spacing * grid_spacing);
          // eikonal_mean += dist_bord[curr_arr_ind][ind_in_lin];
        }
    // printf("Output eikonal mean = %f\n", eikonal_mean / (size_in.x * size_in.y * size_in.z));

    // fprintf(FOUT, "Array OUT_CHECK:\n");
    // for (int i = 0; i < size_in.x; ++i) {
    //   for (int j = 0; j < size_in.y; ++j) {
    //     for (int k = 0; k < size_in.z; ++k) {
    //       fprintf(FOUT, "%12.6f, ", dist_bord[curr_arr_ind][i * (size_in.z * size_in.y) + j * size_in.z + k]);
    //     }
    //     fprintf(FOUT, "\n");
    //   }
    //   fprintf(FOUT, "\n");
    // }

    // fprintf(FOUT, "Array OUT:\n");
    // for (int i = 0; i < size_in.x; ++i) {
    //   for (int j = 0; j < size_in.y; ++j) {
    //     for (int k = 0; k < size_in.z; ++k) {
    //       fprintf(FOUT, "%12.6f, ", dist_in[i * (size_in.z * size_in.y) + j * size_in.z + k]);
    //     }
    //     fprintf(FOUT, "\n");
    //   }
    //   fprintf(FOUT, "\n");
    // }

    delete[] frozen;
    delete[] dist_bord[1];
    delete[] dist_bord[0];
    // fclose(FOUT);
  }
}