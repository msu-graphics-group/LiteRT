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

  static float linear_to_absolute_depth(float lz)
  {
    return lz * (z_far - z_near) + z_near;
  }

  static float absolute_to_linear_depth(float z)
  {
    return (z - z_near) / (z_far - z_near);
  }

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
    case DR_DEBUG_RENDER_MODE_BORDER_DETECTION:
    case DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL:
      return MULTI_RENDER_MODE_DIFFUSE;
    case DR_RENDER_MODE_LAMBERT:
      return MULTI_RENDER_MODE_LAMBERT;    
    case DR_RENDER_MODE_MASK:
      return MULTI_RENDER_MODE_MASK;
    case DR_DEBUG_RENDER_MODE_PRIMITIVE:
      return MULTI_RENDER_MODE_PRIMITIVE;
    case DR_RENDER_MODE_LINEAR_DEPTH:
      return MULTI_RENDER_MODE_LINEAR_DEPTH;
    default:
      printf("Unknown diff_render_mode: %u\n", diff_render_mode);
      return MULTI_RENDER_MODE_DIFFUSE;
    }
  }

  MultiRendererDR::MultiRendererDR()
  {
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
    m_imagesRefOriginal = images;
    m_worldViewRef = worldView;
    m_projRef = proj;
  }

  void MultiRendererDR::PreprocessRefImages(unsigned width, unsigned height, bool to_mask, float3 background_color)
  {
    //preprocess reference images (m_imagesRefOriginal) to m_imagesRef, that will be used for optimization
    m_imagesRef.resize(m_imagesRefOriginal.size());
    for (unsigned image_n = 0; image_n < m_imagesRefOriginal.size(); image_n++)
    {
      //create mask if needed
      LiteImage::Image2D<float4> mask;
      if (to_mask)
      {
        mask = m_imagesRefOriginal[image_n];
        for (int i=0;i<m_imagesRefOriginal[image_n].width()*m_imagesRefOriginal[image_n].height();i++)
        {
          float3 color = to_float3(m_imagesRefOriginal[image_n].data()[i]);
          mask.data()[i] = length(color - background_color) < 0.001f ? float4(0,0,0,1) : float4(1,1,1,1);
        }
      }

      //get reference image by sampling original (or mask) with default bilinear sampler
      LiteImage::Sampler sampler = LiteImage::Sampler();
      sampler.filter = LiteImage::Sampler::Filter::LINEAR;
      LiteImage::Image2D<float4> &original_image = to_mask ? mask : m_imagesRefOriginal[image_n];
      unsigned spp_x = ceil(float(original_image.width()) / width);
      unsigned spp_y = ceil(float(original_image.height()) / height);

      m_imagesRef[image_n] = LiteImage::Image2D<float4>(width, height);
      for (unsigned y = 0; y < height; y++)
      {
        for (unsigned x = 0; x < width; x++)
        {
          for (unsigned dx = 0; dx < spp_x; dx++)
          {
            for (unsigned dy = 0; dy < spp_y; dy++)
            {
              float2 uv = float2(float(spp_x*x + dx) / (spp_x*width), float(spp_y*y + dy) / (spp_y*height));
              m_imagesRef[image_n].data()[y*width + x] += original_image.sample(sampler, uv) / (spp_x*spp_y);
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

    m_preset_dr = preset;
    m_preset.spp = preset.spp;
    m_preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON; //we need newton to minimize calculations for border integral
    m_preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
    m_preset.render_mode = diff_render_mode_to_multi_render_mode(preset.dr_render_mode);
    m_pAccelStruct->SetPreset(m_preset);

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

    m_images.resize(m_imagesRef.size(), LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));
    m_pAccelStruct = std::shared_ptr<ISceneObject>(new BVHDR());

    SetScene(sbs, true);

    float *params = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF.data();
    unsigned images_count = m_imagesRef.size();

    if (preset.debug_pd_images)
      m_imagesDebug.resize(params_count, LiteImage::Image2D<float4>(m_width, m_height, float4(0, 0, 0, 1)));
    
    if (preset.debug_border_samples_mega_image)
      samples_mega_image.resize(m_width*MEGA_PIXEL_SIZE, m_height*MEGA_PIXEL_SIZE);

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
          bool is_geometry = (preset.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_GEOMETRY);
          unsigned color_params = 3*8*sbs.nodes.size();
          unsigned active_params_start = is_geometry ? 0 : params_count - color_params;
          unsigned active_params_end   = is_geometry ? params_count - color_params : params_count;

          loss = RenderDRFiniteDiff(m_imagesRef[image_id].data(), m_images[image_id].data(), m_dLoss_dS_tmp.data(), params_count,
                                    active_params_start, active_params_end, 0.005f);
        }

        loss_sum += loss;
        loss_max = std::max(loss_max, loss);
        loss_min = std::min(loss_min, loss);
      }

      auto t2 = std::chrono::high_resolution_clock::now();

      //accumulate
      for (int i=1; i< max_threads; i++)
        for (int j = 0; j < params_count; j++)
          m_dLoss_dS_tmp[j] += m_dLoss_dS_tmp[j + params_count * i];

      for (int j = 0; j < params_count; j++)
        m_dLoss_dS_tmp[j] /= preset.image_batch_size;

      auto t3 = std::chrono::high_resolution_clock::now();

      OptimizeStepAdam(iter, m_dLoss_dS_tmp.data(), params, m_Opt_tmp.data(), params_count, preset);

      auto t4 = std::chrono::high_resolution_clock::now();
      float time_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      float time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
      float time_3 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
      if (iter == 0)
        timeAvg = time_1 + time_2 + time_3;
      else
        timeAvg = 0.97f * timeAvg + 0.03f * (time_1 + time_2 + time_3);

      if (preset.debug_print && iter % preset.debug_print_interval == 0)
      {
        printf("Iter:%4d, loss: %f (%f-%f) ", iter, loss_sum/preset.image_batch_size, loss_min, loss_max);
        printf("%.1f ms/iter (%.1f + %.1f + %.1f) ", (time_1 + time_2 + time_3), time_1, time_2, time_3);
        printf("ETA %.1f s\n", (timeAvg * (preset.opt_iterations - iter - 1)) / 1000.0f);
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
            m_pAccelStruct->SetPreset(m_preset);

            UpdateCamera(m_worldViewRef[image_id], m_projRef[image_id]);
            RenderFloat(m_images[image_id].data(), m_width, m_height, "color");

            m_preset.render_mode = original_mode;
            m_pAccelStruct->SetPreset(m_preset);
          }
          LiteImage::SaveImage<float4>(("saves/iter_"+std::to_string(iter)+"_"+std::to_string(image_id)+".bmp").c_str(), m_images[image_id]);
        }
      }
    } 

    if  (preset.debug_pd_images && preset.dr_diff_mode == DR_DIFF_MODE_DEFAULT)
    {
      for (int i = 0; i < params_count; i++)
      {
        for (int j = 0; j < m_width * m_height; j++)
        {
          float l = m_imagesDebug[i].data()[j].x;
          m_imagesDebug[i].data()[j] = float4(std::max(0.0f, l), std::max(0.0f, -l),0,1);
        }
        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"a.bmp").c_str(), m_imagesDebug[i]);
      }
    }

    sbs.values_f = ((BVHDR*)m_pAccelStruct.get())->m_SdfSBSDataF;
  }

  float MultiRendererDR::RenderDR(const float4 *image_ref, LiteMath::float4 *out_image,
                                  float *out_dLoss_dS, unsigned params_count)
  {
    bool use_multithreading = !(m_preset_dr.debug_border_samples || 
                                m_preset_dr.debug_pd_images ||
                                m_preset_dr.debug_border_samples_mega_image);

    unsigned max_threads = use_multithreading ? omp_get_max_threads() : 1;
    unsigned steps = (m_width * m_height + max_threads - 1)/max_threads;
    std::vector<double> loss_v(max_threads, 0.0f);

    omp_set_num_threads(max_threads);

    //I - render image, calculate internal derivatives
    #pragma omp parallel for
    for (int thread_id = 0; thread_id < max_threads; thread_id++)
    {
      unsigned start = thread_id * steps;
      unsigned end = std::min((thread_id + 1) * steps, m_width * m_height);
      for (int i = start; i < end; i++)
        loss_v[thread_id] += CastRayWithGrad(i, image_ref, out_image, out_dLoss_dS + (params_count * thread_id));
    }

    //II - find border pixels
    m_borderPixels.resize(0);
    const int search_radius = 1;
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

      float3 color0 = to_float3(out_image[y*m_width + x]);
      float  max_color_diff = 0.0f;

      for (int dx = -search_radius; dx <= search_radius; dx++)
      {
        for (int dy = -search_radius; dy <= search_radius; dy++)
        {
          float d = out_image[(y + dy) * m_width + x + dx].w;
          max_diff = std::max(max_diff, std::abs(d0 - d));

          float3 color = to_float3(out_image[(y + dy) * m_width + x + dx]);
          max_color_diff = std::max(max_color_diff, length(color0 - color));
        }
      }

      float ref_color_diff = length(to_float3(out_image[y*m_width + x]) - to_float3(image_ref[y*m_width + x]));

      if (max_diff > 0)// && 
      // only outer borders, so no reason to check colors
      //    ref_color_diff > m_preset_dr.border_color_threshold &&
      //    max_color_diff > m_preset_dr.border_color_threshold)
      {
        m_borderPixels.push_back(i);
      }
    }

    if (m_preset_dr.debug_border_samples_mega_image)
    {
      unsigned sw = samples_mega_image.width(), sh = samples_mega_image.height();
      unsigned wmult = sw/m_width, hmult = sh/m_height;
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
      #pragma omp parallel for
      for (int thread_id = 0; thread_id < max_threads; thread_id++)
      {
        unsigned start = thread_id * border_steps;
        unsigned end = std::min((thread_id + 1) * border_steps, (unsigned)m_borderPixels.size());
        for (int i = start; i < end; i++)
          CastBorderRay(m_borderPixels[i], image_ref, out_image, out_dLoss_dS + (params_count * thread_id));
      }
    }
    if (m_preset_dr.dr_render_mode == DR_DEBUG_RENDER_MODE_BORDER_DETECTION ||
        m_preset_dr.dr_render_mode == DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL)
    {
      // make image with only border pixels
      LiteImage::Image2D<float4> image_borders(m_width, m_height, float4(0, 0, 0, 1));
      for (int i = 0; i < m_borderPixels.size(); i++)
      {
        const uint XY = m_packedXY[m_borderPixels[i]];
        const uint x = (XY & 0x0000FFFF);
        const uint y = (XY & 0xFFFF0000) >> 16;
        image_borders.data()[y * m_width + x] = out_image[y * m_width + x];
      }

      memcpy(out_image, image_borders.data(), sizeof(float4) * m_width * m_height);
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
      RenderDR(image_ref, out_image, m_dLoss_dS_tmp_2.data(), params_count);
      for (int j = 0; j < m_width * m_height; j++)
      {
        float l = Loss(m_preset_dr.dr_loss_function, to_float3(out_image[j]), to_float3(image_ref[j]));
        image_1.data()[j] = out_image[j];//float4(l,l,0,1);
        loss_plus += l;
      }
    
      params[i] = p0 - delta;
      //RenderFloat(out_image, m_width, m_height, "color");
      RenderDR(image_ref, out_image, m_dLoss_dS_tmp_2.data(), params_count);
      for (int j = 0; j < m_width * m_height; j++)
      {
        float l = Loss(m_preset_dr.dr_loss_function, to_float3(out_image[j]), to_float3(image_ref[j]));
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
        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"_1.bmp").c_str(), image_1);
        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"_2.bmp").c_str(), image_2);

        for (int j = 0; j < m_width * m_height; j++)
        {
          float l1 = Loss(m_preset_dr.dr_loss_function, to_float3(image_1.data()[j]), to_float3(image_ref[j]));
          float l2 = Loss(m_preset_dr.dr_loss_function, to_float3(image_2.data()[j]), to_float3(image_ref[j]));
          float l = (l1 - l2) / (2 * delta);
          image_1.data()[j] = float4(std::max(0.0f, l), std::max(0.0f, -l),0,1);
        }

        LiteImage::SaveImage<float4>(("saves/PD_"+std::to_string(i)+"b.bmp").c_str(), image_1);
      }

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

  float3 MultiRendererDR::CalculateColorWithGrad(const CRT_HitDR &hit,
                                                 LiteMath::float3x3 &dColor_dDiffuse,
                                                 LiteMath::float3x3 &dColor_dNorm)
  {
    float3 color = float3(1, 0, 1);

    switch (m_preset_dr.dr_render_mode)
    {
    case DR_RENDER_MODE_DIFFUSE:
    case DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL:
    case DR_DEBUG_RENDER_MODE_BORDER_DETECTION:
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
    case DR_DEBUG_RENDER_MODE_PRIMITIVE:
      color = to_float3(decode_RGBA8(m_palette[(hit.primId) % palette_size]));
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

  float MultiRendererDR::CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4 *out_image, float *out_dLoss_dS)
  {
    if (tidX >= m_packedXY.size())
      return 0.0;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;
    
    float3 res_color = float3(0,0,0);
    float res_loss = 0.0f;
    int hit_count = 0;

    uint32_t spp_sqrt = uint32_t(sqrt(m_preset.spp));
    float i_spp_sqrt = 1.0f/spp_sqrt;
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
      else if (m_preset_dr.dr_reconstruction_flags & DR_RECONSTRUCTION_FLAG_GEOMETRY)
      {
        if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_MASK)
          ray_flags |= DR_RAY_FLAG_NO_DIFF;
        else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_DIFFUSE)
          ray_flags |= DR_RAY_FLAG_DDIFFUSE_DPOS;
        else if (m_preset_dr.dr_render_mode == DR_RENDER_MODE_LAMBERT)
          ray_flags |= DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS;
      }
    }

    for (uint32_t i = 0; i < m_preset.spp; i++)
    {
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, i) : i_spp_sqrt*float2(i/spp_sqrt+0.5, i%spp_sqrt+0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
      
      float3 color = float3(0,0,0);
      float3 dLoss_dColor = float3(0,0,0);
      LiteMath::float3x3 dColor_dDiffuse = LiteMath::make_float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
      LiteMath::float3x3 dColor_dNorm    = LiteMath::make_float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
      
      CRT_HitDR hit = ((BVHDR*)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(ray_flags, rayPosAndNear, rayDirAndFar, nullptr);
      
      if (hit.primId == 0xFFFFFFFF) //no hit
      {
        color = background_color;
        res_color += color / m_preset.spp;
      }
      else
      {
        hit_count++;
        color = CalculateColorWithGrad(hit, dColor_dDiffuse, dColor_dNorm);
        dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, color, to_float3(image_ref[y * m_width + x]));

        if (ray_flags & DR_RAY_FLAG_DDIFFUSE_DCOLOR)
        {
          for (PDColor &pd : hit.dDiffuse_dSc)
          {
            float3 diff = dLoss_dColor * (dColor_dDiffuse * float3(pd.value));
            out_dLoss_dS[pd.index + 0] += diff.x / m_preset.spp;
            out_dLoss_dS[pd.index + 1] += diff.y / m_preset.spp;
            out_dLoss_dS[pd.index + 2] += diff.z / m_preset.spp;

            if (m_preset_dr.debug_pd_images)
            {
              m_imagesDebug[pd.index + 0].data()[y * m_width + x] += (diff.x / m_preset.spp) * float4(1,1,1,0);
              m_imagesDebug[pd.index + 1].data()[y * m_width + x] += (diff.y / m_preset.spp) * float4(1,1,1,0);
              m_imagesDebug[pd.index + 2].data()[y * m_width + x] += (diff.z / m_preset.spp) * float4(1,1,1,0);
            }
          }
        }
        
        if (ray_flags & (DR_RAY_FLAG_DDIFFUSE_DPOS | DR_RAY_FLAG_DNORM_DPOS))
        {
          for (PDDist &pd : hit.dDiffuseNormal_dSd)
          {
            float diff = dot(dLoss_dColor, dColor_dDiffuse * pd.dDiffuse + dColor_dNorm * pd.dNorm);
            out_dLoss_dS[pd.index] += diff / m_preset.spp;

            if (m_preset_dr.debug_pd_images)
              m_imagesDebug[pd.index].data()[y * m_width + x] += (diff / m_preset.spp) * float4(1,1,1,0);
          }
        }

        if (ray_flags & DR_RAY_FLAG_DDIST_DPOS)
        {
          for (PDDist &pd : hit.dDiffuseNormal_dSd)
          {
            float diff = dot(dLoss_dColor, float3(pd.dDist)) / (z_far - z_near);
            out_dLoss_dS[pd.index] += diff / m_preset.spp;
            
            if (m_preset_dr.debug_pd_images)
              m_imagesDebug[pd.index].data()[y * m_width + x] += (diff / m_preset.spp) * float4(1,1,1,0);
          }
        }
      }

      res_color += color / m_preset.spp;

      float loss          = Loss(m_preset_dr.dr_loss_function, color, to_float3(image_ref[y * m_width + x]));
      res_loss += loss / m_preset.spp;
    }
    out_image[y * m_width + x] = float4(res_color.x, res_color.y, res_color.z, (float)hit_count/m_preset.spp);

    return res_loss;
  }

  void MultiRendererDR::CastBorderRay(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS)
  {
    if (tidX >= m_packedXY.size())
      return;

    const uint XY = m_packedXY[tidX];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;

    const unsigned border_ray_flags = DR_RAY_FLAG_BORDER;
    const float relax_eps = m_preset_dr.border_relax_eps;
    const float3 background_color = float3(0.0f, 0.0f, 0.0f);

    const unsigned border_spp = m_preset_dr.border_spp;
    const uint32_t spp_sqrt = uint32_t(sqrt(border_spp));
    const float i_spp_sqrt = 1.0f / spp_sqrt;
    const float3 dLoss_dColor = LossGrad(m_preset_dr.dr_loss_function, to_float3(out_image[y * m_width + x]), to_float3(image_ref[y * m_width + x]));
    
    float total_diff = 0.0f;

    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      samples_debug_color.resize(border_spp, float4(0,0,0,0));
      samples_debug_pos_size.resize(border_spp, float4(0,0,0,0));
    }

    for (int sample_id = 0; sample_id < border_spp; sample_id++)
    {
      float pixel_diff = 0.0f;
      PDShape relax_pt{
          .f_in = background_color,
          .t = 0.f,
          .f_out = background_color,
          .sdf = relax_eps, // only choose points with SDF() less than this, no need to pass relax_eps
          .indices = {0, 0, 0, 0, 0, 0, 0, 0},
          .dSDF_dtheta = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
      float2 d = m_preset.ray_gen_mode == RAY_GEN_MODE_RANDOM ? rand2(x, y, sample_id) : i_spp_sqrt * float2(sample_id / spp_sqrt + 0.5, sample_id % spp_sqrt + 0.5);
      float4 rayPosAndNear, rayDirAndFar;
      kernel_InitEyeRay(tidX, d, &rayPosAndNear, &rayDirAndFar);
      CRT_HitDR hit = ((BVHDR *)m_pAccelStruct.get())->RayQuery_NearestHitWithGrad(border_ray_flags, rayPosAndNear, rayDirAndFar, &relax_pt);

      if (relax_pt.sdf < relax_eps)
      {
        float3 color_delta = float3(0,0,0);
        if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_COLOR)
        {
          if (relax_pt.t != 0 && hit.primId == 0xFFFFFFFF) //border with background
            color_delta = float3(1,1,1);
          else
            color_delta = relax_pt.f_in - relax_pt.f_out;
        }
        else if (m_preset_dr.dr_input_type == DR_INPUT_TYPE_LINEAR_DEPTH)
        {
          float l_in = linear_to_absolute_depth(relax_pt.t);
          float l_out = linear_to_absolute_depth(hit.t);
          color_delta = float3(l_in - l_out);
        }

        for (int j = 0; j < 8; j++)
        {
          float diff = dot(dLoss_dColor, (1.0f / relax_eps) * relax_pt.dSDF_dtheta[j] * (relax_pt.f_in - relax_pt.f_out));
          out_dLoss_dS[relax_pt.indices[j]] += diff / border_spp;

          total_diff += abs(diff) / border_spp;
          pixel_diff += length((1.0f / relax_eps) * relax_pt.dSDF_dtheta[j] * (relax_pt.f_in - relax_pt.f_out));
          if (m_preset_dr.debug_pd_images)
            m_imagesDebug[relax_pt.indices[j]].data()[y * m_width + x] += (diff / border_spp) * float4(1, 1, 1, 0);
        }
      }


      if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
      {
        samples_debug_pos_size[sample_id] = float4(d.x, d.y, 0, 2.0f/MEGA_PIXEL_SIZE);
        if (relax_pt.sdf < relax_eps && abs(pixel_diff) > 0.01f) //border
          samples_debug_color[sample_id] = float4(0, 0, 0.1*abs(pixel_diff), 1);
        else if (hit.primId == 0xFFFFFFFF) //background
          samples_debug_color[sample_id] = float4(0.05, 0.05, 0.05, 1);
        else //hit
          samples_debug_color[sample_id] = float4(1, 0, 0, 1);
      }
    }

    if (m_preset_dr.debug_border_samples || m_preset_dr.debug_border_samples_mega_image)
    {
      unsigned sw = samples_mega_image.width(), sh = samples_mega_image.height();
      unsigned wmult = sw/m_width, hmult = sh/m_height;
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

    if (m_preset_dr.dr_render_mode == DR_DEBUG_RENDER_MODE_BORDER_DETECTION)
    {
      //printf("%u %u total_diff %f\n", x, y, total_diff);
      out_image[y * m_width + x] = float4(1, 1, 1, 1);
    }
    else if (m_preset_dr.dr_render_mode == DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL)
      out_image[y * m_width + x] = float4(0.001f*total_diff, 0.01f*total_diff, 0.1f*total_diff, 1.0f);

  }
}