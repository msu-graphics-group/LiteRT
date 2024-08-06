#pragma once

#include "DR_common.h"
#include "BVH2DR.h"
#include "../Renderer/eye_ray.h"

namespace dr
{
  static MultiRendererDRPreset getDefaultPresetDR()
  {
    MultiRendererDRPreset preset;

    preset.spp = 1;
    preset.render_width = 0; //use sizes of reference images
    preset.render_height = 0;

    preset.dr_loss_function = DR_LOSS_FUNCTION_MSE;
    preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
    preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_COLOR;
    preset.dr_input_type = DR_INPUT_TYPE_COLOR;

    preset.border_spp = 256;
    preset.border_relax_eps = 3e-4f;
    preset.border_depth_threshold = 1000.0f; //only external borders will pass
    preset.border_color_threshold = 1e-6f;   //only borders with no color change are discarded

    preset.opt_lr = 0.01f;
    preset.opt_beta_1 = 0.9f;
    preset.opt_beta_2 = 0.999f;
    preset.opt_eps = 1e-8f;
    preset.opt_iterations = 500;
    preset.image_batch_size = 1;

    preset.debug_print = false;
    preset.debug_print_interval = 10;
    preset.debug_progress_images = DEBUG_PROGRESS_RAW;
    preset.debug_progress_interval = 100;

    return preset;
  }
  
  class MultiRendererDR : public MultiRenderer
  {
  public:
    MultiRendererDR();
    void SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                      const std::vector<LiteMath::float4x4>& worldView, 
                      const std::vector<LiteMath::float4x4>& proj);
    void OptimizeFixedStructure(MultiRendererDRPreset preset, SdfSBS &sbs);
    void OptimizeGrid(unsigned start_grid_size, bool no_last_step_resize, std::vector<MultiRendererDRPreset> presets);

    const LiteImage::Image2D<float4> &getLastImage(unsigned view_id) const { return m_images[view_id]; }
    const float *getLastdLoss_dS() const { return m_dLoss_dS_tmp.data(); }
    void set_debug_mode(bool enable) { debug_pd_images = enable; }

  protected:
    float RenderDR(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count);
    float RenderDRFiniteDiff(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                             unsigned start_index, unsigned end_index, float delta = 0.001f);
    void OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset);
    float CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS);
    void CastBorderRay(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS);
    float3 CalculateColorWithGrad(const CRT_HitDR &hit, LiteMath::float3x3 &dColor_dDiffuse,
                                  LiteMath::float3x3 &dColor_dNorm);
    void PreprocessRefImages(unsigned width, unsigned height, bool to_mask, float3 background_color = float3(0,0,0));

    std::vector<LiteImage::Image2D<float4>> m_imagesRefOriginal;
    std::vector<LiteImage::Image2D<float4>> m_imagesRef;
    std::vector<LiteImage::Image2D<float4>> m_images;
    std::vector<LiteMath::float4x4> m_worldViewRef;
    std::vector<LiteMath::float4x4> m_projRef;
    std::vector<float> m_dLoss_dS_tmp;
    std::vector<float> m_Opt_tmp;
    std::vector<uint32_t> m_borderPixels;
    MultiRendererDRPreset m_preset_dr;

    std::vector<LiteImage::Image2D<float4>> m_imagesDebug;
    bool debug_pd_images = false;
  };
}