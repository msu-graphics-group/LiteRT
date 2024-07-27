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
    preset.dr_loss_function = DR_LOSS_FUNCTION_MSE;
    preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
    preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    preset.dr_reconstruction_type = DR_RECONSTRUCTION_TYPE_COLOR;

    preset.opt_lr = 0.01f;
    preset.opt_beta_1 = 0.9f;
    preset.opt_beta_2 = 0.999f;
    preset.opt_eps = 1e-8f;
    preset.opt_iterations = 500;
    preset.image_batch_size = 1;

    return preset;
  }
  
  class MultiRendererDR : public MultiRenderer
  {
  public:
    MultiRendererDR();
    void SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                      const std::vector<LiteMath::float4x4>& worldView, 
                      const std::vector<LiteMath::float4x4>& proj);
    void OptimizeColor(MultiRendererDRPreset preset, SdfSBS &sbs, bool verbose = false);
    // void OptimizeShape(MultiRendererDRPreset preset, SdfSBS &sbs, bool verbose = false);

    const LiteImage::Image2D<float4> &getLastImage(unsigned view_id) const { return m_images[view_id]; }
    const float *getLastdLoss_dS() const { return m_dLoss_dS_tmp.data(); }

  protected:
    float RenderDR(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count);
    float RenderDRFiniteDiff(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                             unsigned start_index, unsigned end_index, float delta = 0.001f);
    void OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset);
    float CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS);
    // float RenderDRShape(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count);
    // float CastRayWithGradShape(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS);

    std::vector<LiteImage::Image2D<float4>> m_imagesRef;
    std::vector<LiteImage::Image2D<float4>> m_images;
    std::vector<LiteMath::float4x4> m_worldViewRef;
    std::vector<LiteMath::float4x4> m_projRef;
    std::vector<float> m_dLoss_dS_tmp;
    std::vector<float> m_Opt_tmp;
    MultiRendererDRPreset m_preset_dr;
  };
}