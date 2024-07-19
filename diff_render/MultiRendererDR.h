#pragma once

#include "DR_common.h"
#include "BVH2DR.h"
#include "../Renderer/eye_ray.h"

namespace dr
{
  //enum DRLossFunction
  static constexpr unsigned DR_LOSS_FUNCTION_MSE =  0;

  //enum DRRenderMode
  static constexpr unsigned DR_RENDER_MODE_DIFFUSE = 0;
  static constexpr unsigned DR_RENDER_MODE_LAMBERT = 1;

  struct MultiRendererDRPreset
  {
    unsigned spp;
    unsigned dr_loss_function; //enum DRLossFunction
    unsigned dr_render_mode;   //enum DRRenderMode

    //optimization parameters (Adam optimizer)
    float opt_lr;
    float opt_beta_1;
    float opt_beta_2;
    float opt_eps;
    unsigned opt_iterations;
  };

  static MultiRendererDRPreset getDefaultPresetDR()
  {
    MultiRendererDRPreset preset;

    preset.spp = 1;
    preset.dr_loss_function = DR_LOSS_FUNCTION_MSE;
    preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;

    preset.opt_lr = 0.01f;
    preset.opt_beta_1 = 0.9f;
    preset.opt_beta_2 = 0.999f;
    preset.opt_eps = 1e-8f;
    preset.opt_iterations = 500;

    return preset;
  }
  
  class MultiRendererDR : public MultiRenderer
  {
  public:
    MultiRendererDR();
    void SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                      const std::vector<LiteMath::float4x4>& worldView, 
                      const std::vector<LiteMath::float4x4>& proj);
    void OptimizeColor(MultiRendererDRPreset preset, SdfSBS &sbs);

    const LiteImage::Image2D<float4> &getLastImage(unsigned view_id) const { return m_images[view_id]; }

  protected:
    void RenderDR(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count);
    void OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset);
    float CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS);

    std::vector<LiteImage::Image2D<float4>> m_imagesRef;
    std::vector<LiteImage::Image2D<float4>> m_images;
    std::vector<LiteMath::float4x4> m_worldViewRef;
    std::vector<LiteMath::float4x4> m_projRef;
    MultiRendererDRPreset m_preset_dr;
  };
}