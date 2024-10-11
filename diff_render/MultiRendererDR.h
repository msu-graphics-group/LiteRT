#pragma once

#include "DR_common.h"
#include "BVH2DR.h"
#include "../Renderer/eye_ray.h"
#include <set>
#include <atomic>


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
    preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;

    preset.dr_raycasting_mask = DR_RAYCASTING_MASK_OFF;
    preset.dr_atomic_ders = -1;

    preset.border_spp = 256;
    preset.border_relax_eps = 1e-3f;
    preset.border_integral_mult = 1.0f;

    preset.opt_lr = 0.01f;
    preset.opt_beta_1 = 0.9f;
    preset.opt_beta_2 = 0.999f;
    preset.opt_eps = 1e-8f;
    preset.opt_iterations = 500;
    preset.image_batch_size = 1;

    preset.reg_function = DR_REG_FUNCTION_NONE;
    preset.reg_lambda = 0.25f;
    preset.reg_power  = 2.0f;

    preset.redistancing_enable = false;
    preset.redistancing_interval = 1;

    preset.debug_print = false;
    preset.debug_print_interval = 10;

    preset.debug_progress_images = DEBUG_PROGRESS_RAW;
    preset.debug_progress_interval = 100;

    preset.debug_forced_border = false;

    preset.debug_render_mode = DR_DEBUG_RENDER_MODE_NONE;
    preset.finite_diff_delta = 0.001f;
    preset.finite_diff_brightness = 100.0f;

    preset.debug_pd_images = false;
    preset.debug_pd_brightness = 0.1f;
    
    preset.debug_border_samples = false;
    preset.debug_border_samples_mega_image = false;
    preset.debug_border_save_normals = false;

    return preset;
  }
  
  class MultiRendererDR : public MultiRenderer
  {
  public:
    MultiRendererDR(uint32_t maxPrimitives = 10'000'000);
    void setBorderThickness(uint32_t thickness);
    void cleanMasks();
    void SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                      const std::vector<LiteMath::float4x4>& worldView, 
                      const std::vector<LiteMath::float4x4>& proj);
    void SetReference(const std::vector<LiteImage::Image2D<float4>>& images, 
                      const std::vector<LiteImage::Image2D<float4>>& masks, 
                      const std::vector<LiteMath::float4x4>& worldView, 
                      const std::vector<LiteMath::float4x4>& proj);
    void OptimizeFixedStructure(MultiRendererDRPreset preset, SdfSBS &sbs);
    void OptimizeGrid(unsigned start_grid_size, bool no_last_step_resize, std::vector<MultiRendererDRPreset> presets);

    const LiteImage::Image2D<float4> &getLastImage(unsigned view_id) const { return m_images[view_id]; }
    const LiteImage::Image2D<float4> &getLastDebugImage(unsigned view_id) const { return m_imagesDebug[view_id]; }
    const float *getLastdLoss_dS() const { return m_dLoss_dS_tmp.data(); }

  protected:
    float RenderDR(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                   LiteMath::float4* out_image_depth, LiteMath::float4* out_image_debug);
    float RenderDRFiniteDiff(const float4 *image_ref, LiteMath::float4* out_image, float *out_dLoss_dS, unsigned params_count,
                             unsigned start_index, unsigned end_index, float delta = 0.001f);
    void OptimizeStepAdam(unsigned iter, const float* dX, float *X, float *tmp, unsigned size, MultiRendererDRPreset preset);
    float CastRayWithGrad(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS,
                          LiteMath::float4* out_image_depth, LiteMath::float4* out_image_debug, PDFinalColor *out_pd_tmp);
    float CalculateBorderRayDerivatives(float sampling_pdf, const RayDiffPayload &payload, const CRT_HitDR &hit, float4 rayPosAndNear,
                                        float4 rayDirAndFar, const float4 *image_ref, LiteMath::float4 *out_image, float *out_dLoss_dS);
    void CastBorderRay(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS,
                       LiteMath::float4* out_image_debug);
    void CastBorderRaySVM(uint32_t tidX, const float4 *image_ref, LiteMath::float4* out_image, float* out_dLoss_dS,
                          LiteMath::float4* out_image_debug);
    float3 CalculateColor(const CRT_HitDR &hit);
    float3 CalculateColorWithGrad(const CRT_HitDR &hit, LiteMath::float3x3 &dColor_dDiffuse,
                                  LiteMath::float3x3 &dColor_dNorm);
    float3 ApplyDebugColor(float3 original_color, const CRT_HitDR &hit);
    void PreprocessRefImages(unsigned width, unsigned height, bool to_mask, float3 background_color = float3(0,0,0));
    void CreateRefImageMasks();
    void Regularization(float *out_dLoss_dS, unsigned params_count);

    float SolveEikonal(float3 axes_mins, float grid_spacing);

    float2 TransformWorldToScreenSpace(float4 pos);
    std::array<float4, 2> TransformWorldToScreenSpaceDiff(float4 pos);

    //prevents average gradient from being too big or too small for different image and scene sizes
    float getBaseGradientMult();

    std::vector<LiteImage::Image2D<float4>> m_imagesRefOriginal;
    std::vector<LiteImage::Image2D<float4>> m_imagesRefMask;
    std::vector<LiteImage::Image2D<float4>> m_imagesRef;

    //  It is needed to cast rays in image part where is object and not in empty space
    std::vector<std::vector<uint32_t>> masks;
    std::vector<uint32_t> process_mask;
    //  So, let's determine object frame size and then extend it on N pixels on every side
    uint32_t add_border;
    uint32_t mask_ind;

    std::vector<LiteImage::Image2D<float4>> m_images;
    std::vector<LiteImage::Image2D<float4>> m_imagesDepth;

    std::vector<LiteMath::float4x4> m_worldViewRef;
    std::vector<LiteMath::float4x4> m_projRef;
    std::vector<float> m_dLoss_dS_tmp;
    std::vector<std::atomic<uint32_t>> m_dLoss_dS_tmp_atomic_pos, m_dLoss_dS_tmp_atomic_neg;
    std::vector<float> m_Opt_tmp;
    std::vector<PDFinalColor> m_PD_tmp;
    std::vector<uint32_t> m_borderPixels;
    MultiRendererDRPreset m_preset_dr;

    std::vector<LiteImage::Image2D<float4>> m_imagesDebugPD;
    std::vector<LiteImage::Image2D<float4>> m_imagesDebug;
    std::vector<float4> samples_debug_color;
    std::vector<float4> samples_debug_pos_size;
  public:

    void Redistance(float *dist_in, uint3 size_in, float grid_spacing, uint32_t num_iters);
    std::atomic<uint32_t> border_rays_total{0};
    std::atomic<uint32_t> border_rays_hit{0};

    static constexpr unsigned MEGA_PIXEL_SIZE = 128;
    LiteImage::Image2D<float4> samples_mega_image;
  };
}