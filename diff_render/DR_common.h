#pragma once
#include <vector>
#include <functional>

#include "LiteMath.h"

struct SdfSBS;

namespace dr
{
  using LiteMath::cross;
  using LiteMath::dot;
  using LiteMath::float2;
  using LiteMath::float3;
  using LiteMath::float4;
  using LiteMath::float4x4;
  using LiteMath::int2;
  using LiteMath::inverse4x4;
  using LiteMath::normalize;
  using LiteMath::sign;
  using LiteMath::to_float3;
  using LiteMath::uint2;
  
  static constexpr uint32_t INVALID_INDEX        = 0xFFFFFFFF;
  static constexpr uint32_t MAX_PD_COUNT_DIST    = 64;
    static constexpr uint32_t MAX_PD_COUNT_COLOR = 8;
  struct PDColor
  {
    uint32_t index;
    float value;
  };

  struct PDDist
  {
    float3 dDiffuse;
    uint32_t index;
    float3 dNorm;
    float dDist;
  };

  struct PDFinalColor
  {
    float3 dFinalColor;
    uint32_t index;
  };

  struct CRT_HitDR
  {
    float    t;         ///< intersection distance from ray origin to object
    uint32_t primId; 
    uint32_t instId;
    uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
    float3   color;
    float    sdf;       //used only for missed_hit in RayDiffPayload, but we need padding here anyway
    float3   normal;
    uint32_t _pad1;
  };

  struct RayDiffPayload
  {
    CRT_HitDR missed_hit;
    uint32_t  missed_indices[8];
    float     missed_dSDF_dtheta[8];

#ifdef DEBUG_PAYLOAD_STORE_SDF
    std::vector<float> sdf_i;
#endif

    PDColor dDiffuse_dSc[MAX_PD_COUNT_COLOR]; //8 color points, PDs for diffuse (PDs for R,G,B are the same)
    PDDist  dDiffuseNormal_dSd[MAX_PD_COUNT_DIST]; //up to 64 distance points, PDs for diffuse and normal
  };

  //enum DRLossFunction
  static constexpr unsigned DR_LOSS_FUNCTION_MSE =  0;
  static constexpr unsigned DR_LOSS_FUNCTION_MAE =  1;

  //enum DRRenderMode
  static constexpr unsigned DR_RENDER_MODE_DIFFUSE          = 0;
  static constexpr unsigned DR_RENDER_MODE_LAMBERT          = 1;
  static constexpr unsigned DR_RENDER_MODE_MASK             = 2;
  static constexpr unsigned DR_RENDER_MODE_LINEAR_DEPTH     = 3;
  static constexpr unsigned DR_RENDER_MODE_NORMAL           = 4;

  //enum DRRayCastingMask
  static constexpr unsigned DR_RAYCASTING_MASK_OFF = 0;
  static constexpr unsigned DR_RAYCASTING_MASK_ON  = 1;

  //enum DRDebugRenderMode
  static constexpr unsigned DR_DEBUG_RENDER_MODE_NONE             = 0;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_PRIMITIVE        = 1;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL  = 2;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_BORDER_DETECTION = 3;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_BORDER_FOUND     = 4;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_AREA_INTEGRAL    = 5;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_DEPTH_STAT       = 6;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_DEPTH_DIFF       = 7;

  //enum DRDiffMode
  static constexpr unsigned DR_DIFF_MODE_DEFAULT     = 0;
  static constexpr unsigned DR_DIFF_MODE_FINITE_DIFF = 1;

  //enum DRReconstructionFlag
  static constexpr unsigned DR_RECONSTRUCTION_FLAG_COLOR    = 1 << 0;
  static constexpr unsigned DR_RECONSTRUCTION_FLAG_GEOMETRY = 1 << 1;

  //enum DRInputType
  static constexpr unsigned DR_INPUT_TYPE_COLOR        = 0;
  static constexpr unsigned DR_INPUT_TYPE_LINEAR_DEPTH = 1;

  //enum DRRayFlags
  static constexpr unsigned DR_RAY_FLAG_NO_DIFF         =      0;
  static constexpr unsigned DR_RAY_FLAG_DDIFFUSE_DCOLOR = 1 << 0;
  static constexpr unsigned DR_RAY_FLAG_DDIFFUSE_DPOS   = 1 << 1;
  static constexpr unsigned DR_RAY_FLAG_DNORM_DPOS      = 1 << 2;
  static constexpr unsigned DR_RAY_FLAG_BORDER          = 1 << 3;
  static constexpr unsigned DR_RAY_FLAG_DDIST_DPOS      = 1 << 4;

  //enum DRBorderSampling
  static constexpr unsigned DR_BORDER_SAMPLING_RANDOM = 0;
  static constexpr unsigned DR_BORDER_SAMPLING_SVM    = 1;

  //enum DRRegFunction
  static constexpr unsigned DR_REG_FUNCTION_NONE         = 0;
  static constexpr unsigned DR_REG_FUNCTION_LK_DENOISING = 1;

  static constexpr unsigned DEBUG_PROGRESS_NONE = 1000; //do not save intermediate images with optimization progress
  static constexpr unsigned DEBUG_PROGRESS_RAW  = 1001; //show intermediate images taken from diff render itself

  struct MultiRendererDRPreset
  {
    //main settings, altering the behavior of the renderer
    unsigned dr_loss_function;        //enum DRLossFunction
    unsigned dr_render_mode;          //enum DRRenderMode
    unsigned dr_diff_mode;            //enum DRDiffMode
    unsigned dr_reconstruction_flags; //enum DRReconstructionFlag
    unsigned dr_input_type;           //enum DRInputType
    unsigned dr_border_sampling;      //enum DRBorderSampling
    unsigned dr_raycasting_mask;      //enum DRRayCastingMask

    // main parameters
    unsigned spp;
    unsigned render_width;   // if 0, reference image width is used
    unsigned render_height;  // if 0, reference image width is used

    // border integral estimator parameters
    unsigned border_spp;
    float border_relax_eps;
    float border_integral_mult; //multiplier of border integral, it should be 1 in theory, but making it smaller apparently improves quality

    //optimization parameters (Adam optimizer)
    float opt_lr;
    float opt_beta_1;
    float opt_beta_2;
    float opt_eps;
    unsigned opt_iterations;
    unsigned image_batch_size;

    //regualrization settings and parameters
    unsigned reg_function; //enum DRRegFunction
    float reg_lambda;
    float reg_power;

    //redistancing settings and parameters
    bool redistancing_enable;
    unsigned redistancing_interval;

    //debug settings
    bool debug_print;                //wether to print loss and ETA during optimization or not
    unsigned debug_render_mode;      //enum DRDebugRenderMode
    unsigned debug_print_interval;   //how often to print
    unsigned debug_progress_images;  //render mode to progress images,either DEBUG_PROGRESS_NONE, DEBUG_PROGRESS_RAW or any MultiRenderMode
    unsigned debug_progress_interval;//how often to save progress images
    bool     debug_forced_border;    //disable border detection, force border integral estimation in every pixel
    
    //very heavy and specific debug modes. You probably shouldn't use them on a regular scene
    bool debug_pd_images;
    float debug_pd_brightness; //how bright the PD debug images should be
    
    bool debug_border_samples;
    bool debug_border_samples_mega_image;
  };

  void randomize_color(SdfSBS &sbs);
  void randomize_distance(SdfSBS &sbs, float delta);
  std::vector<float4x4> get_cameras_turntable(int count, float3 center, float radius, float height);
  std::vector<float4x4> get_cameras_uniform_sphere(int count, float3 center, float radius);
  SdfSBS circle_smallest_scene();
  SdfSBS circle_smallest_scene_colored();
  SdfSBS circle_medium_scene();
  SdfSBS circle_small_scene();
  SdfSBS circle_one_brick_scene();
  SdfSBS two_circles_scene();
  float circle_sdf(float3 center, float radius, float3 p);
  float3 gradient_color(float3 p);
  float3 single_color(float3 p);
  SdfSBS create_grid_sbs(unsigned brick_count, unsigned brick_size, 
                         std::function<float(float3)>  sdf_func,
                         std::function<float3(float3)> color_func);
}