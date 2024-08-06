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
  
  static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;
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

  struct PDShape
  {
    float3  f_in; // at y*
    float      t; // t = tReal
    float3 f_out; // at intersection/bg
    float    sdf; // sdf(pos + t * dir)
    uint32_t  indices[8];
    float dSDF_dtheta[8]; // separated, because the array can be passed to a derivative function
  };

  struct CRT_HitDR 
  {
    float    t;         ///< intersection distance from ray origin to object
    uint32_t primId; 
    uint32_t instId;
    uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
    float3   color;
    uint32_t _pad0;
    float3   normal;
    uint32_t _pad1;

    PDColor dDiffuse_dSc[8]; //8 color points, PDs for diffuse (PDs for R,G,B are the same)
    PDDist  dDiffuseNormal_dSd[8]; //8 distance points, PDs for diffuse and normal
    //    dNorm_dSc[8]; is zero, normal vector does not depend on color

  };

  //enum DRLossFunction
  static constexpr unsigned DR_LOSS_FUNCTION_MSE =  0;
  static constexpr unsigned DR_LOSS_FUNCTION_MAE =  1;

  //enum DRRenderMode
  static constexpr unsigned DR_RENDER_MODE_DIFFUSE          = 0;
  static constexpr unsigned DR_RENDER_MODE_LAMBERT          = 1;
  static constexpr unsigned DR_RENDER_MODE_MASK             = 2;
  static constexpr unsigned DR_RENDER_MODE_LINEAR_DEPTH     = 3;

  static constexpr unsigned DR_DEBUG_RENDER_MODE_PRIMITIVE        = 100;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL  = 101;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_BORDER_DETECTION = 102;

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

    // main parameters
    unsigned spp;
    unsigned render_width;   // if 0, reference image width is used
    unsigned render_height;  // if 0, reference image width is used

    // border integral estimator parameters
    unsigned border_spp;
    float border_relax_eps;
    float border_depth_threshold; //practically useless now
    float border_color_threshold;

    //optimization parameters (Adam optimizer)
    float opt_lr;
    float opt_beta_1;
    float opt_beta_2;
    float opt_eps;
    unsigned opt_iterations;
    unsigned image_batch_size;

    //debug settings
    bool debug_print;                //wether to print loss and ETA during optimization or not
    unsigned debug_print_interval;   //how often to print
    unsigned debug_progress_images;  //render mode to progress images,either DEBUG_PROGRESS_NONE, DEBUG_PROGRESS_RAW or any MultiRenderMode
    unsigned debug_progress_interval;//how often to save progress images
    
    //very heavy and specific debug modes. You probably shouldn't use them on a regular scene
    bool debug_pd_images;
    bool debug_border_samples;
    bool debug_border_samples_mega_image;
  };

  void randomize_color(SdfSBS &sbs);
  void randomize_distance(SdfSBS &sbs, float delta);
  std::vector<float4x4> get_cameras_turntable(int count, float3 center, float radius, float height);
  std::vector<float4x4> get_cameras_uniform_sphere(int count, float3 center, float radius);
  SdfSBS circle_smallest_scene();
  SdfSBS circle_medium_scene();
  SdfSBS circle_small_scene();
  SdfSBS circle_one_brick_scene();
  float circle_sdf(float3 center, float radius, float3 p);
  float3 gradient_color(float3 p);
  float3 single_color(float3 p);
  SdfSBS create_grid_sbs(unsigned brick_count, unsigned brick_size, 
                         std::function<float(float3)>  sdf_func,
                         std::function<float3(float3)> color_func);
}