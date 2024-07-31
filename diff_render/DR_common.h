#pragma once
#include <vector>
#include "LiteMath.h"

namespace dr
{
  using LiteMath::float3;
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
    uint32_t _pad;
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
  static constexpr unsigned DR_RENDER_MODE_DIFFUSE = 0;
  static constexpr unsigned DR_RENDER_MODE_LAMBERT = 1;
  static constexpr unsigned DR_RENDER_MODE_MASK    = 2;

  static constexpr unsigned DR_DEBUG_RENDER_MODE_PRIMITIVE = 100;
  static constexpr unsigned DR_DEBUG_RENDER_MODE_LINEAR_DEPTH     = 101;

  //enum DRDiffMode
  static constexpr unsigned DR_DIFF_MODE_DEFAULT     = 0;
  static constexpr unsigned DR_DIFF_MODE_FINITE_DIFF = 1;

  //enum DRReconstructionType
  static constexpr unsigned DR_RECONSTRUCTION_TYPE_COLOR    = 0;
  static constexpr unsigned DR_RECONSTRUCTION_TYPE_GEOMETRY = 1;

  //enum DRRayFlags
  static constexpr unsigned DR_RAY_FLAG_NO_DIFF         =      0;
  static constexpr unsigned DR_RAY_FLAG_DDIFFUSE_DCOLOR = 1 << 0;
  static constexpr unsigned DR_RAY_FLAG_DDIFFUSE_DPOS   = 1 << 1;
  static constexpr unsigned DR_RAY_FLAG_DNORM_DPOS      = 1 << 2;
  static constexpr unsigned DR_RAY_FLAG_BORDER          = 1 << 3;

  struct MultiRendererDRPreset
  {
    unsigned spp;
    unsigned border_spp;
    unsigned dr_loss_function;       //enum DRLossFunction
    unsigned dr_render_mode;         //enum DRRenderMode
    unsigned dr_diff_mode;           //enum DRDiffMode
    unsigned dr_reconstruction_type; //enum DRReconstructionType

    //optimization parameters (Adam optimizer)
    float opt_lr;
    float opt_beta_1;
    float opt_beta_2;
    float opt_eps;
    unsigned opt_iterations;
    unsigned image_batch_size;
  };
}