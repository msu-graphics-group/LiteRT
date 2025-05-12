#include "integrator_pt.h"
#include "include/crandom.h"

#include "include/cmaterial.h"
#include "include/cmat_gltf.h"
#include "include/cmat_conductor.h"
#include "include/cmat_glass.h"
#include "include/cmat_film.h"
#include "include/cmat_diffuse.h"
#include "include/cmat_plastic.h"
#include "include/cmat_dielectric.h"

#include <chrono>
#include <string>

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;


uint32_t Integrator::BlendSampleAndEval(uint a_materialId, uint tid, uint bounce, uint layer, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float2 tc, 
                                        MisData* a_misPrev, BsdfSample* a_pRes)
{
  const float2 texCoordT = mulRows2x4(m_materials[a_materialId].row0[0], m_materials[a_materialId].row1[0], tc);
  const uint   texId     = m_materials[a_materialId].texid[0];
  const float4 weightDat = m_textures[texId]->sample(texCoordT);
  const float  weightTex = weightDat.x;
  const float  weight    = m_materials[a_materialId].data[BLEND_WEIGHT] * weightTex;

  const uint matId1 = m_materials[a_materialId].datai[0];
  const uint matId2 = m_materials[a_materialId].datai[1];

  uint32_t selectedMatId = matId1;
  const float select = GetRandomNumbersMatB(tid, a_gen, int(bounce), int(layer));
  RecordBlendRndNeeded(bounce, layer, select);

  if(select < weight)
  {
    a_pRes->pdf *= weight;
    a_pRes->val *= weight;
    selectedMatId = matId2;
  }
  else
  {
    a_pRes->pdf *= 1.0f - weight;
    a_pRes->val *= 1.0f - weight;
    selectedMatId = matId1;
  }

  return selectedMatId;
}

MatIdWeightPair Integrator::BlendEval(MatIdWeight a_mat, float4 wavelengths, float3 l, float3 v, float3 n, float2 tc)
{
  const float2 texCoordT = mulRows2x4(m_materials[a_mat.id].row0[0], m_materials[a_mat.id].row1[0], tc);
  const uint   texId     = m_materials[a_mat.id].texid[0];
  const float4 weightDat = m_textures[texId]->sample(texCoordT);
  
  const float  weightTex = weightDat.x;
  const float  weight    = m_materials[a_mat.id].data[BLEND_WEIGHT] * weightTex;

  const uint matId1      = m_materials[a_mat.id].datai[0];
  const uint matId2      = m_materials[a_mat.id].datai[1];

  MatIdWeight p1, p2;
  p1.id     = matId1;
  p1.weight = a_mat.weight * (1.0f - weight);
  p2.id     = matId2;
  p2.weight = a_mat.weight * weight;

  return make_weight_pair(p1, p2);
}

static inline float3 NormalMapTransform(const uint materialFlags, float3 normalFromTex)
{
  float3 normalTS = make_float3(2.0f * normalFromTex.x - 1.0f, 2.0f * normalFromTex.y - 1.0f, normalFromTex.z);

  if((materialFlags & FLAG_NMAP_INVERT_X) != 0)
    normalTS.x *= (-1.0f);

  if((materialFlags & FLAG_NMAP_INVERT_Y) != 0)
    normalTS.y *= (-1.0f);

  if((materialFlags & FLAG_NMAP_SWAP_XY) != 0)
  {
    float temp = normalTS.x;
    normalTS.x = normalTS.y;
    normalTS.y = temp;
  }

  return normalTS; // normalize(normalTS); // do we nedd this normalize here?
}

float3 Integrator::BumpMapping(uint normalMapId, uint currMatId, float3 n, float3 tan, float2 tc)
{
  const uint   mflags    = m_materials[currMatId].cflags;
  const float2 texCoordT = mulRows2x4(m_materials[currMatId].row0[1], m_materials[currMatId].row1[1], tc);
  const float4 normalTex = m_textures[normalMapId]->sample(texCoordT);
  const float3 normalTS  = NormalMapTransform(mflags, to_float3(normalTex));
  
  const float3   bitan = cross(n, tan);
  const float3x3 tangentTransform = make_float3x3(tan, bitan, n);

  return normalize(inverse3x3(tangentTransform)*normalTS);
}

BsdfSample Integrator::MaterialSampleAndEval(uint a_materialId, uint tid, uint bounce, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float3 tan, float2 tc, 
                                             MisData* a_misPrev, const uint a_currRayFlags)
{
  BsdfSample res;
  {
    res.val   = float4(0, 0, 0, 0);
    res.pdf   = 1.0f;
    res.dir   = float3(0,1,0);
    res.ior   = 1.0f;
    res.flags = a_currRayFlags;
    res.ior   = 1.0f;
  }

  uint32_t currMatId = a_materialId;
  uint     mtype     = m_materials[currMatId].mtype;
  uint     layer     = 0;
  while(KSPEC_MAT_TYPE_BLEND != 0 && mtype == MAT_TYPE_BLEND)
  {
    currMatId = BlendSampleAndEval(currMatId, tid, bounce, layer, wavelengths, a_gen, v, n, tc, a_misPrev, &res);
    mtype     = m_materials[currMatId].mtype;
    layer++;
  }
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  const uint normalMapId   = m_materials[currMatId].texid[1];
  const float3 geomNormal  = n;
        float3 shadeNormal = n;

  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
    shadeNormal = BumpMapping(normalMapId, currMatId, geomNormal, tan, tc);

  const float2 texCoordT = mulRows2x4(m_materials[currMatId].row0[0], m_materials[currMatId].row1[0], tc);
  const uint   texId     = m_materials[currMatId].texid[0];
  const float4 texColor  = m_textures[texId]->sample(texCoordT);
  const float4 rands     = GetRandomNumbersMats(tid, a_gen, int(bounce));
  const uint cflags      = m_materials[currMatId].cflags;
  RecordMatRndNeeded(bounce, rands);

  float4 fourScalarMatParams = float4(1,1,1,1);
  if(KSPEC_MAT_FOUR_TEXTURES != 0 && (cflags & FLAG_FOUR_TEXTURES) != 0)
  {
    const uint texId2  = m_materials[currMatId].texid[2];
    const uint texId3  = m_materials[currMatId].texid[3];
    
    const float2 texCoord2T = mulRows2x4(m_materials[currMatId].row0[2], m_materials[currMatId].row1[2], tc);
    const float2 texCoord3T = mulRows2x4(m_materials[currMatId].row0[3], m_materials[currMatId].row1[3], tc);

    const float4 color2 = m_textures[texId2]->sample(texCoord2T);
    const float4 color3 = m_textures[texId3]->sample(texCoord3T);
    
    if((cflags & FLAG_PACK_FOUR_PARAMS_IN_TEXTURE) != 0)
      fourScalarMatParams = color2;
    else
      fourScalarMatParams = float4(color2.x, color3.x, 1, 1);
  }

  switch(mtype)
  {
    case MAT_TYPE_GLTF:
    if(KSPEC_MAT_TYPE_GLTF != 0)
    {
      const float4 color = m_materials[currMatId].colors[GLTF_COLOR_BASE]*texColor;
      gltfSampleAndEval(m_materials.data() + currMatId, rands, v, shadeNormal, tc, color, fourScalarMatParams, &res);
    }
    break;
    case MAT_TYPE_GLASS: 
    if(KSPEC_MAT_TYPE_GLASS != 0)
    {
      glassSampleAndEval(m_materials.data() + currMatId, rands, v, geomNormal, tc, &res, a_misPrev);
    }
    break;
    case MAT_TYPE_CONDUCTOR:
    if(KSPEC_MAT_TYPE_CONDUCTOR != 0)
    {
      const float3 alphaTex = to_float3(texColor);    
      const float2 alpha    = float2(m_materials[currMatId].data[CONDUCTOR_ROUGH_V], m_materials[currMatId].data[CONDUCTOR_ROUGH_U]);
      const float4 etaSpec  = SampleMatParamSpectrum(currMatId, wavelengths, CONDUCTOR_ETA, 0);
      const float4 kSpec    = SampleMatParamSpectrum(currMatId, wavelengths, CONDUCTOR_K,   1);
      if(trEffectivelySmooth(alpha))
        conductorSmoothSampleAndEval(m_materials.data() + currMatId, etaSpec, kSpec, rands, v, shadeNormal, tc, &res);
      else
        conductorRoughSampleAndEval(m_materials.data() + currMatId, etaSpec, kSpec, rands, v, shadeNormal, tc, alphaTex, &res);
    }
    break;
    case MAT_TYPE_THIN_FILM:
    if(KSPEC_MAT_TYPE_THIN_FILM != 0)
    {
      const float3 alphaTex = to_float3(texColor);  
      const float2 alpha = float2(m_materials[currMatId].data[FILM_ROUGH_V], m_materials[currMatId].data[FILM_ROUGH_U]);

      uint t_offset = as_uint(m_materials[currMatId].data[FILM_THICKNESS_OFFSET]);
      uint layers = as_uint(m_materials[currMatId].data[FILM_LAYERS_COUNT]);
      const bool spectral_mode = wavelengths[0] > 0.0f;
      // sampling 3 wavelengths for naive RGB method
      float4 wavelengths_spec = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(645.f, 525.f, 445.f, 0.0f);
      float4 wavelengths_sample = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(525.f, 0.f, 0.f, 0.0f);
      float extIOR = m_materials[currMatId].data[FILM_ETA_EXT];
      complex intIOR = complex(
        SampleFilmsSpectrum(currMatId, wavelengths_sample, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, layers - 1)[0],
        SampleFilmsSpectrum(currMatId, wavelengths_sample, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, layers - 1)[0]
      );
      complex filmIOR = complex(
        SampleFilmsSpectrum(currMatId, wavelengths, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, 0)[0],
        SampleFilmsSpectrum(currMatId, wavelengths, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, 0)[0]
      );

      float thickness;
      if (as_uint(m_materials[currMatId].data[FILM_THICKNESS_MAP]) > 0u)
      {
        const uint texId  = m_materials[currMatId].texid[2];
        const float2 texCoord = mulRows2x4(m_materials[currMatId].row0[2], m_materials[currMatId].row1[2], tc);
        const float4 thickness_val = m_textures[texId]->sample(texCoord);
        float thickness_max = m_materials[currMatId].data[FILM_THICKNESS_MAX];
        float thickness_min = m_materials[currMatId].data[FILM_THICKNESS_MIN];
        thickness = (thickness_max - thickness_min) * thickness_val.x + thickness_min;
        //std::cout << fourScalarMatParams.x << " " << fourScalarMatParams.y << " " << fourScalarMatParams.z << " " << fourScalarMatParams.w << std::endl;
      }
      else
      {
        thickness = m_materials[currMatId].data[FILM_THICKNESS];
      }

      bool precomp_flag = as_uint(m_materials[currMatId].data[FILM_PRECOMP_FLAG]) > 0u;

      uint precomp_offset = precomp_flag ? as_uint(m_materials[currMatId].data[FILM_PRECOMP_OFFSET]) : 0;
      if(trEffectivelySmooth(alpha))
        filmSmoothSampleAndEval(m_materials.data() + currMatId, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, a_misPrev->ior, rands, v, n, tc, &res,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);
      else
        filmRoughSampleAndEval(m_materials.data() + currMatId, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, a_misPrev->ior, rands, v, n, tc, alphaTex, &res,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);

      //res.flags |= (specId < 0xFFFFFFFF) ? RAY_FLAG_WAVES_DIVERGED : 0;
      res.flags |= RAY_FLAG_WAVES_DIVERGED;

      a_misPrev->ior = res.ior;
    }
    break;
    case MAT_TYPE_DIFFUSE:
    if(KSPEC_MAT_TYPE_DIFFUSE != 0)
    {
      const float4 color = texColor;
      float4 reflSpec    = SampleMatColorSpectrumTexture(currMatId, wavelengths, DIFFUSE_COLOR, 0, tc);
      if(m_spectral_mode == 0)
        reflSpec *= color;
      
      diffuseSampleAndEval(m_materials.data() + currMatId, reflSpec, rands, v, shadeNormal, tc, &res);
    }
    break;
    case MAT_TYPE_PLASTIC:
    if(KSPEC_MAT_TYPE_PLASTIC != 0)
    {
      const float4 color = texColor;
      float4 reflSpec    = SampleMatColorSpectrumTexture(currMatId, wavelengths, PLASTIC_COLOR, 0, tc);
      // float4 reflSpec    = SampleMatColorParamSpectrum(currMatId, wavelengths, PLASTIC_COLOR, 0);
      if(m_spectral_mode == 0)
        reflSpec *= color;

      const uint precomp_id = m_materials[currMatId].datai[0];

      plasticSampleAndEval(m_materials.data() + currMatId, reflSpec, rands, v, shadeNormal, tc, &res,
                           m_precomp_coat_transmittance.data() + precomp_id * MI_ROUGH_TRANSMITTANCE_RES);
    }
    break;
    case MAT_TYPE_DIELECTRIC:
    if(KSPEC_MAT_TYPE_DIELECTRIC != 0)
    {
      const float4 intIORSpec = SampleMatParamSpectrum(currMatId, wavelengths, DIELECTRIC_ETA_INT, 0);
      const uint specId = m_materials[currMatId].spdid[0];
      dielectricSmoothSampleAndEval(m_materials.data() + currMatId, intIORSpec, a_misPrev->ior, rands, v, shadeNormal, tc, &res);

      res.flags |= (specId < 0xFFFFFFFF) ? RAY_FLAG_WAVES_DIVERGED : 0;

      a_misPrev->ior = res.ior;
    }
    break;
    default:
    break;
  }
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
  {
    const float cosThetaOut1 = std::abs(dot(res.dir, geomNormal));
    const float cosThetaOut2 = std::abs(dot(res.dir, shadeNormal));
    res.val *= cosThetaOut2 / std::max(cosThetaOut1, 1e-10f);
  }

  return res;
}

BsdfEval Integrator::MaterialEval(uint a_materialId, float4 wavelengths, float3 l, float3 v, float3 n, float3 tan, float2 tc)
{
  BsdfEval res;
  {
    res.val = float4(0,0,0,0);
    res.pdf   = 0.0f;
  }

  MatIdWeight currMat = make_id_weight(a_materialId, 1.0f);
  MatIdWeight material_stack[KSPEC_BLEND_STACK_SIZE];
  if(KSPEC_MAT_TYPE_BLEND != 0)
    material_stack[0] = currMat;
  int top = 0;
  bool needPop = false;

  do
  {
    if(KSPEC_MAT_TYPE_BLEND != 0)
    {
      if(needPop)
      {
        top--;
        currMat = material_stack[std::max(top, 0)];
      }
      else
        needPop = true; // if not blend, pop on next iter
    } 
    
    // BSDF is multiplied (outside) by old cosThetaOut.
    // When normal map is enables this becames wrong because normal is changed;
    // First : return cosThetaOut in sam;
    // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
    //
    const float3 geomNormal = n;
          float3 shadeNormal = n;
    float bumpCosMult = 1.0f; 
    const uint normalMapId = m_materials[currMat.id].texid[1];
    if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF) 
    {
      shadeNormal = BumpMapping(normalMapId, currMat.id, geomNormal, tan, tc);
      const float3 lDir     = l;     
      const float  clampVal = 1e-6f;  
      const float cosThetaOut1 = std::max(dot(lDir, geomNormal),  0.0f);
      const float cosThetaOut2 = std::max(dot(lDir, shadeNormal), 0.0f);
      bumpCosMult              = cosThetaOut2 / std::max(cosThetaOut1, clampVal);
      if (cosThetaOut1 <= 0.0f)
        bumpCosMult = 0.0f;
    }

    const float2 texCoordT = mulRows2x4(m_materials[currMat.id].row0[0], m_materials[currMat.id].row1[0], tc);
    const uint   texId     = m_materials[currMat.id].texid[0];
    const float4 texColor  = m_textures[texId]->sample(texCoordT);
    const uint   mtype     = m_materials[currMat.id].mtype;
    const uint   cflags    = m_materials[currMat.id].cflags;

    float4 fourScalarMatParams = float4(1,1,1,1);
    if(KSPEC_MAT_FOUR_TEXTURES != 0 && (cflags & FLAG_FOUR_TEXTURES) != 0)
    {
      const uint texId2  = m_materials[currMat.id].texid[2];
      const uint texId3  = m_materials[currMat.id].texid[3];

      const float2 texCoord2T = mulRows2x4(m_materials[currMat.id].row0[2], m_materials[currMat.id].row1[2], tc);
      const float2 texCoord3T = mulRows2x4(m_materials[currMat.id].row0[3], m_materials[currMat.id].row1[3], tc);

      const float4 color2 = m_textures[texId2]->sample(texCoord2T);
      const float4 color3 = m_textures[texId3]->sample(texCoord3T);
    
      if((cflags & FLAG_PACK_FOUR_PARAMS_IN_TEXTURE) != 0)
        fourScalarMatParams = color2;
      else
        fourScalarMatParams = float4(color2.x, color3.x, 1, 1);
    }

    BsdfEval currVal;
    {
      currVal.val = float4(0,0,0,0);
      currVal.pdf   = 0.0f;
    }
    switch(mtype)
    {
      case MAT_TYPE_GLTF:
      if(KSPEC_MAT_TYPE_GLTF != 0)
      {
        const float4 color     = (m_materials[currMat.id].colors[GLTF_COLOR_BASE]) * texColor;
        gltfEval(m_materials.data() + currMat.id, l, v, shadeNormal, tc, color, fourScalarMatParams, &currVal);
        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_GLASS:
      if(KSPEC_MAT_TYPE_GLASS != 0)
      {
        glassEval(m_materials.data() + currMat.id, l, v, geomNormal, tc, float3(0,0,0), &currVal);
        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_CONDUCTOR: 
      if(KSPEC_MAT_TYPE_CONDUCTOR != 0)
      {
        const float3 alphaTex  = to_float3(texColor);
        const float2 alpha     = float2(m_materials[currMat.id].data[CONDUCTOR_ROUGH_V], m_materials[currMat.id].data[CONDUCTOR_ROUGH_U]);

        if(!trEffectivelySmooth(alpha))
        {
          const float4 etaSpec = SampleMatParamSpectrum(currMat.id, wavelengths, CONDUCTOR_ETA, 0);
          const float4 kSpec   = SampleMatParamSpectrum(currMat.id, wavelengths, CONDUCTOR_K,   1);
          conductorRoughEval(m_materials.data() + currMat.id, etaSpec, kSpec, l, v, shadeNormal, tc, alphaTex, &currVal);
        }

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_THIN_FILM: 
      if(KSPEC_MAT_TYPE_THIN_FILM != 0)
      {
        const float3 alphaTex = to_float3(texColor);  
        const float2 alpha = float2(m_materials[currMat.id].data[FILM_ROUGH_V], m_materials[currMat.id].data[FILM_ROUGH_U]);

        if(!trEffectivelySmooth(alpha))
        {
          uint t_offset = as_uint(m_materials[currMat.id].data[FILM_THICKNESS_OFFSET]);
          uint layers = as_uint(m_materials[currMat.id].data[FILM_LAYERS_COUNT]);
          const bool spectral_mode = wavelengths[0] > 0.0f;
          // sampling 3 wavelengths for naive RGB method
          float4 wavelengths_spec = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(700.f, 525.f, 450.f, 0.0f);
          float4 wavelengths_sample = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(525.f, 0.f, 0.f, 0.0f);
          float extIOR = m_materials[currMat.id].data[FILM_ETA_EXT];
          complex intIOR = complex(
            SampleFilmsSpectrum(currMat.id, wavelengths_sample, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, layers - 1)[0],
            SampleFilmsSpectrum(currMat.id, wavelengths_sample, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, layers - 1)[0]
          );      
          complex filmIOR = complex(
            SampleFilmsSpectrum(currMat.id, wavelengths, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, 0)[0],
            SampleFilmsSpectrum(currMat.id, wavelengths, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, 0)[0]
          );

          float thickness;
          if (as_uint(m_materials[currMat.id].data[FILM_THICKNESS_MAP]) > 0u)
          {
            const uint texId  = m_materials[currMat.id].texid[2];
            const float2 texCoord = mulRows2x4(m_materials[currMat.id].row0[2], m_materials[currMat.id].row1[2], tc);
            const float4 thickness_val = m_textures[texId]->sample(texCoord);
            float thickness_max = m_materials[currMat.id].data[FILM_THICKNESS_MAX];
            float thickness_min = m_materials[currMat.id].data[FILM_THICKNESS_MIN];
            thickness = (thickness_max - thickness_min) * thickness_val.x + thickness_min;
          }
          else
          {
            thickness = m_materials[currMat.id].data[FILM_THICKNESS];
          }

          bool precomp_flag = as_uint(m_materials[currMat.id].data[FILM_PRECOMP_FLAG]) > 0u;
          uint precomp_offset = precomp_flag ? as_uint(m_materials[currMat.id].data[FILM_PRECOMP_OFFSET]) : 0;
          filmRoughEval(m_materials.data() + currMat.id, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, l, v, n, tc, alphaTex, &currVal,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);
        }

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_DIFFUSE:
      if(KSPEC_MAT_TYPE_DIFFUSE != 0)
      {
        const float4 color = texColor;
        float4 reflSpec    = SampleMatColorSpectrumTexture(currMat.id, wavelengths, DIFFUSE_COLOR, 0, tc);
        if(m_spectral_mode == 0)
          reflSpec *= color;        
        diffuseEval(m_materials.data() + currMat.id, reflSpec, l, v, shadeNormal, tc,  &currVal);

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_PLASTIC:
      if(KSPEC_MAT_TYPE_PLASTIC != 0)
      {
        const float4 color = texColor;
        float4 reflSpec    = SampleMatColorSpectrumTexture(currMat.id, wavelengths, PLASTIC_COLOR, 0, tc);
        // float4 reflSpec    = SampleMatColorParamSpectrum(currMat.id, wavelengths, PLASTIC_COLOR, 0);
        if(m_spectral_mode == 0)
          reflSpec *= color;
        const uint precomp_id = m_materials[currMat.id].datai[0];
        plasticEval(m_materials.data() + currMat.id, reflSpec, l, v, shadeNormal, tc, &currVal, 
                    m_precomp_coat_transmittance.data() + precomp_id * MI_ROUGH_TRANSMITTANCE_RES);

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_DIELECTRIC:
      if(KSPEC_MAT_TYPE_DIELECTRIC != 0)
      {
        dielectricSmoothEval(&res); // val and pdf are always zero
      }
      break;
      case MAT_TYPE_BLEND:
      if(KSPEC_MAT_TYPE_BLEND != 0)
      {
        auto childMats = BlendEval(currMat, wavelengths, l, v, geomNormal, tc);
        currMat = childMats.first;
        needPop = false;                        // we already put 'childMats.first' in 'currMat'
        if(top + 1 <= KSPEC_BLEND_STACK_SIZE)
        {
          material_stack[top] = childMats.second; // remember second mat in stack
          top++;
        }
      }
      break;
      default:
        break;
    }

  } while(KSPEC_MAT_TYPE_BLEND != 0 && top > 0);

  return res;
}

uint Integrator::RemapMaterialId(uint a_mId, int a_instId)
{
  const int remapListId  = m_remapInst[a_instId];
  if(remapListId == -1)
    return a_mId;

  const int r_offset     = m_allRemapListsOffsets[remapListId];
  const int r_size       = m_allRemapListsOffsets[remapListId+1] - r_offset;
  const int2 offsAndSize = int2(r_offset, r_size);
  
  uint res = a_mId;
  
  // for (int i = 0; i < offsAndSize.y; i++) // linear search version
  // {
  //   int idRemapFrom = m_allRemapLists[offsAndSize.x + i * 2 + 0];
  //   int idRemapTo   = m_allRemapLists[offsAndSize.x + i * 2 + 1];
  //   if (idRemapFrom == a_mId) {
  //     res = idRemapTo;
  //     break;
  //   }
  // }

  int low  = 0;
  int high = offsAndSize.y - 1;              // binary search version
  
  while (low <= high)
  {
    const int mid         = low + ((high - low) / 2);
    const int idRemapFrom = m_allRemapLists[offsAndSize.x + mid * 2 + 0];
    if (uint(idRemapFrom) >= a_mId)
      high = mid - 1;
    else //if(a[mid]<i)
      low = mid + 1;
  }

  if (high+1 < offsAndSize.y)
  {
    const int idRemapFrom = m_allRemapLists[offsAndSize.x + (high + 1) * 2 + 0];
    const int idRemapTo   = m_allRemapLists[offsAndSize.x + (high + 1) * 2 + 1];
    res                   = (uint(idRemapFrom) == a_mId) ? uint(idRemapTo) : a_mId;
  }

  return res;
} 
