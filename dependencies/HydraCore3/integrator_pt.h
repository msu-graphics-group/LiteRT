#ifndef TEST_CLASS_H
#define TEST_CLASS_H

#include "include/cglobals.h" // We assume that all code that should pe passed to kernels will be just included both for CPU and OpenCL
#include "include/crandom.h"
#include "include/clight.h"
#include "include/cmaterial.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include <utility>
#include <cfloat>

//#ifndef LITERT_RENDERER
//#include "CrossRT.h" // special include for ray tracing
//#else
#include "../../ISceneObject.h" // special include for ray tracing
#include "../../BVH/BVH2Common.h"
//#endif
#include "Image2d.h" // special include for textures

#include "spectrum.h"
#include "cam_plugin/CamPluginAPI.h"

using LiteImage::ICombinedImageSampler;

#ifndef KERNEL_SLICER
struct SceneInfo
{
  int width;
  int height;
  int spectral;
  uint32_t maxMeshes;
  uint32_t maxTotalVertices;
  uint32_t maxTotalPrimitives;
  uint32_t maxPrimitivesPerMesh;
  uint64_t memGeom;
  uint64_t memTextures;
};
#endif

class Integrator // : public DataClass, IRenderer
{
public:

  Integrator(int a_maxThreads = 1, std::vector<uint32_t> a_features = {}) : m_enabledFeatures(a_features), m_maxThreadId(a_maxThreads)
  {
    InitRandomGens(a_maxThreads);
    m_pAccelStruct = std::shared_ptr<ISceneObject>(CreateSceneRT(""), [](ISceneObject *p) { DeleteSceneRT(p); } );
    InitDataForGbuffer();
  }

  virtual ~Integrator() { m_pAccelStruct = nullptr; }

  virtual void SceneRestrictions(uint32_t a_restrictions[4]) const
  {
    a_restrictions[0] = g_lastSceneInfo.maxMeshes;
    a_restrictions[1] = g_lastSceneInfo.maxTotalVertices;
    a_restrictions[2] = g_lastSceneInfo.maxTotalPrimitives;
    a_restrictions[3] = g_lastSceneInfo.maxPrimitivesPerMesh;
  }

  static std::vector<uint32_t> PreliminarySceneAnalysis(const char* a_scenePath, const char* a_sncDir, SceneInfo* pSceneInfo);


  void SetSpectralMode(int a_mode) { m_spectral_mode = a_mode; }

  void InitRandomGens(int a_maxThreads);

  void SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct) { m_pAccelStruct = a_customAccelStruct; };
  virtual bool LoadScene(const char* a_scehePath, const char* a_sncDir);
  virtual void LoadSceneBegin(){} ///<! override it in derived class
  virtual void LoadSceneEnd(){}   ///<! override it in derived class

  void PackXY         (uint tidX, uint tidY);

  void CastSingleRay  (uint tid, float* out_color                [[size("tid*4")]]);        ///<! ray casting, draw diffuse or emisive color
  void RayTrace       (uint tid, uint channels, float* out_color [[size("tid*channels")]]); ///<! whitted ray tracing
  void NaivePathTrace (uint tid, uint channels, float* out_color [[size("tid*channels")]]); ///<! NaivePT
  void PathTrace      (uint tid, uint channels, float* out_color [[size("tid*channels")]]); ///<! MISPT and ShadowPT

  void PathTraceFromInputRays(uint tid, uint channels, 
                              const RayPosAndW* in_rayPosAndNear [[size("tid*channels")]], 
                              const RayDirAndT* in_rayDirAndFar  [[size("tid*channels")]],
                              float*            out_color        [[size("tid*channels")]]);
  
  struct GBufferPixel
  {
    float   depth;
    float   norm[3];
    float   texc[2];
    float   rgba[4];
    float   shadow;
    float   coverage;
    int32_t matId;
    int32_t objId;
    int32_t instId;
  };

  struct Map2DPiecewiseSample
  {
    float2 texCoord;
    float  mapPdf;
  };

  struct LensElementInterface 
  {
    float curvatureRadius;
    float thickness;
    float eta;
    float apertureRadius;
  };

#ifndef KERNEL_SLICER
  struct CamData 
  {
    float m_exposureMult  = 1.0f;
    float m_camLensRadius = 0.0f;
    float m_camTargetDist = 0.0f;
    int   m_camResponseSpectrumId[3] = {-1, -1, -1};
    int   m_camResponseType = 0; // 0 -- XYZ, 1 -- RGB
  
    uint m_enableOpticSim = 0;
    std::vector<LensElementInterface> m_lines;
    float2 m_physSize;
    float  m_diagonal;
    float  m_aspect;

    float4x4 m_proj;
    float4x4 m_worldView;
    float4x4 m_projInv;
    float4x4 m_worldViewInv;
  };

  void SetCamId(int a_camId);
  void AppendCamFromInternalVariables();
  std::vector<CamData> m_allCams;
#endif

  struct EyeRayData
  {
    float3 rayPos;
    float3 rayDir;
    float  timeSam;
    float  waveSam;
    float  cosTheta; // cos with sensor plane
    uint   x;        // screen x coord
    uint   y;        // screen y coord
  };

  virtual void EvalGBuffer(uint blockId, uint localId, GBufferPixel* out_gbuffer);
  virtual void GBufferReduction(uint blockId, uint blockSize, GBufferPixel* samples, GBufferPixel* out_gbuffer);

  virtual void EvalGBuffer(uint blockNum, GBufferPixel* out_gbuffer);
  virtual void kernelBE1D_EvalGBuffer(uint blockNum, GBufferPixel* out_gbuffer);

  virtual void PackXYBlock(uint tidX, uint tidY, uint a_passNum);
  virtual void CastSingleRayBlock(uint tid, float* out_color, uint a_passNum);
  virtual void NaivePathTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum);
  virtual void PathTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum);
  virtual void PathTraceFromInputRaysBlock(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, 
                                           float* out_color, uint a_passNum);
  virtual void RayTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum);

  virtual void CommitDeviceData() {}                                     // will be overriden in generated class
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]); // will be overriden in generated class

  virtual void UpdateMembersPlainData() {}                               // will be overriden in generated class, optional function
  //virtual void UpdateMembersVectorData() {}                              // will be overriden in generated class, optional function
  //virtual void UpdateMembersTexureData() {}                              // will be overriden in generated class, optional function

  virtual std::string GetResourcesRootDir() {return m_resourcesDir; }

  void SetResourcesDir(const std::string& a_dir) {m_resourcesDir = a_dir; }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY);

  void kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar);        // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
  virtual void kernel_InitEyeRay2(uint tid, float4* rayPosAndNear, float4* rayDirAndFar, float4* wavelengths,
                                  float4* accumColor, float4* accumuThoroughput, RandomGen* gen, uint* rayFlags, MisData* misData, float* time);

  void kernel_InitEyeRay3(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor,
                          float4* accumuThoroughput, uint* rayFlags);        

  void kernel_InitEyeRayFromInput(uint tid, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar,
                                  float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumuThoroughput, 
                                  RandomGen* gen, uint* rayFlags, MisData* misData, float4* wavelengths, float* time);

  void kernel_InitEyeRayGB(uint tidX, uint tidY, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar);  
  void kernel_GetRayGBuff(uint tidX, uint tidY, const Lite_Hit* pHit, const float2* bars, GBufferPixel* out_gbuffer);                                 

  bool kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                       Lite_Hit* out_hit, float2* out_bars);

  void kernel_RayTrace2(uint tid, uint bounce, const float4* rayPosAndNear, const float4* rayDirAndFar, const float* a_time,
                        float4* out_hit1, float4* out_hit2, float4* out_hit3, uint* out_instId, uint* rayFlags);

  void kernel_GetRayColor(uint tid, const Lite_Hit* in_hit, const float2* bars, const uint* in_pakedXY, float* out_color);

  void kernel_NextBounce(uint tid, uint bounce, const float4* in_hitPart1, const float4* in_hitPart2, const float4* in_hitPart3, 
                         const uint* in_instId, const float4* in_shadeColor, float4* rayPosAndNear, float4* rayDirAndFar, const float4* wavelengths,
                         float4* accumColor, float4* accumThoroughput, RandomGen* a_gen, MisData* a_prevMisData, uint* rayFlags);

  void kernel_RayBounce(uint tid, uint bounce, const float4* in_hitPart1, const float4* in_hitPart2, 
                        float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput, uint* rayFlags);

  void kernel_SampleLightSource(uint tid, const float4* rayPosAndNear, const float4* rayDirAndFar, const float4* wavelengths, 
                                const float4* in_hitPart1, const float4* in_hitPart2, const float4* in_hitPart3,
                                const uint* rayFlags, const float* a_time, uint bounce,
                                RandomGen* a_gen, float4* out_shadeColor);

  void kernel_HitEnvironment(uint tid, const uint* rayFlags, const float4* rayDirAndFar, const MisData* a_prevMisData, const float4* accumThoroughput,
                             float4* accumColor);

  void kernel_RealColorToUint32(uint tid, float4* a_accumColor, uint* out_color);

  virtual void kernel_ContributeToImage(uint tid, const uint* rayFlags, uint channels, const float4* a_accumColor, const RandomGen* gen, const uint* in_pakedXY, 
                                        const float4* wavelengths, float* out_color);

  void kernel_CopyColorToOutput(uint tid, uint channels, const float4* a_accumColor, const RandomGen* gen, 
                                float* out_color);

  void kernel_ContributeToImage3(uint tid, uint channels, const float4* a_accumColor, const uint* in_pakedXY, float* out_color);                               
  void kernel_ContributePathRayToImage3(float4* out_color, const std::vector<float4>& a_rayColor, std::vector<float3>& a_rayPos);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  static constexpr uint INTEGRATOR_STUPID_PT = 0;
  static constexpr uint INTEGRATOR_SHADOW_PT = 1;
  static constexpr uint INTEGRATOR_MIS_PT    = 2;

  static inline bool isDeadRay     (uint a_flags)  { return (a_flags & RAY_FLAG_IS_DEAD)        != 0; }
  static inline bool hasNonSpecular(uint a_flags)  { return (a_flags & RAY_FLAG_HAS_NON_SPEC)   != 0; }
  static inline bool hasInvNormal  (uint a_flags)  { return (a_flags & RAY_FLAG_HAS_INV_NORMAL) != 0; }
  static inline bool isOutOfScene  (uint a_flags)  { return (a_flags & RAY_FLAG_OUT_OF_SCENE)   != 0; }
  static inline bool terminateWavelngths(uint a_flags)  { return (a_flags & RAY_FLAG_WAVES_DIVERGED)   != 0; }

  static inline uint extractMatId(uint a_flags)    { return (a_flags & 0x00FFFFFF); }       
  static inline uint packMatId(uint a_flags, uint a_matId) { return (a_flags & 0xFF000000) | (a_matId & 0x00FFFFFF); }       
  static inline uint maxMaterials()             { return 0x00FFFFFF+1; }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Integrator Settings

  int2 GetResolution() const { return {m_winWidth, m_winHeight};}
  
  float GetAspect() const { return m_aspect; }

  float GetDiagonal() const { return m_diagonal; }
  void  SetDiagonal(float new_diagonal) { m_diagonal = new_diagonal; }

  float2 GetPhysSize() const { return m_physSize; }
  void  SetPhysSize(float2 new_size) { m_physSize = new_size; }

  void SetLines(const std::vector<LensElementInterface>& a_lines) {m_lines = a_lines;}

  void SetIntegratorType(const uint a_type) { m_intergatorType = a_type; }
  
  void SetFrameBufferSize(int a_width, int a_height) { m_fbWidth = a_width; m_fbHeight = a_height; }

  void SetViewport(int a_xStart, int a_yStart, int a_width, int a_height) 
  { 
    m_winStartX = a_xStart; 
    m_winStartY = a_yStart;
    
    m_winWidth  = a_width;  
    m_winHeight = a_height;
    m_packedXY.resize(m_winWidth*m_winHeight);

    if(m_fbWidth == 0 || m_fbHeight == 0)
      SetFrameBufferSize(a_width, a_height);

    const auto sizeX = a_width  - a_xStart;
    const auto sizeY = a_height - a_yStart;

    if(sizeX % 8 == 0 && sizeY % 8 == 0)
      m_tileSize = 8;
    else if(sizeX % 4 == 0 && sizeY % 4 == 0)
      m_tileSize = 4;
    else if(sizeX % 2 == 0 && sizeY % 2 == 0)
      m_tileSize = 2;
    else 
      m_tileSize = 1;
    
    m_maxThreadId = a_width*a_height;
  }
  
  void SetWorldView(const float4x4& a_mat)
  {
    m_worldView = a_mat;
    m_worldViewInv = LiteMath::inverse4x4(m_worldView);
  }
  
  void SetProj(const float4x4& a_mat) 
  { 
    m_proj = a_mat; 
    m_projInv = LiteMath::inverse4x4(m_proj);
  }
  
  static constexpr uint32_t FB_COLOR    = 0;
  static constexpr uint32_t FB_DIRECT   = 1;
  static constexpr uint32_t FB_INDIRECT = 2;
  void SetFrameBufferLayer(uint32_t a_layer) { m_renderLayer = a_layer; }
 
  uint GetSPP() const { return m_spp; } 

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////// \\ Integrator Settings

//protected:
  int m_winStartX   = 0;
  int m_winStartY   = 0;
  int m_winWidth    = 0;
  int m_winHeight   = 0;
  int m_fbWidth     = 0;
  int m_fbHeight    = 0;
  uint m_traceDepth = 10;
  
  uint m_renderLayer = FB_COLOR; ///!< when greater than 1, skip all bounce before this one: 2 for secondary light, 3 for thertiary and e.t.c. 
                                      ///!< TODO: don't account specular bounces(!)

  uint m_spp         = 1024;
  uint m_tileSize    = 8; ///!< screen mini tile, 2x2, 4x4 or 8x8 pixels.
  uint m_maxThreadId = m_winWidth*m_winHeight;

  LightSample LightSampleRev(int a_lightId, float3 rands, float3 illiminationPoint);
  float LightPdfSelectRev(int a_lightId);
  float4 LightIntensity(uint a_lightId, float4 a_wavelengths, float3 a_rayPos, float3 a_rayDir);

  /**
  \brief offset reflected ray position by epsilon;
  \param  a_lightId   - light id
  \param  ray_pos     - surface point from which we shoot shadow ray (i.e. ShadowRayPos)
  \param  ray_dir     - direction of the shadow ray                  (i.e. shadowRayDir)
  \param  lpos        - position on light surface
  \param  lnorm       - normal   on light surface
  \param  a_envPdf    - pdf for sampling environment which is evaluated else-where
  \return PdfW (solid-angle probability density) for sampling target light from point 'ray_pos' with direction 'ray_dir' to surface point on light (lpos, lnorm)
  */
  float  LightEvalPDF(int a_lightId, float3 ray_pos, float3 ray_dir, const float3 lpos, const float3 lnorm, float a_envPdf);

  float4 EnvironmentColor(float3 a_dir, float& outPdf);
  float3 BumpMapping(uint normalMapId, uint currMatId, float3 n, float3 tan, float2 tc);
  BsdfSample MaterialSampleWhitted(uint a_materialId, float3 v, float3 n, float2 tc);
  float3     MaterialEvalWhitted  (uint a_materialId, float3 l, float3 v, float3 n, float2 tc);

  virtual BsdfSample MaterialSampleAndEval(uint a_materialId, uint tid, uint bounce, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float3 tan, float2 tc, 
                                           MisData* a_misPrev, const uint a_currRayFlags);
                                    
  virtual BsdfEval   MaterialEval(uint a_materialId, float4 wavelengths, float3 l, float3 v, float3 n, float3 tan, float2 tc);

  uint32_t BlendSampleAndEval(uint a_materialId, uint tid, uint bounce, uint layer, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float2 tc, 
                              MisData* a_misPrev, BsdfSample* a_pRes);

  MatIdWeightPair BlendEval(MatIdWeight a_mat, float4 wavelengths, float3 l, float3 v, float3 n, float2 tc);

  uint RemapMaterialId(uint a_mId, int a_instId); 

  ////////////////////////////////////////////////////////////////////////////////////////////////

  void InitSceneMaterials(int a_numSpheres, int a_seed = 0);

  virtual void Update_m_materials(size_t a_first, size_t a_offet) {}
  virtual void Update_m_lights(size_t a_first, size_t a_offet) {}
  virtual void Update_m_matIdOffsets() {}

  std::vector<Material>         m_materials;  

  std::vector<uint32_t>         m_matIdOffsets;  ///< offset = m_matIdOffsets[geomId]
  std::vector<uint32_t>         m_matIdByPrimId; ///< matId  = m_matIdByPrimId[offset + primId]
  std::vector<uint32_t>         m_triIndices;    ///< (A,B,C) = m_triIndices[(offset + primId)*3 + 0/1/2]
  std::vector<uint32_t>         m_packedXY;
                                
  std::vector<uint32_t>         m_vertOffset;    ///< vertOffs = m_vertOffset[geomId]
  std::vector<float4>           m_vNorm4f;       ///< vertNorm = m_vNorm4f[vertOffs + vertId]
  std::vector<float4>           m_vTang4f;       ///< vertTang = m_vTang4f[vertOffs + vertId]
                                
  std::vector<int>              m_remapInst;
  std::vector<int>              m_allRemapLists;
  std::vector<int>              m_allRemapListsOffsets;
  std::vector<uint32_t>         m_instIdToLightInstId;
                                
  float4x4                      m_proj;
  float4x4                      m_worldView;
  float4x4                      m_projInv;
  float4x4                      m_worldViewInv;

  std::vector<RandomGen>        m_randomGens;
  std::vector<float4x4>         m_normMatrices;  ///< per instance normal matrix, local to world
  std::vector<float4x4>         m_normMatrices2; ///< per instance normal matrix for motion end point (used when motion blur is enabled)

  std::shared_ptr<ISceneObject> m_pAccelStruct = nullptr;

  int m_motionBlur = 0;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////// light source
  std::vector<LightSource> m_lights;
  std::vector<float>       m_pdfLightData;

  Map2DPiecewiseSample SampleMap2D(float3 rands, uint32_t a_tableOffset, int sizeX, int sizeY);

  float4 m_envColor      = float4{0.0f};
  float4 m_envSamRow0    = float4(1,0,0,0);
  float4 m_envSamRow1    = float4(0,1,0,0);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  float4 m_camRespoceRGB = float4(1,1,1,1);
  float3 SpectralCamRespoceToRGB(float4 specSamples, float4 waves, uint32_t rayFlags);

  uint  m_intergatorType = INTEGRATOR_STUPID_PT;
  int   m_spectral_mode  = 0;
  uint  m_envTexId       = uint(-1);
  uint  m_envLightId     = uint(-1);
  uint  m_envEnableSam   = 0;
  uint  m_envCamBackId   = uint(-1);

  /// @brief ////////////////////////////////////////////////////// cam variables
  float m_exposureMult   = 1.0f;
  float m_camLensRadius = 0.0f;
  float m_camTargetDist = 0.0f;
  static constexpr int CAM_RESPONCE_XYZ = 0;
  static constexpr int CAM_RESPONCE_RGB = 1;
  int   m_camResponseSpectrumId[3] = {-1, -1, -1};
  int   m_camResponseType = CAM_RESPONCE_XYZ; // 0 -- XYZ, 1 -- RGB

  /// @brief ////////////////////////////////////////////////////// optics sim
  

  uint m_enableOpticSim = 0;
  std::vector<LensElementInterface> m_lines;
  float2 m_physSize;
  float  m_diagonal;
  float  m_aspect;

  inline float LensRearZ()      const { return m_lines[0].thickness; }
  inline float LensRearRadius() const { return m_lines[0].apertureRadius; }         

  bool IntersectSphericalElement(float radius, float zCenter, float3 rayPos, float3 rayDir, 
                                 float *t, float3 *n) const;

  bool TraceLensesFromFilm(float3& rayPos, float3& rayDir) const;

  /////////////////////////////////////////////////////////////////

  float naivePtTime  = 0.0f;
  float shadowPtTime = 0.0f;
  float raytraceTime = 0.0f;
  float fromRaysPtTime = 0.0f;

  //// textures
  //
  std::vector< std::shared_ptr<ICombinedImageSampler> > m_textures; ///< all textures, right now represented via combined image/sampler

  std::vector<float> m_spec_values;
  std::vector<uint2> m_spec_offset_sz;

  std::vector<uint2> m_spec_tex_ids_wavelengths;
  std::vector<uint2> m_spec_tex_offset_sz;

  std::vector<float> m_cie_x;
  std::vector<float> m_cie_y;
  std::vector<float> m_cie_z;

  std::vector<float> m_precomp_coat_transmittance; //MI_ROUGH_TRANSMITTANCE_RES elements per material

  std::vector<float> m_films_thickness_vec;
  std::vector<uint> m_films_spec_id_vec;
  std::vector<float> m_films_eta_k_vec;
  std::vector<float> m_precomp_thin_films; //frenel precomputed data for thin films

  float4 SampleMatColorParamSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId);
  float4 SampleMatParamSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId);
  float4 SampleFilmsSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId, uint32_t layer);
  float4 SampleMatColorSpectrumTexture(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId, float2 texCoords);

  static constexpr uint32_t KSPEC_MAT_TYPE_GLTF       = 1;
  static constexpr uint32_t KSPEC_MAT_TYPE_GLASS      = 2;
  static constexpr uint32_t KSPEC_MAT_TYPE_CONDUCTOR  = 3;
  static constexpr uint32_t KSPEC_MAT_TYPE_DIFFUSE    = 4;
  static constexpr uint32_t KSPEC_MAT_TYPE_PLASTIC    = 5;
  static constexpr uint32_t KSPEC_FILMS_STACK_SIZE    = 6;
  static constexpr uint32_t KSPEC_MAT_TYPE_THIN_FILM  = 7;
 
  static constexpr uint32_t KSPEC_SPECTRAL_RENDERING  = 8;
  static constexpr uint32_t KSPEC_MAT_TYPE_BLEND      = 9;
  static constexpr uint32_t KSPEC_BLEND_STACK_SIZE    = 10;
  static constexpr uint32_t KSPEC_BUMP_MAPPING        = 11;
  static constexpr uint32_t KSPEC_MAT_TYPE_DIELECTRIC = 12;
  static constexpr uint32_t KSPEC_MAT_FOUR_TEXTURES   = 13;
  
  static constexpr uint32_t KSPEC_LIGHT_IES           = 14;
  static constexpr uint32_t KSPEC_LIGHT_ENV           = 15;

  static constexpr uint32_t KSPEC_MOTION_BLUR         = 16;  
  static constexpr uint32_t KSPEC_OPTIC_SIM           = 17;
  static constexpr uint32_t KSPEC_LIGHT_PROJECTIVE    = 18;
  static constexpr uint32_t KSPEC_SPD_TEX             = 19;

  static constexpr uint32_t TOTAL_FEATURES_NUM        = 20; // (!!!) DON'T rename it to KSPEC_TOTAL_FEATURES_NUM.

  //virtual std::vector<uint32_t> ListRequiredFeatures()  { return {1,1,1,1,1,1,1,1,4,1}; } 
  virtual std::vector<uint32_t> ListRequiredFeatures()  { return m_enabledFeatures; } 

  std::vector<uint32_t>         m_enabledFeatures;
  std::vector<uint32_t>         m_actualFeatures;
  std::string                   GetFeatureName(uint32_t a_featureId);

  static std::string g_lastScenePath;
  static std::string g_lastSceneDir;
  static SceneInfo   g_lastSceneInfo;

  std::string m_resourcesDir = ".";

  // for recording path "constant" parameters, override in dereved class
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual void RecordPixelRndIfNeeded(float4 offsets, float2 wt){}
  virtual void RecordRayHitIfNeeded(uint32_t bounceId, CRT_Hit hit){}
  virtual void RecordShadowHitIfNeeded(uint32_t bounceId, bool inShadow){}
  virtual void RecordLightRndIfNeeded(uint32_t bounceId, float4 rands) {}
  virtual void RecordMatRndNeeded(uint32_t bounceId, float4 rands){}
  virtual void RecordBlendRndNeeded(uint32_t bounceId, uint layer, float rand){}

  virtual float  GetRandomNumbersSpec(uint tid, RandomGen* a_gen);
  virtual float  GetRandomNumbersTime(uint tid, RandomGen* a_gen);
  virtual float4 GetRandomNumbersLens(uint tid, RandomGen* a_gen);
  virtual float4 GetRandomNumbersMats(uint tid, RandomGen* a_gen, int a_bounce);
  virtual float4 GetRandomNumbersLgts(uint tid, RandomGen* a_gen, int a_bounce);
  virtual float  GetRandomNumbersMatB(uint tid, RandomGen* a_gen, int a_bounce, int a_layer);
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual EyeRayData SampleCameraRay(RandomGen* pGen, uint tid);
  virtual uint       RandomGenId(uint tid);

  uint m_disableImageContrib = 0;

  virtual void ProgressBarStart();
  virtual void ProgressBarAccum(float a_progress);
  virtual void ProgressBarDone();

  virtual void _ProgressBarStart();
  virtual void _ProgressBarAccum(float a_progress);
  virtual void _ProgressBarDone();
  float m_currProgress    = 0.0f;
  float m_currProgressOld = 0.0f;
  
  static constexpr uint GBUFFER_SAMPLES = 16;
  std::vector<float2> m_qmcHammersley;
  virtual void InitDataForGbuffer();
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // struct IORVector
  // {
  //   complex value[KSPEC_FILMS_STACK_SIZE];
  // };
};

#endif
