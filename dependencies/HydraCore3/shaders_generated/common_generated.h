/////////////////////////////////////////////////////////////////////
/////////////  Required  Shader Features ////////////////////////////
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/////////////////// include files ///////////////////////////////////
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/////////////////// declarations in class ///////////////////////////
/////////////////////////////////////////////////////////////////////
#ifndef uint32_t
#define uint32_t uint
#endif
#define FLT_MAX 1e37f
#define FLT_MIN -1e37f
#define FLT_EPSILON 1e-6f
#define DEG_TO_RAD  0.017453293f
#define unmasked
#define half  float16_t
#define half2 f16vec2
#define half3 f16vec3
#define half4 f16vec4
bool  isfinite(float x)            { return !isinf(x); }
float copysign(float mag, float s) { return abs(mag)*sign(s); }

struct complex
{
  float re, im;
};

complex make_complex(float re, float im) { 
  complex res;
  res.re = re;
  res.im = im;
  return res;
}

complex to_complex(float re)              { return make_complex(re, 0.0f);}
complex complex_add(complex a, complex b) { return make_complex(a.re + b.re, a.im + b.im); }
complex complex_sub(complex a, complex b) { return make_complex(a.re - b.re, a.im - b.im); }
complex complex_mul(complex a, complex b) { return make_complex(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re); }
complex complex_div(complex a, complex b) {
  const float scale = 1 / (b.re * b.re + b.im * b.im);
  return make_complex(scale * (a.re * b.re + a.im * b.im), scale * (a.im * b.re - a.re * b.im));
}

complex real_add_complex(float value, complex z) { return complex_add(to_complex(value),z); }
complex real_sub_complex(float value, complex z) { return complex_sub(to_complex(value),z); }
complex real_mul_complex(float value, complex z) { return complex_mul(to_complex(value),z); }
complex real_div_complex(float value, complex z) { return complex_div(to_complex(value),z); }

complex complex_add_real(complex z, float value) { return complex_add(z, to_complex(value)); }
complex complex_sub_real(complex z, float value) { return complex_sub(z, to_complex(value)); }
complex complex_mul_real(complex z, float value) { return complex_mul(z, to_complex(value)); }
complex complex_div_real(complex z, float value) { return complex_div(z, to_complex(value)); }

float real(complex z) { return z.re;}
float imag(complex z) { return z.im; }
float complex_norm(complex z) { return z.re * z.re + z.im * z.im; }
float complex_abs(complex z) { return sqrt(complex_norm(z)); }
complex complex_sqrt(complex z) 
{
  float n = complex_abs(z);
  float t1 = sqrt(0.5f * (n + abs(z.re)));
  float t2 = 0.5f * z.im / t1;
  if (n == 0.0f)
    return to_complex(0.0f);
  if (z.re >= 0.0f)
    return make_complex(t1, t2);
  else
    return make_complex(abs(t2), copysign(t1, z.im));
}

#define Lite_Hit Lite_HitT
#define SurfaceHit SurfaceHitT
#define RandomGen RandomGenT
#define RefractResult RefractResultT
const uint RAY_FLAG_IS_DEAD = 0x80000000;
const uint RAY_FLAG_OUT_OF_SCENE = 0x40000000;
const uint RAY_FLAG_HIT_LIGHT = 0x20000000;
const uint RAY_FLAG_HAS_NON_SPEC = 0x10000000;
const uint RAY_FLAG_HAS_INV_NORMAL = 0x08000000;
const uint RAY_FLAG_WAVES_DIVERGED = 0x04000000;
const uint RAY_FLAG_PRIME_RAY_MISS = 0x02000000;
const uint RAY_FLAG_FIRST_NON_SPEC = 0x01000000;
const uint MI_ROUGH_TRANSMITTANCE_RES = 64;
const uint FILM_ANGLE_RES = 180;
const uint FILM_LENGTH_RES = 94;
const uint FILM_THICKNESS_RES = 32;
const float LAMBDA_MIN = 360.0f;
const float LAMBDA_MAX = 830.0f;
const float EPSILON_32 = 5.960464477539063E-8;
const uint SPECTRUM_SAMPLE_SZ = 4;
struct Lite_HitT
{
  float t;
  int   primId; 
  int   instId;
  int   geomId;
};
struct SurfaceHitT
{
  vec3 pos;
  vec3 norm;
  vec3 tang;
  vec2 uv;
};
const float GEPSILON = 1e-5f;
const float DEPSILON = 1e-20f;
struct MisData
{
  float matSamplePdf; ///< previous angle pdf (pdfW) that were used for sampling material. if < 0, then material sample was pure specular 
  float cosTheta;     ///< previous dot(matSam.dir, hit.norm)
  float ior;          ///< previous ior
  float dummy;        ///< dummy for 4 float
};
struct RandomGenT
{
  uvec2 state;

};
const uint LIGHT_GEOM_RECT = 1;
const uint LIGHT_GEOM_DISC = 2;
const uint LIGHT_GEOM_SPHERE = 3;
const uint LIGHT_GEOM_DIRECT = 4;
const uint LIGHT_GEOM_POINT = 5;
const uint LIGHT_GEOM_ENV = 6;
const uint LIGHT_DIST_LAMBERT = 0;
const uint LIGHT_DIST_OMNI = 1;
const uint LIGHT_DIST_SPOT = 2;
const uint LIGHT_FLAG_POINT_AREA = 1;
const uint LIGHT_FLAG_PROJECTIVE = 2;
struct LightSource
{
  mat4 matrix;         ///<! translation in matrix is always (0,0,0,1)
  mat4 iesMatrix;      ///<! translation in matrix is always (0,0,0,1), except projective light when it is used for light matrix ('mWorldLightProj')

  vec4   samplerRow0;    ///<! texture sampler, row0
  vec4   samplerRow1;    ///<! texture sampler, row1
  vec4   samplerRow0Inv; ///<! texture sampler, inverse matrix, row0
  vec4   samplerRow1Inv; ///<! texture sampler, inverse matrix, row1
  
  vec4   pos;            ///<! translation aclually stored here
  vec4   intensity;      ///<! brightress, i.e. screen value if light is visable directly
  vec4   norm;           ///<! light direction

  vec2   size;
  float    pdfA;
  uint     geomType;  ///<! LIGHT_GEOM_RECT, LIGHT_GEOM_DISC, LIGHT_GEOM_SPHERE, ...
  
  uint     distType;  ///<! LIGHT_DIST_LAMBERT, LIGHT_DIST_OMNI, ...
  uint     flags;     ///<! 
  uint     pdfTableOffset;
  uint     pdfTableSize;

  uint     specId;
  uint     texId;
  uint     iesId;
  float    mult;

  uint     pdfTableSizeX;
  uint     pdfTableSizeY;
  uint     camBackTexId;
  float    lightCos1;
  
  float    lightCos2;
  uint     matId;
  float    dummy2;
  float    dummy3;
};
struct LightSample
{
  vec3 pos;
  vec3 norm;
  float  pdf;
  bool   isOmni;
  bool   hasIES;
};
struct BsdfSample
{
  vec4 val;
  vec3 dir;
  float  pdf; 
  uint   flags;
  float  ior;
};
struct BsdfEval
{
  vec4 val;
  float  pdf; 
};
const uint GLTF_COMPONENT_LAMBERT = 1;
const uint GLTF_COMPONENT_COAT = 2;
const uint GLTF_COMPONENT_METAL = 4;
const uint GLTF_METAL_PERF_MIRROR = 8;
const uint GLTF_COMPONENT_ORENNAYAR = 16;
const uint FLAG_NMAP_INVERT_X = 32;
const uint FLAG_NMAP_INVERT_Y = 64;
const uint FLAG_NMAP_SWAP_XY = 128;
const uint FLAG_FOUR_TEXTURES = 256;
const uint FLAG_PACK_FOUR_PARAMS_IN_TEXTURE = 512;
const uint FLAG_INVERT_GLOSINESS = 1024;
const uint MAT_TYPE_GLTF = 1;
const uint MAT_TYPE_GLASS = 2;
const uint MAT_TYPE_CONDUCTOR = 3;
const uint MAT_TYPE_DIFFUSE = 4;
const uint MAT_TYPE_PLASTIC = 5;
const uint MAT_TYPE_BLEND = 6;
const uint MAT_TYPE_DIELECTRIC = 7;
const uint MAT_TYPE_THIN_FILM = 8;
const uint MAT_TYPE_LIGHT_SOURCE = 0xEFFFFFFF;
const uint RAY_EVENT_S = 1;
const uint RAY_EVENT_D = 2;
const uint RAY_EVENT_G = 4;
const uint RAY_EVENT_T = 8;
const uint RAY_EVENT_V = 16;
const uint RAY_EVENT_TOUT = 32;
const uint RAY_EVENT_TNINGLASS = 64;
const uint GLTF_COLOR_BASE = 0;
const uint GLTF_COLOR_COAT = 1;
const uint GLTF_COLOR_METAL = 2;
const uint GLTF_COLOR_LAST_IND = GLTF_COLOR_METAL;
const uint GLTF_FLOAT_MI_FDR_INT = 0;
const uint GLTF_FLOAT_MI_FDR_EXT = 1;
const uint GLTF_FLOAT_MI_SSW = 2;
const uint GLTF_FLOAT_ALPHA = 3;
const uint GLTF_FLOAT_GLOSINESS = 4;
const uint GLTF_FLOAT_IOR = 5;
const uint GLTF_FLOAT_ROUGH_ORENNAYAR = 6;
const uint GLTF_FLOAT_REFL_COAT = 7;
const uint GLTF_CUSTOM_LAST_IND = GLTF_FLOAT_REFL_COAT;
const uint GLASS_COLOR_REFLECT = 0;
const uint GLASS_COLOR_TRANSP = 1;
const uint GLASS_COLOR_LAST_IND = GLASS_COLOR_TRANSP;
const uint GLASS_FLOAT_GLOSS_REFLECT = 0;
const uint GLASS_FLOAT_GLOSS_TRANSP = 1;
const uint GLASS_FLOAT_IOR = 2;
const uint GLASS_CUSTOM_LAST_IND = GLASS_FLOAT_IOR;
const uint DIELECTRIC_COLOR_REFLECT = 0;
const uint DIELECTRIC_COLOR_TRANSMIT = 1;
const uint DIELECTRIC_COLOR_LAST_IND = DIELECTRIC_COLOR_TRANSMIT;
const uint DIELECTRIC_ETA_EXT = 0;
const uint DIELECTRIC_ETA_INT = 1;
const uint DIELECTRIC_ETA_INT_SPECID = 2;
const uint DIELECTRIC_CUSTOM_LAST_IND = DIELECTRIC_ETA_INT_SPECID;
const uint EMISSION_COLOR = 0;
const uint EMISSION_COLOR_LAST_IND = EMISSION_COLOR;
const uint EMISSION_MULT = 0;
const uint CONDUCTOR_COLOR = 0;
const uint CONDUCTOR_COLOR_LAST_IND = CONDUCTOR_COLOR;
const uint CONDUCTOR_ROUGH_U = 0;
const uint CONDUCTOR_ROUGH_V = 1;
const uint CONDUCTOR_ETA = 2;
const uint CONDUCTOR_K = 3;
const uint CONDUCTOR_CUSTOM_LAST_IND = CONDUCTOR_K;
const uint PLASTIC_COLOR = 0;
const uint PLASTIC_COLOR_LAST_IND = PLASTIC_COLOR;
const uint PLASTIC_ROUGHNESS = 0;
const uint PLASTIC_IOR_RATIO = 1;
const uint PLASTIC_SPEC_SAMPLE_WEIGHT = 2;
const uint PLASTIC_PRECOMP_REFLECTANCE = 3;
const uint PLASTIC_CUSTOM_LAST_IND = PLASTIC_PRECOMP_REFLECTANCE;
const uint DIFFUSE_COLOR = 0;
const uint DIFFUSE_COLOR_LAST_IND = DIFFUSE_COLOR;
const uint DIFFUSE_ROUGHNESS = 0;
const uint DIFFUSE_CUSTOM_LAST_IND = DIFFUSE_ROUGHNESS;
const uint BLEND_COLOR_LAST_IND = 0;
const uint BLEND_WEIGHT = 0;
const uint BLEND_CUSTOM_LAST_IND = BLEND_WEIGHT;
const uint FILM_COLOR = 0;
const uint FILM_COLOR_LAST_IND = FILM_COLOR;
const uint FILM_ROUGH_U = 0;
const uint FILM_ROUGH_V = 1;
const uint FILM_PRECOMP_FLAG = 2;
const uint FILM_PRECOMP_OFFSET = 3;
const uint FILM_ETA_OFFSET = 4;
const uint FILM_K_OFFSET = 5;
const uint FILM_ETA_SPECID_OFFSET = 6;
const uint FILM_K_SPECID_OFFSET = 7;
const uint FILM_ETA_EXT = 8;
const uint FILM_THICKNESS_OFFSET = 9;
const uint FILM_THICKNESS_MIN = 10;
const uint FILM_THICKNESS_MAX = 11;
const uint FILM_THICKNESS_MAP = 12;
const uint FILM_THICKNESS = 13;
const uint FILM_LAYERS_COUNT = 14;
const uint FILM_TRANSPARENT = 15;
const uint FILM_CUSTOM_LAST_IND = FILM_TRANSPARENT;
const uint COLOR_DATA_SIZE = 4;
const uint CUSTOM_DATA_SIZE = 16;
struct Material
{
  uint mtype;
  uint cflags;
  uint lightId;
  uint nonlinear;
  
  uint texid[4];
  uint spdid[4];
  uint datai[4];

  vec4 colors[COLOR_DATA_SIZE]; ///< colors data
  vec4 row0[4];                 ///< texture matrix
  vec4 row1[4];                 ///< texture matrix
      
  float  data[CUSTOM_DATA_SIZE]; ///< float, uint and custom data. Read uint: uint x = as_uint(data[INDEX]), write: data[INDEX] = as_float(x)
};
struct MatIdWeight
{
  uint id;
  float weight;
};
struct MatIdWeightPair
{
  MatIdWeight first;
  MatIdWeight second;
};
const uint PolarizationS = 0;
const uint PolarizationP = 1;
struct FrReflRefr
{
  float refl, refr;
};
const uint NONE = 0x00000000;
const uint BUILD_LOW = 0x00000001;
const uint BUILD_MEDIUM = 0x00000002;
const uint BUILD_HIGH = 0x00000004;
const uint BUILD_REFIT = 0x00000008;
const uint MOTION_BLUR = 0x00000010;
const uint BUILD_NOW = 32;
const uint BUILD_OPTIONS_MAX_ENUM = 0x7FFFFFFF;
struct CRT_Hit 
{
  float    t;         ///< intersection distance from ray origin to object
  uint primId; 
  uint instId;
  uint geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
  float    coords[4]; ///< custom intersection data; for triangles coords[0] and coords[1] stores baricentric coords (u,v)
};
const uint CRT_GEOM_MASK_AABB_BIT = 0x80000000;
const uint CRT_GEOM_MASK_AABB_BIT_RM = 0x7fffffff;
struct CRT_AABB
{
  vec4 boxMin;
  vec4 boxMax;
};
struct CRT_LeafInfo 
{
  uint aabbId; ///<! id of aabb/box  inside BLAS
  uint primId; ///<! id of primitive inside BLAS
  uint instId; ///<! instance id
  uint geomId; ///<! end-to-end index of custom geometry processerd with AABB
  uint rayxId; ///<! unique ray id (x coord on image)
  uint rayyId; ///<! unique ray id (y coord on image)
};
const uint SDF_PRIM_SPHERE = 0;
const uint SDF_PRIM_BOX = 1;
const uint SDF_PRIM_CYLINDER = 2;
const uint SDF_PRIM_SIREN = 3;
const uint NEURAL_SDF_MAX_LAYERS = 8;
const uint NEURAL_SDF_MAX_LAYER_SIZE = 1024;
const float SIREN_W0 = 30;
const uint SDF_SBS_ADAPT_MAX_UNITS = 1 << 15u;
const uint INVALID_IDX = 1u<<31u;
const uint SBS_IN_SIDE_X_NEG = 0u;
const uint SBS_IN_SIDE_X_POS = 1u;
const uint SBS_IN_SIDE_Y_NEG = 2u;
const uint SBS_IN_SIDE_Y_POS = 3u;
const uint SBS_IN_SIDE_Z_NEG = 4u;
const uint SBS_IN_SIDE_Z_POS = 5u;
const uint SDF_SBS_NODE_LAYOUT_UNDEFINED = 0 << 24u;
const uint SDF_SBS_NODE_LAYOUT_DX = 1 << 24u;
const uint SDF_SBS_NODE_LAYOUT_DX_UV16 = 2 << 24u;
const uint SDF_SBS_NODE_LAYOUT_DX_RGB8 = 3 << 24u;
const uint SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F = 4 << 24u;
const uint SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN = 5 << 24u;
const uint SDF_SBS_NODE_LAYOUT_MASK = 0xFF000000;
const uint OCTREE_FLAG_NODE_BORDER = 0;
const uint OCTREE_FLAG_NODE_EMPTY = 1;
const uint OCTREE_FLAG_NODE_FULL = 2;
const uint OCTREE_FLAG_NODE_PARENT = 3;
struct SdfObject
{
  uint type;          // from enum SdfPrimitiveType
  uint params_offset; // in parameters vector
  uint params_count;
  uint neural_id; // index in neural_properties if type is neural

  float distance_mult;
  float distance_add;
  uint complement; // 0 or 1
  uint _pad; 

  vec4 max_pos;    //vec4 to prevent padding issues
  vec4 min_pos;    //vec4 to prevent padding issues

  mat4 transform;
};
struct SdfConjunction
{
  vec4 max_pos;
  vec4 min_pos;
  uint offset; // in objects vector
  uint size;
  uint _pad[2];
};
struct NeuralDenseLayer
{
  uint offset;
  uint in_size;
  uint out_size;
};
struct NeuralProperties
{
  uint layer_count;
  NeuralDenseLayer layers[NEURAL_SDF_MAX_LAYERS];
};
struct SdfHit
{
  vec4 hit_pos;  // hit_pos.w < 0 if no hit, hit_pos.w > 0 otherwise
  vec4 hit_norm; // hit_norm.w can store different types of things for debug/visualization purposes
};
struct SdfFrameOctreeNode
{
  float values[8];
  uint offset; // offset for children (they are stored together). 0 offset means it's a leaf  
};
struct SdfCompactOctreeNode
{
  uint offset; // offset for children (they are stored together). 0 offset means it's a leaf
  uint flags;  // enum SdfOctreeNodeFlags
  uint values[2]; //compressed distance values, 1 byte per value
};
struct SdfSVSNode
{
  uint pos_xy; //position of voxel in it's LOD
  uint pos_z_lod_size; //size of it's LOD, (i.e. 2^LOD)
  uint values[2]; //compressed distance values, 1 byte per value
};
struct SdfSBSNode
{
  uint pos_xy; //position of start voxel of the block in it's LOD
  uint pos_z_lod_size; //size of it's LOD, (i.e. 2^LOD)
  uint data_offset; //offset in data vector for block with distance values, offset is in uint, not bytes 
  uint _pad;
};
struct SdfSBSAdaptNode
{
  uint pos_xy; // per-axis positions of the block's start voxel, in units. Limits: [0, 2^15). 2^15 means 1.0f (sdf scene's upper bound).
  uint pos_z_vox_size; // vox_size -- voxel size in units ('tis always a cube)
  uint data_offset;  //offset in data vector for block with distance values, offset is in uint, not bytes
  uint vox_count_xyz_pad; // _pad - 1 byte | voxel_numbers_xyz - 3 bytes
};
struct SdfSBSHeader
{
  uint brick_size;      //number of voxels in each brick, 1 to 16
  uint brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is required for tricubic filtration
  uint bytes_per_value; //1, 2 or 4 bytes per value is allowed
  uint aux_data;        //SdfSBSNodeLayout
};
struct SdfSBSAdaptHeader
{
  uint brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is required for tricubic filtration
  uint bytes_per_value; //1, 2 or 4 bytes per value is allowed
  uint aux_data;        //SdfSBSNodeLayout
  uint _pad;
};
struct SdfFrameOctreeTexNode
{
  float tex_coords[16];
  float values[8];
  uint offset; // offset for children (they are stored together). 0 offset means it's a leaf  
  uint material_id;
};
struct OTStackElement
{
  uint nodeId;
  uint info;
  uvec2 p_size;
};
const uint COCTREE_NODE_PACK_MODE_DEFAULT = 0;
const uint COCTREE_NODE_PACK_MODE_SIM_COMP_FULL = 1;
const uint COCTREE_NODE_PACK_MODE_SIM_COMP_SMALL = 2;
const uint COCTREE_LEAF_TYPE_NOT_A_LEAF = 0;
const uint COCTREE_LEAF_TYPE_GRID = 1;
const uint COCTREE_LEAF_TYPE_BIT_PACK = 2;
const uint COCTREE_LEAF_TYPE_SLICES = 3;
const uint COCTREE_LEAF_TYPE_BITS = 2;
const uint COCTREE_LEAF_TYPE_MASK = 0x3;
const uint COCTREE_LOD_LEAF_TYPE_SHIFT = 30;
const uint COCTREE_MAX_CHILD_INFO_SIZE = 2;
const uint COCTREE_USE_BEST_LEAF_TYPE = 1000;
struct COctreeV3Header
{
  uint brick_size;      //number of voxels in each brick, 1 to 16
  uint brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is for tricubic filtration or normals smoothing
  uint bits_per_value;  //6, 8, 10, 16, 32 bits per value is allowed
  uint uv_size;         //0 if COctreeV3 is not textured, 1 for default (16 for u and v) and 2 for more precision (32 for u and v, not supported)
  uint sim_compression; //0 or 1, indicates if similarity compression is used
  uint lods;            //0 or 1
  
  uint default_leaf_type; //enum COctreeLeafType
  uint fallback_leaf_type;//enum COctreeLeafType
  
  uint node_pack_mode;  
  // precomputed values for non-leaf nodes, fully determined by node_pack_mode
  uint uints_per_child_info;
  uint idx_mask;
  uint idx_sh;
  uint trans_off;
  uint rot_mask;
  uint add_mask;
};
struct OpenVDBHeader
{
  uint offset;
};
const uint TYPE_MESH_TRIANGLE = 0;
const uint TYPE_SDF_GRID = 1;
const uint TYPE_SDF_FRAME_OCTREE = 3;
const uint TYPE_RF_GRID = 4;
const uint TYPE_SDF_SVS = 5;
const uint TYPE_SDF_SBS = 6;
const uint TYPE_GS_PRIMITIVE = 7;
const uint TYPE_SDF_FRAME_OCTREE_TEX = 8;
const uint TYPE_SDF_SBS_TEX = 10;
const uint TYPE_SDF_SBS_COL = 11;
const uint TYPE_SDF_SBS_ADAPT = 12;
const uint TYPE_SDF_SBS_ADAPT_TEX = 14;
const uint TYPE_SDF_SBS_ADAPT_COL = 15;
const uint TYPE_NURBS = 16;
const uint TYPE_GRAPHICS_PRIM = 17;
const uint TYPE_COCTREE_V1 = 18;
const uint TYPE_COCTREE_V2 = 19;
const uint TYPE_COCTREE_V3 = 20;
const uint TYPE_CATMUL_CLARK = 21;
const uint TYPE_RIBBON = 22;
const uint TYPE_OPENVDB_GRID = 23;
const uint SH_TYPE = 24;
const uint GEOM_ID_MASK = 0x00FFFFFFu;
const uint DEVICE_CPU = 0;
const uint DEVICE_GPU = 1;
const uint DEVICE_GPU_RTX = 2;
const uint SDF_OCTREE_NODE_INTERSECT_ST = 0;
const uint SDF_OCTREE_NODE_INTERSECT_ANALYTIC = 1;
const uint SDF_OCTREE_NODE_INTERSECT_NEWTON = 2;
const uint SDF_OCTREE_NODE_INTERSECT_BBOX = 3;
const uint SDF_OCTREE_NODE_INTERSECT_IT = 4;
const uint MULTI_RENDER_MODE_MASK = 0;
const uint MULTI_RENDER_MODE_LAMBERT_NO_TEX = 1;
const uint MULTI_RENDER_MODE_DEPTH = 2;
const uint MULTI_RENDER_MODE_LINEAR_DEPTH = 3;
const uint MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH = 4;
const uint MULTI_RENDER_MODE_PRIMITIVE = 5;
const uint MULTI_RENDER_MODE_TYPE = 6;
const uint MULTI_RENDER_MODE_GEOM = 7;
const uint MULTI_RENDER_MODE_NORMAL = 8;
const uint MULTI_RENDER_MODE_BARYCENTRIC = 9;
const uint MULTI_RENDER_MODE_ST_ITERATIONS = 10;
const uint MULTI_RENDER_MODE_RF = 11;
const uint MULTI_RENDER_MODE_PHONG_NO_TEX = 12;
const uint MULTI_RENDER_MODE_GS = 13;
const uint MULTI_RENDER_MODE_RF_DENSITY = 14;
const uint MULTI_RENDER_MODE_TEX_COORDS = 15;
const uint MULTI_RENDER_MODE_DIFFUSE = 16;
const uint MULTI_RENDER_MODE_LAMBERT = 17;
const uint MULTI_RENDER_MODE_PHONG = 18;
const uint MULTI_RENDER_MODE_HSV_DEPTH = 19;
const uint MULTI_RENDER_MODE_LOD = 20;
const uint NORMAL_MODE_GEOMETRY = 0;
const uint NORMAL_MODE_VERTEX = 1;
const uint NORMAL_MODE_SDF_SMOOTHED = 2;
const uint RAY_GEN_MODE_REGULAR = 0;
const uint RAY_GEN_MODE_RANDOM = 1;
const uint INTERPOLATION_MODE_TRILINEAR = 0;
const uint INTERPOLATION_MODE_TRICUBIC = 1;
const uint REPRESENTATION_MODE_SURFACE = 0;
const uint REPRESENTATION_MODE_VOLUME = 1;
struct MultiRenderPreset
{
  uint render_mode;        //enum MultiRenderMode
  uint sdf_node_intersect; //enum SdfNodeIntersect
  uint normal_mode;        //enum NormalMode
  uint ray_gen_mode;       //enum RayGenMode
  uint interpolation_mode; //enum InterpolationMode
  uint representation_mode;//enum RepresentationMode

  uint spp;                //samples per pixel
  uint fixed_lod;          //0 or 1, use fixed level of detail or dynamic (based on distance)
  float level_of_detail;       //level of detail that you get on an object 1 meter away from you. It is inverted (worst LOD is 0!)
};
const uint LEAF_NORMAL = 0xFFFFFFFF;
const uint LEAF_EMPTY = 0xFFFFFFFD;
const uint ESCAPE_ROOT = 0xFFFFFFFE;
struct BVHNode 
  {
    vec3 boxMin;
    uint   leftOffset; //!< please note that when LEAF_BIT (0x80000000) is set in leftOffset, this node is a leaf
    vec3 boxMax;
    uint   escapeIndex;
  };
struct BVHNodePair
  { 
    BVHNode left;
    BVHNode right;
  };
struct BoxHit
{
  uint id;
  float tHit;
};
const int LBVH_MAXHITS = 32;
const int STACK_SIZE = 80;
const uint START_MASK = 0x00FFFFFF;
const uint END_MASK = 0xFF000000;
const uint SIZE_MASK = 0x7F000000;
const uint LEAF_BIT = 0x80000000;
const uint EMPTY_NODE = 0x7fffffff;
struct NURBSHeader
{
  int offset;
  int p, q;
  int uknots_cnt, vknots_cnt;
};
struct NURBS_HitInfo
{
  bool hitten;
  vec3 point;
  vec3 normal;
  vec2 uv;
};
const uint GRAPH_PRIM_POINT = 0u;
const uint GRAPH_PRIM_LINE = 1u;
const uint GRAPH_PRIM_LINE_SEGMENT = 2u;
const uint GRAPH_PRIM_LINE_SEGMENT_DIR = 3u;
const uint GRAPH_PRIM_BOX = 4u;
const uint GRAPH_PRIM_POINT_COLOR = 5u;
const uint GRAPH_PRIM_LINE_COLOR = 6u;
const uint GRAPH_PRIM_LINE_SEGMENT_COLOR = 7u;
const uint GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR = 8u;
const uint GRAPH_PRIM_BOX_COLOR = 9u;
struct GraphicsPrimHeader
{
    uint prim_type; // enum GraphicsPrimType
    vec3 color; // used when prim_type isn't GRAPH_PRIM_*_COLOR
};
struct CatmulClarkHeader
{
  /*
  * One object information
  * Offsets in data array (see BVHRT structure)
  * No pointers, vectors, etc
  * Only trivial types: int, float, ..., struct SomeStruct{ int, int }, etc
  */
  uint data_offset;
};
struct RibbonHeader
{
  /*
  * One object information
  * Offsets in data array (see BVHRT structure)
  * No pointers, vectors, etc
  * Only trivial types: int, float, ..., struct SomeStruct{ int, int }, etc
  */
  uint data_offset;
};
struct GeomData
{
  vec4 boxMin;
  vec4 boxMax;

  uint bvhOffset;
  uint type; // enum GeomType
  uvec2 offset;
};
const uint NURBS_MAX_DEGREE = 10;
struct InstanceData
{
  vec4 boxMin;
  vec4 boxMax;
  uint geomId;
  uint instStart;
  uint instEnd;
  uint _pad[5];
  mat4 transform;
  mat4 transformInv;
  mat4 transformInvTransposed; //for normals
};
const uint ROT_COUNT = 48;
const uint m_bitcount[256] = {
    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
  };
struct RayPosAndW 
{
  float origin[3]; ///<! ray origin, x,y,z
  float wave;      ///<! wavelength
};
struct RayDirAndT 
{
  float direction[3]; ///<! normalized ray direction, x,y,z
  float time;         ///<! time in ... 
};
struct GBufferPixel
  {
    float   depth;
    float   norm[3];
    float   texc[2];
    float   rgba[4];
    float   shadow;
    float   coverage;
    int matId;
    int objId;
    int instId;
  };
struct Map2DPiecewiseSample
  {
    vec2 texCoord;
    float  mapPdf;
  };
struct LensElementInterface 
  {
    float curvatureRadius;
    float thickness;
    float eta;
    float apertureRadius;
  };
struct EyeRayData
  {
    vec3 rayPos;
    vec3 rayDir;
    float  timeSam;
    float  waveSam;
    float  cosTheta; // cos with sensor plane
    uint   x;        // screen x coord
    uint   y;        // screen y coord
  };
const uint INTEGRATOR_STUPID_PT = 0;
const uint INTEGRATOR_SHADOW_PT = 1;
const uint INTEGRATOR_MIS_PT = 2;
const uint FB_COLOR = 0;
const uint FB_DIRECT = 1;
const uint FB_INDIRECT = 2;
const int CAM_RESPONCE_XYZ = 0;
const int CAM_RESPONCE_RGB = 1;
const uint TOTAL_FEATURES_NUM = 20;
const uint GBUFFER_SAMPLES = 16;
struct RefractResultT
{
  vec3 rayDir;
  bool   success;
  float  eta;

};

#ifndef SKIP_UBO_INCLUDE
#include "include/Integrator_generated_ubo.h"
#endif

/////////////////////////////////////////////////////////////////////
/////////////////// local functions /////////////////////////////////
/////////////////////////////////////////////////////////////////////

mat4 translate4x4(vec3 delta)
{
  return mat4(vec4(1.0, 0.0, 0.0, 0.0),
              vec4(0.0, 1.0, 0.0, 0.0),
              vec4(0.0, 0.0, 1.0, 0.0),
              vec4(delta, 1.0));
}

mat4 rotate4x4X(float phi)
{
  return mat4(vec4(1.0f, 0.0f,  0.0f,           0.0f),
              vec4(0.0f, +cos(phi),  +sin(phi), 0.0f),
              vec4(0.0f, -sin(phi),  +cos(phi), 0.0f),
              vec4(0.0f, 0.0f,       0.0f,      1.0f));
}

mat4 rotate4x4Y(float phi)
{
  return mat4(vec4(+cos(phi), 0.0f, -sin(phi), 0.0f),
              vec4(0.0f,      1.0f, 0.0f,      0.0f),
              vec4(+sin(phi), 0.0f, +cos(phi), 0.0f),
              vec4(0.0f,      0.0f, 0.0f,      1.0f));
}

mat4 rotate4x4Z(float phi)
{
  return mat4(vec4(+cos(phi), sin(phi), 0.0f, 0.0f),
              vec4(-sin(phi), cos(phi), 0.0f, 0.0f),
              vec4(0.0f,      0.0f,     1.0f, 0.0f),
              vec4(0.0f,      0.0f,     0.0f, 1.0f));
}

mat4 inverse4x4(mat4 m) { return inverse(m); }
vec3 mul4x3(mat4 m, vec3 v) { return (m*vec4(v, 1.0f)).xyz; }
vec3 mul3x3(mat4 m, vec3 v) { return (m*vec4(v, 0.0f)).xyz; }

mat3 make_float3x3(vec3 a, vec3 b, vec3 c) { // different way than mat3(a,b,c)
  return mat3(a.x, b.x, c.x,
              a.y, b.y, c.y,
              a.z, b.z, c.z);
}

vec4 cross3(vec4 a, vec4 b) { return vec4(cross(a.xyz, b.xyz), 1.0f); }

struct Box4f 
{ 
  vec4 boxMin; 
  vec4 boxMax;
};  


#define KGEN_FLAG_RETURN            1
#define KGEN_FLAG_BREAK             2
#define KGEN_FLAG_DONT_SET_EXIT     4
#define KGEN_FLAG_SET_EXIT_NEGATIVE 8
#define KGEN_REDUCTION_LAST_STEP    16
#define IMAGE2D_H 
#define DISABLE_RF_GRID 1
#define DISABLE_SDF_HP 1
#define LITERT_RENDERER 1
#define DISABLE_SDF_FRAME_OCTREE_TEX 1
#define DISABLE_SDF_SBS_ADAPT 1
#define DISABLE_SDF_GRID 1
#define DISABLE_GS_PRIMITIVE 1
#define DISABLE_GRAPHICS_PRIM 1
#define MAXFLOAT FLT_MAX
#define BASIC_PROJ_LOGIC_H 
#define TEST_CLASS_H 
#define RTC_RANDOM 
#define SPECTRUM_H 
#define CFLOAT_GUARDIAN 
#define RTC_MATERIAL 

float Cos2Theta(vec3 w) {
  return w.z * w.z;
}

float Sin2Theta(vec3 w) {
  return max(0.0f, 1.0f - Cos2Theta(w));
}

float safe_sqrt(float val) {
  return sqrt(max(val, 0.0f));
}

float SinTheta(vec3 w) {
  return safe_sqrt(Sin2Theta(w));
}

float Tan2Theta(vec3 w) {
  return Sin2Theta(w) / Cos2Theta(w);
}

float SinPhi(vec3 w) {
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 0 : clamp(w.y / sinTheta, -1.0f, 1.0f);
}

float CosPhi(vec3 w) {
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 1 : clamp(w.x / sinTheta, -1.0f, 1.0f);
}

float trLambda(vec3 w, vec2 alpha) {
  float tan2Theta = Tan2Theta(w);
  if (isinf(tan2Theta))
    return 0;
  float alpha2 = (CosPhi(w) * alpha.x) * (CosPhi(w) * alpha.x) + (SinPhi(w) * alpha.y) * (SinPhi(w) * alpha.y);
  return (safe_sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) / 2.0f;
}

uint SBS_v_to_i(float i, float j, float k, uint v_size, uint pad) {
  return (uint(i)+pad)*v_size*v_size + (uint(j)+pad)*v_size + (uint(k)+pad);
}

void CoordinateSystemV2(in vec3 n, inout vec3 s, inout vec3 t) {
  float sign = n.z >= 0 ? 1.0f : -1.0f;
  float a    = -(1.0f / (sign + n.z));
  float b    = n.x * n.y * a;

  float tmp = (n.z >= 0 ? n.x * n.x * a : -n.x * n.x * a);
  (s) = vec3(tmp + 1.0f,n.z >= 0 ? b : -b,n.z >= 0 ? -n.x : n.x);
  
  (t) = vec3(b,n.y * n.y * a + sign,-n.y);
}

int uknots_offset(NURBSHeader h) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * (h.uknots_cnt-1) * (h.vknots_cnt-1);
}

uint EXTRACT_COUNT(uint a_leftOffset) { return (a_leftOffset & SIZE_MASK) >> 24; }

float sqr(float val) {
  return val * val;
}

vec2 square_to_uniform_disk_concentric(in vec2 s) {
  float x = 2.f * s.x - 1.f,
        y = 2.f * s.y - 1.f;

  float phi, r;
  if (x == 0 && y == 0) 
  {
    r = phi = 0;
  } 
  else if (x * x > y * y) 
  {
    r = x;
    phi = (M_PI / 4.f) * (y / x);
  } 
  else 
  {
    r = y;
    phi = (M_PI / 2.f) - (x / y) * (M_PI / 4.f);
  }
  return vec2(r * cos(phi),r * sin(phi));
}

float trG1(vec3 w, vec2 alpha) { 
  return 1.0f / (1.0f + trLambda(w, alpha)); 
}

float trD(vec3 wm, vec2 alpha) {
  float tan2Theta = Tan2Theta(wm);
  if (isinf(tan2Theta))
      return 0;
  float cos4Theta = Cos2Theta(wm) * Cos2Theta(wm);
  if (cos4Theta < 1e-16f)
      return 0;
  float e = tan2Theta * ((CosPhi(wm) / alpha.x) * (CosPhi(wm) / alpha.x) + (SinPhi(wm) / alpha.y) * (SinPhi(wm) / alpha.y));
  return 1.0f / (M_PI * alpha.x * alpha.y * cos4Theta * (1 + e) * (1 + e));
}

float AbsCosTheta(vec3 w) {
  return abs(w.z);
}

uint NextState(inout RandomGen gen) {
  const uint x = (gen.state).x * 17 + (gen.state).y * 13123;
  (gen.state).x = (x << 13) ^ x;
  (gen.state).y ^= (x << 7);
  return x;
}

float sin_theta_2(in vec3 v) { 
  return v.x * v.x + v.y * v.y; 
}

complex filmPhaseDiff(complex cosTheta, complex eta, float thickness, float lambda) {
  return complex_div(complex_mul(complex_mul(real_mul_complex(4 * M_PI,eta),cosTheta),to_complex(thickness)),to_complex(lambda));
}

complex FrComplexRefl(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT, uint polarization) {
  if (complex_norm(cosThetaI) < 1e-6f) //approximated
  {
    return make_complex(-1,0);
  }
  if (polarization == PolarizationS)
  {
    return complex_div((complex_sub(complex_mul(iorI,cosThetaI),complex_mul(iorT,cosThetaT))),(complex_add(complex_mul(iorI,cosThetaI),complex_mul(iorT,cosThetaT))));
  }
  else if (polarization == PolarizationP)
  {
    return complex_div((complex_sub(complex_mul(iorT,cosThetaI),complex_mul(iorI,cosThetaT))),(complex_add(complex_mul(iorT,cosThetaI),complex_mul(iorI,cosThetaT))));
  }
  return to_complex(1.0f);
}

float getRefractionFactor(float cosThetaI, complex cosThetaT, complex iorI, complex iorT) {
  complex mult = complex_mul(cosThetaT,iorT);
  if (cosThetaI <= 1e-6f || mult.im > 1e-6f)
  {
    return 0;
  }
  return mult.re / (iorI.re * cosThetaI);
}

vec3 faceforward(const vec3 n, const vec3 v) { return (dot(n, v) < 0.f) ? (-1.0f)*n : n; }

bool Quadratic(float A, float B, float C, inout float t0, inout float t1) {
  // Find quadratic discriminant
  double discrim = double(B) * double(B) - 4. * double(A) * double(C);
  if (discrim < 0.) 
    return false;
  double rootDiscrim = sqrt(discrim);
  float floatRootDiscrim   = float(rootDiscrim);
  // Compute quadratic _t_ values
  float q;
  if (float(B) < 0)
      q = -.5 * (B - floatRootDiscrim);
  else
      q = -.5 * (B + floatRootDiscrim);
  t0 = q / A;
  t1 = C / q;
  if (float(t0) > float(t1)) 
  {
    // std::swap(*t0, *t1);
    float temp = t0;
    t0 = t1;
    t1 = temp;
  }
  return true;
}

vec3 MapSampleToCosineDistribution(float r1, float r2, vec3 direction, vec3 hit_norm, float power) {
  if(power >= 1e6f)
    return direction;

  const float sin_phi = sin(M_TWOPI * r1);
  const float cos_phi = cos(M_TWOPI * r1);

  //sincos(2.0f*r1*3.141592654f, &sin_phi, &cos_phi);

  const float cos_theta = pow(1.0f - r2, 1.0f / (power + 1.0f));
  const float sin_theta = sqrt(1.0f - cos_theta*cos_theta);

  vec3 deviation;
  deviation.x = sin_theta*cos_phi;
  deviation.y = sin_theta*sin_phi;
  deviation.z = cos_theta;

  vec3 ny = direction,  nx,  nz;
  CoordinateSystemV2(ny, nx, nz);

  {
    vec3 temp = ny;
    ny = nz;
    nz = temp;
  }

  vec3 res = nx*deviation.x + ny*deviation.y + nz*deviation.z;

  float invSign = dot(direction, hit_norm) > 0.0f ? 1.0f : -1.0f;

  if (invSign*dot(res, hit_norm) < 0.0f) // reflected ray is below surface #CHECK_THIS
  {
    res = (-1.0f)*nx*deviation.x + ny*deviation.y - nz*deviation.z;
    //belowSurface = true;
  }

  return res;
}

vec2 sincos_phi(in vec3 v) {
  float sin_theta2 = sin_theta_2(v);
  float inv_sin_theta = 1.f / safe_sqrt(sin_theta_2(v));

  vec2 result = vec2(v.x * inv_sin_theta,v.y * inv_sin_theta);

  result = abs(sin_theta2) <= 4.f * EPSILON_32 ? vec2(1.f,0.f) : clamp(result, -1.f, 1.f);

  return vec2(result.y,result.x);
}

vec2 sample_visible_11(float cos_theta_i, vec2 samp) {
  vec2 p = square_to_uniform_disk_concentric(samp);

  float s = 0.5f * (1.f + cos_theta_i);
  p.y = mix(safe_sqrt(1.f - p.x * p.x), p.y, s);

  // Project onto chosen side of the hemisphere
  float x = p.x, y = p.y,
        z = safe_sqrt(1.f - dot(p, p));

  // Convert to slope
  float sin_theta_i = safe_sqrt(1.f - cos_theta_i * cos_theta_i);
  float norm = 1.f / (sin_theta_i * y + cos_theta_i * z);
  return vec2(cos_theta_i * y - sin_theta_i * z,x) * norm;
}

int vknots_offset(NURBSHeader h) {
  return uknots_offset(h)+h.uknots_cnt;
}

float tricubic_spline(float p0, float p1, float p2, float p3, float x) {
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + x * (3.0 * (p1 - p2) + p3 - p0)));
}

int pts_offset(NURBSHeader h, int uspan, int vspan) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * ((h.vknots_cnt-1)*uspan + vspan); 
}

float fresnelSlick(const float VdotH) {
  const float tmp = 1.0f - abs(VdotH);
  return (tmp*tmp)*(tmp*tmp)*tmp;
}

float trG(vec3 wo, vec3 wi, vec2 alpha) { 
  return 1.0f / (1.0f + trLambda(wo, alpha) + trLambda(wi, alpha)); 
}

vec3 reflect2(const vec3 dir, const vec3 n) {  
  return normalize(dir - 2.0f * dot(dir, n) * n);  // dir - vector from light
}

vec3 SphericalDirectionPBRT(const float sintheta, const float costheta, const float phi) { 
  return vec3(sintheta * cos(phi),sintheta * sin(phi),costheta); 
}

float FrComplexConductor(float cosThetaI, complex eta) {
  float sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinThetaT = real_div_complex(sinThetaI,(complex_mul(eta,eta)));
  complex cosThetaT = complex_sqrt(real_sub_complex(1.0f,sinThetaT));

  complex r_parl = complex_div((complex_sub(complex_mul(eta,to_complex(cosThetaI)),cosThetaT)),(complex_add(complex_mul(eta,to_complex(cosThetaI)),cosThetaT)));
  complex r_perp = complex_div((real_sub_complex(cosThetaI,complex_mul(eta,cosThetaT))),(real_add_complex(cosThetaI,complex_mul(eta,cosThetaT))));
  return (complex_norm(r_parl) + complex_norm(r_perp)) / 2.0f;
}

float cosPhiPBRT(const vec3 w, const float sintheta) {
  if (sintheta == 0.0f)
    return 1.0f;
  else
    return clamp(w.x / sintheta, -1.0f, 1.0f);
}

float rndFloat1_Pseudo(inout RandomGen gen) {
  const uint x = NextState(gen);
  const uint tmp = (x * (x * x * 15731 + 74323) + 871483);
  const float scale      = (1.0f / 4294967296.0f);
  return (float((tmp)))*scale;
}

float smith_g1(in vec3 v, in vec3 m, vec2 alpha) {
  float xy_alpha_2 = alpha.x * v.x * alpha.x * v.x + alpha.y * v.y * alpha.y * v.y,
        tan_theta_alpha_2 = xy_alpha_2 / (v.z * v.z),
        result;


  result = 2.f / (1.f + safe_sqrt(1.f + tan_theta_alpha_2));

  // Perpendicular incidence -- no shadowing/masking
  if(xy_alpha_2 == 0.f)
    result = 1.f;

  /* Ensure consistent orientation (can't see the back
      of the microfacet from the front and vice versa) */

  if(v.z * dot(v, m) <= 0.f)
    result = 0.f;

  return result;
}

float GGX_GeomShadMask(const float cosThetaN, const float alpha) {
  // Height - Correlated G.
  //const float tanNV      = sqrt(1.0f - cosThetaN * cosThetaN) / cosThetaN;
  //const float a          = 1.0f / (alpha * tanNV);
  //const float lambda     = (-1.0f + sqrt(1.0f + 1.0f / (a*a))) / 2.0f;
  //const float G          = 1.0f / (1.0f + lambda);

  // Optimized and equal to the commented-out formulas on top.
  const float cosTheta_sqr = clamp(cosThetaN * cosThetaN, 0.0f, 1.0f);
  const float tan2         = (1.0f - cosTheta_sqr) / max(cosTheta_sqr, 1e-6f);
  const float GP           = 2.0f / (1.0f + safe_sqrt(1.0f + alpha * alpha * tan2));
  return GP;
}

complex FrComplexRefr(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT, uint polarization) {
  if (complex_norm(cosThetaI) < 1e-6f) //approximated
  {
    if (complex_norm(complex_sub(iorI,iorT)) < 1e-6f)
    {
      return make_complex(1,0);
    }
    return make_complex(0,0);
  }
  if (polarization == PolarizationS)
  {
    return complex_div((complex_mul(real_mul_complex(2,iorI),cosThetaI)),(complex_add(complex_mul(iorI,cosThetaI),complex_mul(iorT,cosThetaT))));
  }
  else if (polarization == PolarizationP)
  {
    return complex_div((complex_mul(real_mul_complex(2,iorI),cosThetaI)),(complex_add(complex_mul(iorT,cosThetaI),complex_mul(iorI,cosThetaT))));
  }
  return to_complex(0.f);
}

vec2 SampleUniformDiskPolar(vec2 u) {
  float r = safe_sqrt(u[0]);
  float theta = M_TWOPI * u[1];
  return vec2(r * cos(theta),r * sin(theta));
}

float GGX_Distribution(const float cosThetaNH, const float alpha) {
  const float alpha2 = alpha * alpha;
  const float NH_sqr = clamp(cosThetaNH * cosThetaNH, 0.0f, 1.0f);
  const float den    = NH_sqr * alpha2 + (1.0f - NH_sqr);
  return alpha2 / max(float((M_PI)) * den * den, 1e-6f);
}

float eval_microfacet(in vec3 m, vec2 alpha, int type) {
  float alpha_uv = alpha.x * alpha.y;
  float cos_theta = m.z;
  float cos_theta_2 = cos_theta * cos_theta;

  float result = 0.0f;
  if (type == 0) // Beckmann distribution function for Gaussian random surfaces 
  {
      result = exp(-(sqr(m.x / alpha.x) + sqr(m.y / alpha.y)) / cos_theta_2) / (M_PI * alpha_uv * sqr(cos_theta_2));
  } 
  else // GGX / Trowbridge-Reitz distribution function 
  {
      result = 1.f / (M_PI * alpha_uv * sqr(sqr(m.x / alpha.x) + sqr(m.y / alpha.y) + sqr(m.z)));
  }

  return result * cos_theta > 1e-20f ? result : 0.f;
}

float sinPhiPBRT(const vec3 w, const float sintheta) {
  if (sintheta == 0.0f)
    return 0.0f;
  else
    return clamp(w.y / sintheta, -1.0f, 1.0f);
}

float trD(vec3 w, vec3 wm, vec2 alpha) {
  return trG1(w, alpha) / AbsCosTheta(w) * trD(wm, alpha) * abs(dot(w, wm));
}

float orennayarFunc(const vec3 a_l, const vec3 a_v, const vec3 a_n, const float a_roughness) {
  const float cosTheta_wi = dot(a_l, a_n);
  const float cosTheta_wo = dot(a_v, a_n);

  const float sinTheta_wi = safe_sqrt(1.0f - cosTheta_wi * cosTheta_wi);
  const float sinTheta_wo = safe_sqrt(1.0f - cosTheta_wo * cosTheta_wo);

  const float sigma  = a_roughness * M_PI * 0.5f; //Radians(sig)
  const float sigma2 = sigma * sigma;
  const float A      = 1.0f - (sigma2 / (2.0f * (sigma2 + 0.33f)));
  const float B      = 0.45f * sigma2 / (sigma2 + 0.09f);

  ///////////////////////////////////////////////////////////////////////////// to PBRT coordinate system
  // wo = a_v = -ray_dir
  // wi = a_l = newDir
  //
  vec3 nx,  ny, nz = a_n;
  CoordinateSystemV2(nz, nx, ny);

  ///////////////////////////////////////////////////////////////////////////// to PBRT coordinate system

  // Compute cosine term of Oren-Nayar model
  float maxcos = 0.0f;

  if (sinTheta_wi > 1e-4f && sinTheta_wo > 1e-4f)
  {
    const vec3 wo = vec3(-dot(a_v, nx),-dot(a_v, ny),-dot(a_v, nz));
    const vec3 wi = vec3(-dot(a_l, nx),-dot(a_l, ny),-dot(a_l, nz));
    const float sinphii = sinPhiPBRT(wi, sinTheta_wi);
    const float cosphii = cosPhiPBRT(wi, sinTheta_wi);
    const float sinphio = sinPhiPBRT(wo, sinTheta_wo);
    const float cosphio = cosPhiPBRT(wo, sinTheta_wo);
    const float dcos    = cosphii * cosphio + sinphii * sinphio;
    maxcos              = max(0.0f, dcos);
  }

  // Compute sine and tangent terms of Oren-Nayar model
  float sinalpha = 0.0f, tanbeta = 0.0f;

  if (abs(cosTheta_wi) > abs(cosTheta_wo))
  {
    sinalpha = sinTheta_wo;
    tanbeta  = sinTheta_wi / max(abs(cosTheta_wi), DEPSILON);
  }
  else
  {
    sinalpha = sinTheta_wi;
    tanbeta  = sinTheta_wo / max(abs(cosTheta_wo), DEPSILON);
  }

  return (A + B * maxcos * sinalpha * tanbeta);
}

vec3 NormalMapTransform(const uint materialFlags, vec3 normalFromTex) {
  vec3 normalTS = vec3(2.0f * normalFromTex.x - 1.0f, 2.0f * normalFromTex.y - 1.0f, normalFromTex.z);

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

vec4 FrDielectricDetailedV2(float cos_theta_i, float eta) {
  cos_theta_i = clamp(cos_theta_i, -1.0f, 1.0f);
  
  float eta_it = eta;
  float eta_ti = 1.f / eta;
  if (cos_theta_i < 0.0f) 
  {
      eta_it = eta_ti;
      eta_ti = eta;
  }

  float cos_theta_t_sqr = -1.f * (-1.f * cos_theta_i * cos_theta_i + 1.f) * eta_ti * eta_ti + 1.f;
  float cos_theta_i_abs = abs(cos_theta_i);
  float cos_theta_t_abs = safe_sqrt(cos_theta_t_sqr);


  float r = 0.0f;
  if((eta == 1.f) || (cos_theta_i_abs == 0.f))
  {
    r = (eta == 1.f) ? 0.f : 1.f;
  }
  else
  {
    float a_s = (-1.f * eta_it * cos_theta_t_abs + cos_theta_i_abs) /
                (eta_it * cos_theta_t_abs + cos_theta_i_abs);

    float a_p = (-1.f * eta_it * cos_theta_i_abs + cos_theta_t_abs) /
                (eta_it * cos_theta_i_abs + cos_theta_t_abs);

    r = 0.5f * (a_s * a_s + a_p * a_p);
  }
  
  float cos_theta_t = cos_theta_i >= 0 ? -cos_theta_t_abs : cos_theta_t_abs;

  return vec4(r,cos_theta_t,eta_it,eta_ti);

}

vec3 lambertSample(const vec2 rands, const vec3 v, const vec3 n) {
  return MapSampleToCosineDistribution(rands.x, rands.y, n, n, 1.0f);
}

float lambertEvalBSDF(vec3 l, vec3 v, vec3 n) {
  return INV_PI;
}

float microfacet_G(in vec3 wi, in vec3 wo, in vec3 m, vec2 alpha) {
  return smith_g1(wi, m, alpha) * smith_g1(wo, m, alpha);
}

float lambertEvalPDF(vec3 l, vec3 v, vec3 n) { 
  return abs(dot(l, n)) * INV_PI;
}

float FrDielectric(float cosTheta_i, float eta) {
  cosTheta_i = clamp(cosTheta_i, -1.0f, 1.0f);
  
  if (cosTheta_i < 0.0f) 
  {
      eta = 1.0f / eta;
      cosTheta_i = -cosTheta_i;
  }

  float sin2Theta_i = 1.0f - cosTheta_i * cosTheta_i;
  float sin2Theta_t = sin2Theta_i / (eta * eta);
  if (sin2Theta_t >= 1.0f)
      return 1.f;
  float cosTheta_t = safe_sqrt(1.0f - sin2Theta_t);

  float r_parl = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
  float r_perp = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);
  return (r_parl * r_parl + r_perp * r_perp) / 2.0f;
}

vec3 square_to_cosine_hemisphere(in vec2 s) {
    // Low-distortion warping technique based on concentric disk mapping
    vec2 p = square_to_uniform_disk_concentric(s);

    // Guard against numerical imprecisions
    float z =  safe_sqrt(1.f - (p.x * p.x + p.y * p.y));
   

    return vec3(p.x,p.y,z);
}

vec4 sample_visible_normal(vec3 wi, vec2 rands, vec2 alpha) {
  // Step 1: stretch wi
  vec3 wi_p = normalize(vec3(alpha.x * wi.x,alpha.y * wi.y,wi.z));

  const vec2 sincos = sincos_phi(wi_p);
  const float sin_phi = sincos.x;
  const float cos_phi = sincos.y;

  const float cos_theta = wi_p.z;

  // Step 2: simulate P22_{wi}(slope.x, slope.y, 1, 1)
  vec2 slope = sample_visible_11(cos_theta, rands);

  // Step 3: rotate & unstretch
  slope = vec2((cos_phi * slope.x - sin_phi * slope.y) * alpha.x,(sin_phi * slope.x + cos_phi * slope.y) * alpha.y);

  // Step 4: compute normal
  vec3 m = normalize(vec3(-slope.x,-slope.y,1));

  float pdf = eval_microfacet(m, alpha, 1) * smith_g1(wi, m, alpha) * abs(dot(wi, m)) / wi.z;

  return vec4(m.x,m.y,m.z,pdf);
}

vec2 project2planes(in vec4 P1, in vec4 P2, in vec4 point) {
  return vec2(dot(P1, point),dot(P2, point));
}

float ggxEvalBSDF(const vec3 l, const vec3 v, const vec3 n, const float roughness) {
  if(abs(dot(l, n)) < 1e-5f)
    return 0.0f; 
 
  const float dotNV = dot(n, v);  
  const float dotNL = dot(n, l);
  if (dotNV < 1e-6f || dotNL < 1e-6f)
    return 0.0f; 

  const float  roughSqr = roughness * roughness;
  const vec3 h = normalize(v + l); // half vector.
  const float dotNH = dot(n, h);
  const float D     = GGX_Distribution(dotNH, roughSqr);
  const float G     = GGX_GeomShadMask(dotNV, roughSqr)*GGX_GeomShadMask(dotNL, roughSqr);      

  return (D * G / max(4.0f * dotNV * dotNL, 1e-6f));  // Pass single-scattering
}

vec3 refract(in vec3 wi, float cos_theta_t, float eta_ti) {
    return vec3(-eta_ti * wi.x,-eta_ti * wi.y,cos_theta_t);
}

vec3 eval_dist_trilinear_diff(const float values[8], vec3 dp) {
    float ddist_dx = -(1-dp.y)*(1-dp.z)*values[0] + 
                     -(1-dp.y)*(  dp.z)*values[1] + 
                     -(  dp.y)*(1-dp.z)*values[2] + 
                     -(  dp.y)*(  dp.z)*values[3] + 
                      (1-dp.y)*(1-dp.z)*values[4] + 
                      (1-dp.y)*(  dp.z)*values[5] + 
                      (  dp.y)*(1-dp.z)*values[6] + 
                      (  dp.y)*(  dp.z)*values[7];
    
    float ddist_dy = -(1-dp.x)*(1-dp.z)*values[0] + 
                     -(1-dp.x)*(  dp.z)*values[1] + 
                      (1-dp.x)*(1-dp.z)*values[2] + 
                      (1-dp.x)*(  dp.z)*values[3] + 
                     -(  dp.x)*(1-dp.z)*values[4] + 
                     -(  dp.x)*(  dp.z)*values[5] + 
                      (  dp.x)*(1-dp.z)*values[6] + 
                      (  dp.x)*(  dp.z)*values[7];

    float ddist_dz = -(1-dp.x)*(1-dp.y)*values[0] + 
                      (1-dp.x)*(1-dp.y)*values[1] + 
                     -(1-dp.x)*(  dp.y)*values[2] + 
                      (1-dp.x)*(  dp.y)*values[3] + 
                     -(  dp.x)*(1-dp.y)*values[4] + 
                      (  dp.x)*(1-dp.y)*values[5] + 
                     -(  dp.x)*(  dp.y)*values[6] + 
                      (  dp.x)*(  dp.y)*values[7];
  
    return vec3(ddist_dx,ddist_dy,ddist_dz);
  }

float step(float edge0, float edge1, float x) {
  float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t;
}

float FrDielectricPBRT(float cosThetaI, float etaI, float etaT) {
  cosThetaI = clamp(cosThetaI, -1.0f, 1.0f);
  // Potentially swap indices of refraction
  bool entering = cosThetaI > 0.0f;
  if (!entering) 
  {
    const float tmp = etaI;
    etaI = etaT;
    etaT = tmp;
    cosThetaI = abs(cosThetaI);
  }

  // Compute _cosThetaT_ using Snell's law
  float sinThetaI = safe_sqrt(1.0f - cosThetaI * cosThetaI);
  float sinThetaT = etaI / etaT * sinThetaI;

  // Handle total internal reflection
  if (sinThetaT >= 1.0f) 
    return 1.0f;

  const float cosThetaT = safe_sqrt(1.0f - sinThetaT * sinThetaT);
  const float Rparl     = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
  const float Rperp     = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
  return 0.5f*(Rparl * Rparl + Rperp * Rperp);
}

float trPDF(vec3 w, vec3 wm, vec2 alpha) { 
  return trD(w, wm, alpha); 
}

bool first_hit_is_closest(uint tag) {
  return tag != 1; /*TAG_TRIANGLE*/
}

vec2 sphereMapToPhiTheta(vec3 ray_dir) {
  const float x = ray_dir.z;
  const float y = ray_dir.x;
  const float z = -ray_dir.y;
                                  // r == 1.0f
  float theta = acos(z);     // [0,pi] 
  float phi   = atan(y,x); // [-pi,pi]
  if (phi < 0.0f)
    phi += 2.0f*M_PI;             // [-pi,pi] --> [0, 2*pi];  see PBRT.

  return vec2(phi, theta);
}

float SpectrumAverage(vec4 spec) {
  float sum = spec[0];
  for (uint i = 1; i < SPECTRUM_SAMPLE_SZ; ++i)
    sum += spec[int(i)];
  return sum / float(SPECTRUM_SAMPLE_SZ);
}

vec2 RayBoxIntersection2(vec3 rayOrigin, vec3 rayDirInv, vec3 boxMin, vec3 boxMax) {
  const float lo  = rayDirInv.x * (boxMin.x - rayOrigin.x);
  const float hi  = rayDirInv.x * (boxMax.x - rayOrigin.x);
  const float lo1 = rayDirInv.y * (boxMin.y - rayOrigin.y);
  const float hi1 = rayDirInv.y * (boxMax.y - rayOrigin.y);
  const float lo2 = rayDirInv.z * (boxMin.z - rayOrigin.z);
  const float hi2 = rayDirInv.z * (boxMax.z - rayOrigin.z);

  const float tmin = max(min(lo, hi), min(lo1, hi1));
  const float tmax = min(max(lo, hi), max(lo1, hi1));

  return vec2(max(tmin, min(lo2, hi2)),min(tmax, max(lo2, hi2)));
}

uint EXTRACT_START(uint a_leftOffset) { return  a_leftOffset & START_MASK; }

float FrFilmRefl(float cosThetaI, complex etaI, complex etaF, complex etaT, float thickness, float lambda) {
  complex sinThetaI = to_complex(1.0f - cosThetaI * cosThetaI);
  complex sinThetaF = complex_div(complex_mul(sinThetaI,to_complex((etaI.re * etaI.re))),(complex_mul(etaF,etaF)));
  complex cosThetaF = complex_sqrt(real_sub_complex(1.0f,sinThetaF));
  complex sinThetaT = complex_div(complex_mul(sinThetaI,to_complex((etaI.re * etaI.re))),(complex_mul(etaT,etaT)));
  complex cosThetaT = complex_sqrt(real_sub_complex(1.0f,sinThetaT));
  
  complex phaseDiff = filmPhaseDiff(cosThetaF, etaF, thickness, lambda);

  float result = 0;
  uint polarization[2] = {PolarizationS, PolarizationP};
  for (int p = 0; p <= 1; ++p)
  {
    complex FrReflI = FrComplexRefl(to_complex(cosThetaI), cosThetaF, etaI, etaF, polarization[p]);
    complex FrReflF = FrComplexRefl(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex FrRefl  = complex_mul(complex_mul(FrReflF,to_complex(exp(-phaseDiff.im))),make_complex(cos(phaseDiff.re),sin(phaseDiff.re)));
    FrRefl          = complex_div((complex_add(FrReflI,FrRefl)),(real_add_complex(1,complex_mul(FrReflI,FrRefl))));
    result += complex_norm(FrRefl);
  }

  return result / 2;
}

vec3 SafeInverse(vec3 d) {
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  vec3 res;
  res.x = 1.0f / (abs(d.x) > ooeps ? d.x : abs(ooeps)*sign(d.x));
  res.y = 1.0f / (abs(d.y) > ooeps ? d.y : abs(ooeps)*sign(d.y));
  res.z = 1.0f / (abs(d.z) > ooeps ? d.z : abs(ooeps)*sign(d.z));
  return res;
}

vec3 refract2(const vec3 dir, const vec3 n, const float relativeIor) {  
  const float cosi = dot(dir, n);        // dir - vector from light. The normal should always look at the light vector.
  const float eta  = 1.0f / relativeIor; // Since the incoming vector and the normal are directed in the same direction.
  const float k    = 1.0f - eta * eta * (1.0f - cosi * cosi);
  if (k < 0)       
    return reflect2(dir, n); // full internal reflection 
  else         
    return normalize(eta * dir - (eta * cosi + sqrt(k)) * n); // the refracted vector    
}

float sum(vec4 v) {return v.x + v.y + v.z + v.w;}

float fresnel2(vec3 v, vec3 n, float ior) {
  // Calculating the angle of incidence of light
  const float cosi = dot(v, n);
  // We calculate the angle of refraction of light according to the Snellius law
  const float sint = sqrt(1.0f - cosi * cosi) / ior;
  // Check if there is a complete internal reflection
  if (sint > 1.0f) 
  {
    // If yes, then we return the reflection coefficient equal to 1
    return 1.0f;
  }
  else 
  {
    // Otherwise we calculate the angle of refraction of light
    const float cost = sqrt(1.0f - sint * sint);
    // We calculate the reflection coefficients for parallel and perpendicular polarization using Fresnel formulas
    const float Rp   = (ior * cosi - cost) / (ior * cosi + cost);
    const float Rs   = (cosi - ior * cost) / (cosi + ior * cost);
    // We return the average value of these coefficients
    return (Rp * Rp + Rs * Rs) * 0.5f;
  }
}

float conductorRoughEvalInternal(vec3 wo, vec3 wi, vec3 wm, vec2 alpha, complex ior) {
  if(wo.z * wi.z < 0) // not in the same hemisphere
  {
    return 0.0f;
  }

  float cosTheta_o = AbsCosTheta(wo);
  float cosTheta_i = AbsCosTheta(wi);
  if (cosTheta_i == 0 || cosTheta_o == 0)
    return 0.0f;

  float F = FrComplexConductor(abs(dot(wo, wm)), ior);
  float val = trD(wm, alpha) * F * trG(wo, wi, alpha) / (4.0f * cosTheta_i * cosTheta_o);

  return val;
}

vec4 hydraFresnelCond(vec4 f0, float VdotH, float ior, float roughness) {  
  if(ior == 0.0f) // fresnel reflactance is disabled
    return f0;

  return f0 + (vec4(1.0f) - f0) * fresnelSlick(VdotH); // return bsdf * (f0 + (1 - f0) * (1 - abs(VdotH))^5)
}

bool Refract(const vec3 wi, const vec3 n, float eta, inout vec3 wt) {
  // Compute $\cos \theta_\roman{t}$ using Snell's law
  float cosThetaI  = dot(n, wi);
  float sin2ThetaI = max(float(0), float(1.0f - cosThetaI * cosThetaI));
  float sin2ThetaT = eta * eta * sin2ThetaI;
  // Handle total internal reflection for transmission
  if (sin2ThetaT >= 1) return false;
  float cosThetaT = sqrt(1 - sin2ThetaT);
  wt = eta * (-1.0f)*wi + (eta * cosThetaI - cosThetaT) * n;
  return true;
}

vec3 FaceForward(vec3 v, vec3 n2) {
    return (dot(v, n2) < 0.f) ? (-1.0f) * v : v;
}

FrReflRefr FrFilm(float cosThetaI, complex etaI, complex etaF, complex etaT, float thickness, float lambda) {
  complex sinThetaI = to_complex(1.0f - cosThetaI * cosThetaI);
  complex sinThetaF = complex_div(complex_mul(sinThetaI,to_complex((etaI.re * etaI.re))),(complex_mul(etaF,etaF)));
  complex cosThetaF = complex_sqrt(real_sub_complex(1.0f,sinThetaF));
  complex sinThetaT = complex_div(complex_mul(sinThetaI,to_complex((etaI.re * etaI.re))),(complex_mul(etaT,etaT)));
  complex cosThetaT = complex_sqrt(real_sub_complex(1.0f,sinThetaT));

  complex phaseDiff = filmPhaseDiff(cosThetaF, etaF, thickness, lambda);

  FrReflRefr result = {0, 0};
  uint polarization[2] = {PolarizationS, PolarizationP};
  for (int p = 0; p <= 1; ++p)
  {
    complex FrReflI = FrComplexRefl(to_complex(cosThetaI), cosThetaF, etaI, etaF, polarization[p]);
    complex FrReflF = FrComplexRefl(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex FrRefrI = FrComplexRefr(to_complex(cosThetaI), cosThetaF, etaI, etaF, polarization[p]);
    complex FrRefrF = FrComplexRefr(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex exp_1 = real_mul_complex(exp(-phaseDiff.im / 2),make_complex(cos(phaseDiff.re / 2),sin(phaseDiff.re / 2)));
    complex exp_2 = complex_mul(exp_1,exp_1);

    complex denom = real_add_complex(1,complex_mul(complex_mul(FrReflI,FrReflF),exp_2));
    if (complex_norm(denom) < 1e-6f)
    {
      result.refl += 0.5f;
    }
    else
    {
      result.refl += complex_norm(complex_div((complex_add(FrReflI,complex_mul(FrReflF,exp_2))),denom)) / 2;
      result.refr += complex_norm(complex_div(complex_mul(complex_mul(FrRefrI,FrRefrF),exp_1),denom)) / 2;
    }

  }
  result.refr *= getRefractionFactor(cosThetaI, cosThetaT, etaI, etaT);

  return result;
}

MatIdWeightPair make_weight_pair(MatIdWeight a, MatIdWeight b) {
  MatIdWeightPair res;
  res.first  = a;
  res.second = b;
  return res;
}

vec3 trSample(vec3 wo, vec2 rands, vec2 alpha) {
  // Transform _w_ to hemispherical configuration
  vec3 wh = normalize(vec3(alpha.x * wo.x,alpha.y * wo.y,wo.z));
  if (wh.z < 0)
  {
    wh = (-1.0f) * wh;
  }

  // Find orthonormal basis for visible normal sampling
  vec3 T1 = (wh.z < 0.99999f) ? normalize(cross(vec3(0,0,1), wh)) : vec3(1,0,0);
  vec3 T2 = cross(wh, T1);

  // Generate uniformly distributed points on the unit disk
  vec2 p = SampleUniformDiskPolar(rands);

  // Warp hemispherical projection for visible normal sampling
  float h = safe_sqrt(1 - p.x * p.x);
  p.y = mix(h, p.y, (1 + wh.z) / 2);

  // Reproject to hemisphere and transform normal to ellipsoid configuration
  float pz = safe_sqrt(1.0f - dot(p, p));
  vec3 nh = p.x * T1 + p.y * T2 + pz * wh;
  return normalize(vec3(alpha.x * nh.x,alpha.y * nh.y,max(1e-6f, nh.z)));
}

vec4 rndFloat4_Pseudo(inout RandomGen gen) {
  uint x = NextState(gen);

  const uint x1 = (x * (x * x * 15731 + 74323) + 871483);
  const uint y1 = (x * (x * x * 13734 + 37828) + 234234);
  const uint z1 = (x * (x * x * 11687 + 26461) + 137589);
  const uint w1 = (x * (x * x * 15707 + 789221) + 1376312589);

  const float scale = (1.0f / 4294967296.0f);

  return vec4(float((x1)), float((y1)), float((z1)), float((w1)))*scale;
}

float ggxEvalPDF(const vec3 l, const vec3 v, const vec3 n, const float roughness) { 
  const float dotNV = dot(n, v);
  const float dotNL = dot(n, l);
  if (dotNV < 1e-6f || dotNL < 1e-6f)
    return 1.0f;

  const float  roughSqr  = roughness * roughness;
    
  const vec3 h = normalize(v + l); // half vector.
  const float dotNH = dot(n, h);
  const float dotHV = dot(h, v);
  const float D     = GGX_Distribution(dotNH, roughSqr);
  return  D * dotNH / (4.0f * max(dotHV, 1e-6f));
}

vec3 ggxSample(const vec2 rands, const vec3 v, const vec3 n, const float roughness) {
  const float roughSqr = roughness * roughness;
    
  vec3 nx,  ny, nz = n;
  CoordinateSystemV2(nz, nx, ny);
    
  const vec3 wo = vec3(dot(v, nx),dot(v, ny),dot(v, nz));
  const float phi       = rands.x * M_TWOPI;
  const float cosTheta  = clamp(safe_sqrt((1.0f - rands.y) / (1.0f + roughSqr * roughSqr * rands.y - rands.y)), 0.0f, 1.0f);
  const float sinTheta  = safe_sqrt(1.0f - cosTheta * cosTheta);
  const vec3 wh = SphericalDirectionPBRT(sinTheta, cosTheta, phi);
    
  const vec3 wi = 2.0f * dot(wo, wh) * wh - wo;      // Compute incident direction by reflecting about wm  
  return normalize(wi.x * nx + wi.y * ny + wi.z * nz); // back to normal coordinate system
}

float PdfAtoW(const float aPdfA, const float aDist, const float aCosThere) {
  return (aPdfA*aDist*aDist) / max(aCosThere, 1e-30f);
}

float mylocalsmoothstep(float edge0, float edge1, float x) {
  float  tVal = (x - edge0) / (edge1 - edge0);
  float  t    = min(max(tVal, 0.0f), 1.0f); 
  return t * t * (3.0f - 2.0f * t);
}

vec2 MapSamplesToDisc(vec2 xy) {
  float x = xy.x;
  float y = xy.y;

  float r = 0;
  float phi = 0;

  vec2 res = xy;

  if (x>y && x>-y)
  {
    r = x;
    phi = 0.25f*3.141592654f*(y / x);
  }

  if (x < y && x > -y)
  {
    r = y;
    phi = 0.25f*3.141592654f*(2.0f - x / y);
  }

  if (x < y && x < -y)
  {
    r = -x;
    phi = 0.25f*3.141592654f*(4.0f + y / x);
  }

  if (x >y && x<-y)
  {
    r = -y;
    phi = 0.25f*3.141592654f*(6 - x / y);
  }

  //float sin_phi, cos_phi;
  //sincosf(phi, &sin_phi, &cos_phi);
  float sin_phi = sin(phi);
  float cos_phi = cos(phi);

  res.x = r*sin_phi;
  res.y = r*cos_phi;

  return res;
}

vec3 texCoord2DToSphereMap(vec2 a_texCoord, inout float pSinTheta) {
  const float phi   = a_texCoord.x * 2.f * M_PI; // see PBRT coords:  Float phi = uv[0] * 2.f * Pi;
  const float theta = a_texCoord.y * M_PI;       // see PBRT coords:  Float theta = uv[1] * Pi

  const float sinTheta = sin(theta);

  const float x = sinTheta*cos(phi);           // see PBRT coords: (Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta)
  const float y = sinTheta*sin(phi);
  const float z = cos(theta);

  (pSinTheta)  = sinTheta;
  return vec3(y, -z, x);
}

int new_node(float txm, int x, float tym, int y, float tzm, int z) {
  return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
}

int first_node(vec3 t0, vec3 tm) {
uint answer = 0;   // initialize to 00000000
// select the entry plane and set bits
if(t0.x > t0.y){
    if(t0.x > t0.z){ // PLANE YZ
        if(t0.x > tm.y) answer|=2;    // set bit at position 1
        if(t0.x > tm.z) answer|=1;    // set bit at position 0
        return int(answer);
    }
}
else {
    if(t0.y > t0.z){ // PLANE XZ
        if(tm.x < t0.y) answer|=4;    // set bit at position 2
        if(t0.y > tm.z) answer|=1;    // set bit at position 0
        return int(answer);
    }
}
// PLANE XY
if(tm.x < t0.z) answer|=4;    // set bit at position 2
if(tm.y < t0.z) answer|=2;    // set bit at position 1
return int(answer);
}

bool trEffectivelySmooth(vec2 alpha) { 
  return max(alpha.x, alpha.y) < 1e-3f; 
}

void dielectricSmoothEval(inout BsdfEval pRes) {
  pRes.val = vec4(0.0f);
  pRes.pdf = 0.0f;
}

MatIdWeight make_id_weight(uint a, float b) {
  MatIdWeight res;
  res.id  = a;
  res.weight = b;
  return res;
}

vec2 sphereMapTo2DTexCoord(vec3 ray_dir, inout float pSinTheta) {
  const vec2 angles = sphereMapToPhiTheta(ray_dir);

  const float texX = clamp(angles.x*0.5f*INV_PI, 0.0f, 1.0f);
  const float texY = clamp(angles.y*INV_PI,      0.0f, 1.0f);

  (pSinTheta) = sqrt(1.0f - ray_dir.y*ray_dir.y); // sin(angles.y);
  return vec2(texX, texY);
}

float misHeuristicPower1(float p) { return isfinite(p) ? abs(p) : 0.0f; }

vec2 mulRows2x4(const vec4 row0, const vec4 row1, vec2 v) {
  vec2 res;
  res.x = row0.x*v.x + row0.y*v.y + row0.w;
  res.y = row1.x*v.x + row1.y*v.y + row1.w;
  return res;
}

vec3 matmul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

float epsilonOfPos(vec3 hitPos) { return max(max(abs(hitPos.x), max(abs(hitPos.y), abs(hitPos.z))), 2.0f*GEPSILON)*GEPSILON; }

vec3 XYZToRGB(vec3 xyz) {
  vec3 rgb;
  rgb[0] = +3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
  rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
  rgb[2] = +0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];

  return rgb;
}

vec3 matmul3x3(mat4 m, vec3 v) { 
  return (m*vec4(v, 0.0f)).xyz;
}

bool isLeafAndIntersect(uint flags) { return (flags == (LEAF_BIT | 0x1 )); }

bool notLeafAndIntersect(uint flags) { return (flags != (LEAF_BIT | 0x1)); }

bool isLeafOrNotIntersect(uint flags) { return (flags & LEAF_BIT) !=0 || (flags & 0x1) == 0; }

vec3 OffsRayPos(const vec3 a_hitPos, const vec3 a_surfaceNorm, const vec3 a_sampleDir) {
  const float signOfNormal2 = dot(a_sampleDir, a_surfaceNorm) < 0.0f ? -1.0f : 1.0f;
  const float offsetEps     = epsilonOfPos(a_hitPos);
  return a_hitPos + signOfNormal2*offsetEps*a_surfaceNorm;
}

bool isSpecular(in MisData data) { return (data.matSamplePdf < 0.0f); }

float misWeightHeuristic(float a, float b) {
  const float w = misHeuristicPower1(a) / max(misHeuristicPower1(a) + misHeuristicPower1(b), 1e-30f);
  return isfinite(w) ? w : 0.0f;
}

MisData makeInitialMisData() {
  MisData data;
  data.matSamplePdf = 1.0f;
  data.ior          = 1.0f; // start from air
  return data;
}

vec3 EyeRayDirNormalized(float x, float y, mat4 a_mViewProjInv) {
  vec4 pos = vec4(2.0f*x - 1.0f,2.0f*y - 1.0f,0.0f,1.0f);
  pos = a_mViewProjInv * pos;
  pos /= pos.w;
  return normalize(pos.xyz);
}

void transform_ray3f(mat4 a_mWorldViewInv, inout vec3 ray_pos, inout vec3 ray_dir) {
  vec3 pos = mul4x3(a_mWorldViewInv, (ray_pos));
  vec3 pos2 = mul4x3(a_mWorldViewInv, ((ray_pos) + 100.0f*(ray_dir)));

  vec3 diff = pos2 - pos;

  (ray_pos)  = pos;
  (ray_dir)  = normalize(diff);
}

vec4 SampleWavelengths(float u, float a, float b) {
  // pdf is 1.0f / (b - a)
  vec4 res;

  res[0] = mix(a, b, u);

  float delta = (b - a) / float(SPECTRUM_SAMPLE_SZ);
  for (uint i = 1; i < SPECTRUM_SAMPLE_SZ; ++i) 
  {
      res[int(i)] = res[i - 1] + delta;
      if (res[int(i)] > b)
        res[int(i)] = a + (res[int(i)] - b);
  }

  return res;
}

float maxcomp(vec3 v) { return max(v.x, max(v.y, v.z)); }

vec3 decode_normal(vec2 e) {
  vec3 v = vec3(e.x,e.y,1.0f - abs(e.x) - abs(e.y));
  if (v.z < 0) 
  {
    float vx = v.x;
    v.x = (1.0f - abs(v.y)) * ((v.x >= 0.0) ? +1.0f : -1.0f);
    v.y = (1.0f - abs(vx)) * ((v.y >= 0.0) ? +1.0f : -1.0f);
  }
  return normalize(v);
}

uint fakeOffset(uint x, uint y, uint pitch) { return y*pitch + x; }  // RTV pattern, for 2D threading


