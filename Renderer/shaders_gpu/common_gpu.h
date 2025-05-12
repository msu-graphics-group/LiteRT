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
const uint MULTI_RENDER_MATERIAL_TYPE_COLORED = 0;
const uint MULTI_RENDER_MATERIAL_TYPE_TEXTURED = 1;
const uint DEFAULT_MATERIAL = 0u;
const uint DEFAULT_TEXTURE = 0u;
const uint MULTI_RENDER_MAX_TEXTURES = 16;
const uint PACK_XY_BLOCK_SIZE = 8;
struct MultiRendererMaterial
{
  uint type;
  uint texId; // valid if type == MULTI_RENDER_MATERIAL_TYPE_TEXTURED
  uint _pad[2];
  vec4 base_color; // valid if type == MULTI_RENDER_MATERIAL_TYPE_COLORED
};
const uint LIGHT_TYPE_DIRECT = 0;
const uint LIGHT_TYPE_POINT = 1;
const uint LIGHT_TYPE_AMBIENT = 2;
struct Light
{
  vec3 space; //position or direction
  uint type;
  vec3 color; //intensity included
  uint _pad;
};
struct HydraSceneProperties
{
  uint num_primitives;
};
const uint palette_size = 20;
const uint m_palette[20] = {
    0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
    0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
    0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
    0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
    0xff808000, 0xffffd8b1, 0xff000080, 0xff808080
  };
const uint m_lod_palette[20] = {
    0xff000000, 0xff000040, 0xff000080, 0xff0000c0,
    0xff0040ff, 0xff0080ff, 0xff00c0ff, 0xff00ffff,
    0xff00ffc0, 0xff00ff80, 0xff00ff40, 0xff00ff00,
    0xff00c000, 0xff008000, 0xff004000, 0xff000000,
    0xffff0000, 0xffff0000, 0xffff0000, 0xffff0000,
  };
const float GEPSILON = 2e-5f;
const float DEPSILON = 1e-20f;

#ifndef SKIP_UBO_INCLUDE
#include "include/MultiRenderer_gpu_ubo.h"
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
#define CFLOAT_GUARDIAN 
#define DISABLE_RF_GRID 1
#define DISABLE_GS_PRIMITIVE 1
#define DISABLE_SDF_HP 1
#define CMESH4_GEOM_H 
#define DISABLE_OPENVDB 1
#define DISABLE_CATMUL_CLARK 1
#define DISABLE_RIBBON 1
#define IMAGE2D_H 
#define MAXFLOAT FLT_MAX

float tricubic_spline(float p0, float p1, float p2, float p3, float x) {
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + x * (3.0 * (p1 - p2) + p3 - p0)));
}

uint SBS_v_to_i(float i, float j, float k, uint v_size, uint pad) {
  return (uint(i)+pad)*v_size*v_size + (uint(j)+pad)*v_size + (uint(k)+pad);
}

uint EXTRACT_COUNT(uint a_leftOffset) { return (a_leftOffset & SIZE_MASK) >> 24; }

int uknots_offset(NURBSHeader h) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * (h.uknots_cnt-1) * (h.vknots_cnt-1);
}

int pts_offset(NURBSHeader h, int uspan, int vspan) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * ((h.vknots_cnt-1)*uspan + vspan); 
}

int vknots_offset(NURBSHeader h) {
  return uknots_offset(h)+h.uknots_cnt;
}

vec3 SafeInverse(vec3 d) {
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  vec3 res;
  res.x = 1.0f / (abs(d.x) > ooeps ? d.x : abs(ooeps)*sign(d.x));
  res.y = 1.0f / (abs(d.y) > ooeps ? d.y : abs(ooeps)*sign(d.y));
  res.z = 1.0f / (abs(d.z) > ooeps ? d.z : abs(ooeps)*sign(d.z));
  return res;
}

uint EXTRACT_START(uint a_leftOffset) { return  a_leftOffset & START_MASK; }

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

bool first_hit_is_closest(uint tag) {
  return tag != 1; /*TAG_TRIANGLE*/
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

vec2 project2planes(in vec4 P1, in vec4 P2, in vec4 point) {
  return vec2(dot(P1, point),dot(P2, point));
}

float step(float edge0, float edge1, float x) {
  float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t;
}

int new_node(float txm, int x, float tym, int y, float tzm, int z) {
  return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
}

vec4 rayCapsuleIntersect(in vec3 ray_pos, in vec3 ray_dir, in vec3 pa, in vec3 pb, float ra) {
  vec3 ba = pb - pa;
  vec3 oa = ray_pos - pa;
  vec3 ob = ray_pos - pb;
  float baba = dot(ba,ba);
  float bard = dot(ba,ray_dir);
  float baoa = dot(ba,oa);
  float rdoa = dot(ray_dir,oa);
  float oaoa = dot(oa,oa);
  float a = baba      - bard*bard;
  float b = baba*rdoa - baoa*bard;
  float c = baba*oaoa - baoa*baoa - ra*ra*baba;
  float h = b*b - a*c;

  if(h >= 0.f)
  {
    float dist = (-b-sqrt(h))/a;
    float y = baoa + dist*bard;

    // body
    if(y > 0.f && y < baba)
      return vec4(normalize(oa+dist*ray_dir - ba*y/baba), dist);

    // cap
    vec3 oc = (y <= 0.0) ? oa : ob;
    b = dot(ray_dir,oc);
    c = dot(oc,oc) - ra*ra;
    h = b*b - c;
    if(h > 0.f)
    {
      dist = -b - sqrt(h);
      vec3 norm = normalize(ray_pos+dist*ray_dir + (oc - ray_pos));
      return vec4(norm, dist);
    }
  }
  return vec4(2e15f);
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

vec3 mymul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

vec3 matmul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

vec3 matmul3x3(mat4 m, vec3 v) { 
  return (m*vec4(v, 0.0f)).xyz;
}

bool isLeafAndIntersect(uint flags) { return (flags == (LEAF_BIT | 0x1 )); }

bool notLeafAndIntersect(uint flags) { return (flags != (LEAF_BIT | 0x1)); }

bool isLeafOrNotIntersect(uint flags) { return (flags & LEAF_BIT) !=0 || (flags & 0x1) == 0; }

vec3 EyeRayDirNormalized(float x, float y, mat4 a_mViewProjInv) {
  vec4 pos = vec4(2.0f*x - 1.0f,-2.0f*y + 1.0f,1.0f,1.0f);
  pos = a_mViewProjInv * pos;
  pos /= pos.w;
  return normalize(pos.xyz);
}

void transform_ray3f(mat4 a_mWorldViewInv, inout vec3 ray_pos, inout vec3 ray_dir) {
  vec3 pos = mymul4x3(a_mWorldViewInv, (ray_pos));
  vec3 pos2 = mymul4x3(a_mWorldViewInv, ((ray_pos) + 100.0f*(ray_dir)));

  vec3 diff = pos2 - pos;

  (ray_pos)  = pos;
  (ray_dir)  = normalize(diff);
}

uint SuperBlockIndex2DOpt(uint tidX, uint tidY, uint a_width) {
  const uint inBlockIdX = tidX & 0x00000003; // 4x4 blocks
  const uint inBlockIdY = tidY & 0x00000003; // 4x4 blocks
  const uint localIndex = inBlockIdY*4 + inBlockIdX;
  const uint wBlocks    = a_width >> 2;
  const uint blockX     = tidX    >> 2;
  const uint blockY     = tidY    >> 2;
  
  const uint inHBlockIdX = blockX & 0x00000001; // 2x2 SuperBlocks
  const uint inHBlockIdY = blockY & 0x00000001; // 2x2 SuperBlocks
  const uint localIndexH = inHBlockIdY*2 + inHBlockIdX;
  const uint wBlocksH    = wBlocks >> 1;
  const uint blockHX     = blockX  >> 1;
  const uint blockHY     = blockY  >> 1;

  return (blockHX + blockHY*wBlocksH)*64 + localIndexH*16 + localIndex;
}

uint fakeOffset(uint x, uint y, uint pitch) { return y*pitch + x; }  // RTV pattern, for 2D threading


