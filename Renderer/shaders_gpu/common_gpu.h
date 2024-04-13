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

const uint SDF_PRIM_SPHERE = 0;
const uint SDF_PRIM_BOX = 1;
const uint SDF_PRIM_CYLINDER = 2;
const uint SDF_PRIM_SIREN = 3;
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
const int NEURAL_SDF_MAX_LAYERS = 8;
const int NEURAL_SDF_MAX_LAYER_SIZE = 1024;
const float SIREN_W0 = 30;
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
struct SdfOctreeNode
{
  float value;
  uint offset; // offset for children (they are stored together). 0 offset means it's a leaf
};
struct SdfFrameOctreeNode
{
  float values[8];
  uint offset; // offset for children (they are stored together). 0 offset means it's a leaf  
};
const uint BUILD_LOW = 0;
const uint BUILD_MEDIUM = 1;
const uint BUILD_HIGH = 2;
const uint BUILD_REFIT = 3;
struct CRT_Hit 
{
  float    t;         ///< intersection distance from ray origin to object
  uint primId; 
  uint instId;
  uint geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
  float    coords[4]; ///< custom intersection data; for triangles coords[0] and coords[1] stores baricentric coords (u,v)
                      // coords[2] and coords[3] stores normal.xy
  float    adds[4];
};
const uint SH_TYPE = 28;
const uint TYPE_MESH_TRIANGLE = 0;
const uint TYPE_SDF_PRIMITIVE = 1;
const uint TYPE_SDF_GRID = 2;
const uint TYPE_SDF_OCTREE = 3;
const uint TYPE_SDF_FRAME_OCTREE = 4;
const uint TYPE_RF_GRID = 5;
const uint SDF_OCTREE_SAMPLER_MIPSKIP_3X3 = 0;
const uint SDF_OCTREE_SAMPLER_MIPSKIP_CLOSEST = 1;
const uint SDF_OCTREE_SAMPLER_CLOSEST = 2;
const uint SDF_FRAME_OCTREE_BLAS_NO = 0;
const uint SDF_FRAME_OCTREE_BLAS_DEFAULT = 1;
const uint SDF_FRAME_OCTREE_INTERSECT_DEFAULT = 0;
const uint SDF_FRAME_OCTREE_INTERSECT_ST = 1;
const uint SDF_FRAME_OCTREE_INTERSECT_ANALYTIC = 2;
const uint SDF_FRAME_OCTREE_INTERSECT_NEWTON = 3;
const uint VISUALIZE_STAT_NONE = 0;
const uint VISUALIZE_STAT_SPHERE_TRACE_ITERATIONS = 1;
struct TracerPreset
{
  uint need_normal;
  uint sdf_octree_sampler; //enum SdfOctreeSampler
  uint visualize_stat; //enum VisualizeStatType 
  uint sdf_frame_octree_blas; //enum SdfFrameOctreeBLAS
  uint sdf_frame_octree_intersect; //enum SdfFrameOctreeIntersect
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
const uint MULTI_RENDER_MODE_MASK = 0;
const uint MULTI_RENDER_MODE_LAMBERT = 1;
const uint MULTI_RENDER_MODE_DEPTH = 2;
const uint MULTI_RENDER_MODE_LINEAR_DEPTH = 3;
const uint MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH = 4;
const uint MULTI_RENDER_MODE_PRIMIVIVE = 5;
const uint MULTI_RENDER_MODE_TYPE = 6;
const uint MULTI_RENDER_MODE_GEOM = 7;
const uint MULTI_RENDER_MODE_NORMAL = 8;
const uint MULTI_RENDER_MODE_BARYCENTRIC = 9;
const uint MULTI_RENDER_MODE_SPHERE_TRACE_ITERATIONS = 10;
const uint MULTI_RENDER_MODE_RF = 11;
struct MultiRenderPreset
{
  uint mode; //enum MultiRenderMode
  uint sdf_octree_sampler; //enum SdfOctreeSampler
  uint spp; //samples per pixel, should be a square (1, 4, 9, 16 etc.)
  uint sdf_frame_octree_blas; //enum SdfFrameOctreeBLAS
  uint sdf_frame_octree_intersect; //enum SdfFrameOctreeIntersect
};
const uint palette_size = 20;
const uint m_palette[20] = {
    0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
    0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
    0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
    0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
    0xff808000, 0xffffd8b1, 0xff000080, 0xff808080
  };
const float GEPSILON = 2e-5f;
const float DEPSILON = 1e-20f;
const uint X_L = 1<<0;
const uint X_H = 1<<1;
const uint Y_L = 1<<2;
const uint Y_H = 1<<3;
const uint Z_L = 1<<4;
const uint Z_H = 1<<5;
const uint INVALID_IDX = 1u<<31u;
struct SDONeighbor
{
  SdfOctreeNode node;
  uint overshoot;
};

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

void sh_eval_2(in vec3 d, inout float fout[9]) {
  float x = d.x, y = d.y, z = d.z, z2 = z * z;
  float c0, c1, s0, s1, tmp_a, tmp_b, tmp_c;

  fout[0] = 0.28209479177387814;
  fout[2] = z * 0.488602511902919923;
  fout[6] = z2 * 0.94617469575756008 + -0.315391565252520045;
  c0 = x;
  s0 = y;

  tmp_a = -0.488602511902919978;
  fout[3] = tmp_a * c0;
  fout[1] = tmp_a * s0;
  tmp_b = z * -1.09254843059207896;
  fout[7] = tmp_b * c0;
  fout[5] = tmp_b * s0;
  c1 = x * c0 - y * s0;
  s1 = x * s0 + y * c0;

  tmp_c = 0.546274215296039478;
  fout[8] = tmp_c * c1;
  fout[4] = tmp_c * s1;
}

int indexGrid(int x, int y, int z, int gridSize) {
    return (x + y * gridSize + z * gridSize * gridSize) * 28;
}

void lerpCellf(const float v0[28], const float v1[28], const float t, inout float memory[28]) {
  for (int i = 0; i < 28; i++)
    memory[i] = mix(v0[i], v1[i], t);
}

float eval_sh(inout float sh[28], vec3 rayDir, const int offset) {
  float sh_coeffs[9];
  sh_eval_2(rayDir, sh_coeffs);

  float sum = 0.0f;
  for (int i = 0; i < 9; i++)
    sum += sh[offset + i] * sh_coeffs[i];

  return sum;
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

vec2 RayBoxIntersection(vec3 ray_pos, vec3 ray_dir, vec3 boxMin, vec3 boxMax) {
  ray_dir.x = 1.0f / ray_dir.x; // may precompute if intersect many boxes
  ray_dir.y = 1.0f / ray_dir.y; // may precompute if intersect many boxes
  ray_dir.z = 1.0f / ray_dir.z; // may precompute if intersect many boxes

  float lo = ray_dir.x * (boxMin.x - ray_pos.x);
  float hi = ray_dir.x * (boxMax.x - ray_pos.x);

  float tmin = min(lo, hi);
  float tmax = max(lo, hi);

  float lo1 = ray_dir.y * (boxMin.y - ray_pos.y);
  float hi1 = ray_dir.y * (boxMax.y - ray_pos.y);

  tmin = max(tmin, min(lo1, hi1));
  tmax = min(tmax, max(lo1, hi1));

  float lo2 = ray_dir.z * (boxMin.z - ray_pos.z);
  float hi2 = ray_dir.z * (boxMax.z - ray_pos.z);

  tmin = max(tmin, min(lo2, hi2));
  tmax = min(tmax, max(lo2, hi2));

  return vec2(tmin,tmax);
}

vec3 SafeInverse(vec3 d) {
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  vec3 res;
  res.x = 1.0f / (abs(d.x) > ooeps ? d.x : copysign(ooeps, d.x));
  res.y = 1.0f / (abs(d.y) > ooeps ? d.y : copysign(ooeps, d.y));
  res.z = 1.0f / (abs(d.z) > ooeps ? d.z : copysign(ooeps, d.z));
  return res;
}

uint EXTRACT_START(uint a_leftOffset) { return  a_leftOffset & START_MASK; }

uint EXTRACT_COUNT(uint a_leftOffset) { return (a_leftOffset & SIZE_MASK) >> 24; }

bool notLeafAndIntersect(uint flags) { return (flags != (LEAF_BIT | 0x1)); }

vec3 matmul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

bool isLeafAndIntersect(uint flags) { return (flags == (LEAF_BIT | 0x1 )); }

vec3 mymul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

vec3 matmul3x3(mat4 m, vec3 v) { 
  return (m*vec4(v, 0.0f)).xyz;
}

bool isLeafOrNotIntersect(uint flags) { return (flags & LEAF_BIT) !=0 || (flags & 0x1) == 0; }

void transform_ray3f(mat4 a_mWorldViewInv, inout vec3 ray_pos, inout vec3 ray_dir) {
  vec3 pos = mymul4x3(a_mWorldViewInv, (ray_pos));
  vec3 pos2 = mymul4x3(a_mWorldViewInv, ((ray_pos) + 100.0f*(ray_dir)));

  vec3 diff = pos2 - pos;

  (ray_pos)  = pos;
  (ray_dir)  = normalize(diff);
}

vec3 EyeRayDirNormalized(float x, float y, mat4 a_mViewProjInv) {
  vec4 pos = vec4(2.0f*x - 1.0f,-2.0f*y + 1.0f,0.0f,1.0f);
  pos = a_mViewProjInv * pos;
  pos /= pos.w;
  return normalize(pos.xyz);
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

#define KGEN_FLAG_RETURN            1
#define KGEN_FLAG_BREAK             2
#define KGEN_FLAG_DONT_SET_EXIT     4
#define KGEN_FLAG_SET_EXIT_NEGATIVE 8
#define KGEN_REDUCTION_LAST_STEP    16
#define CFLOAT_GUARDIAN 
#define MAXFLOAT FLT_MAX

