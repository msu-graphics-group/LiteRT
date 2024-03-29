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
  vec4 hit_norm; // hit_norm.w is not used
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
};
const uint TYPE_MESH_TRIANGLE = 0;
const uint TYPE_SDF_PRIMITIVE = 1;
struct RenderPreset
{
  bool  isAORadiusInMeters;
  float aoRayLength; // in meters if isAORadiusInMeters is true else in percent of max box size
  int   aoRaysNum;
  int   numBounces;
  bool  measureOverhead;
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
const uint palette_size = 20;
const uint m_palette[20] = {
    0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
    0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
    0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
    0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
    0xff808000, 0xffffd8b1, 0xff000080, 0xff808080
  };
const uint BSIZE = 8;
const float GEPSILON = 2e-5f;
const float DEPSILON = 1e-20f;
const float h = 0.001;

#ifndef SKIP_UBO_INCLUDE
#include "include/EyeRayCaster_generated_ubo.h"
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

vec3 SafeInverse(vec3 d) {
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  vec3 res;
  res.x = 1.0f / (abs(d.x) > ooeps ? d.x : copysign(ooeps, d.x));
  res.y = 1.0f / (abs(d.y) > ooeps ? d.y : copysign(ooeps, d.y));
  res.z = 1.0f / (abs(d.z) > ooeps ? d.z : copysign(ooeps, d.z));
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

uint EXTRACT_COUNT(uint a_leftOffset) { return (a_leftOffset & SIZE_MASK) >> 24; }

vec3 matmul3x3(mat4 m, vec3 v) { 
  return (m*vec4(v, 0.0f)).xyz;
}

bool isLeafAndIntersect(uint flags) { return (flags == (LEAF_BIT | 0x1 )); }

vec3 mymul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

vec3 matmul4x3(mat4 m, vec3 v) {
  return (m*vec4(v, 1.0f)).xyz;
}

bool notLeafAndIntersect(uint flags) { return (flags != (LEAF_BIT | 0x1)); }

bool isLeafOrNotIntersect(uint flags) { return (flags & LEAF_BIT) !=0 || (flags & 0x1) == 0; }

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

vec3 EyeRayDirNormalized(float x, float y, mat4 a_mViewProjInv) {
  vec4 pos = vec4(2.0f*x - 1.0f,-2.0f*y + 1.0f,0.0f,1.0f);
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

uint fakeOffset(uint x, uint y, uint pitch) { return y*pitch + x; }  // RTV pattern, for 2D threading

#define KGEN_FLAG_RETURN            1
#define KGEN_FLAG_BREAK             2
#define KGEN_FLAG_DONT_SET_EXIT     4
#define KGEN_FLAG_SET_EXIT_NEGATIVE 8
#define KGEN_REDUCTION_LAST_STEP    16
#define MAXFLOAT FLT_MAX
#define CFLOAT_GUARDIAN 

