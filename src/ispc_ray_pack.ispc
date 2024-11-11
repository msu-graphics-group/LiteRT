#include <embree4/rtcore.isph>

typedef float<4> float4;
typedef float<3> float3;
typedef float<2> float2;

static
float4 rbezier_curve_point(
    float u,
    uniform int p,
    uniform const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  uniform int bc = 1.0f;
  float4 res = pw[0] * _1_u;
  for (uniform int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * pw[i]) * _1_u;
  }
  res += (u_n * u) * pw[p];
  return res;
}

static 
float4 rbezier_curve_der(
    float u,
    uniform int p,
    uniform const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  uniform int bc = 1.0f;
  
  float4 next = pw[1];
  float4 cur = pw[0];
  float4 res = (next-cur) * _1_u;
  if (p > 1)
    cur = next;

  for (uniform int i = 1; i <= p-2; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    next = pw[i+1];
    res = (res + u_n * bc * (next-cur)) * _1_u;
    cur = next;
  }

  next = pw[p];
  res += (u_n * u) * (next-cur);

  res *= p;
  return res;
}

static 
float4 
surface_point(
    float u, float v, 
    uniform int n, uniform int m, 
    const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  uniform int bc = 1.0f;
  float4 res = rbezier_curve_point(v, m, pw) * _1_u;
  for (uniform int i = 1; i <= n-1; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    float4 curve_point = rbezier_curve_point(v, m, pw+(m+1)*i);
    res = (res + u_n * bc * curve_point) * _1_u;
  }
  res += (u_n*u) * rbezier_curve_point(v, m, pw+(m+1)*n);
  return res;
}

static
float4 surface_vder(
    float u,  float v, const float4 &Sw,
    uniform int n, uniform int m, const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  uniform int bc = 1.0f;
  float4 Sw_der = rbezier_curve_der(v, m, pw) * _1_u;
  for (uniform int i = 1; i <= n-1; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    float4 curve_der = rbezier_curve_der(v, m, pw+i*(m+1));
    Sw_der = (Sw_der + u_n * bc * curve_der) * _1_u;
  }
  Sw_der += (u_n*u) * rbezier_curve_der(v, m, pw+n*(m+1));
  
  float4 res = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
  return res;
}

static 
float4 surface_uder(
    float u, float v, const float4 &Sw,
    uniform int n, uniform int m, uniform float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  uniform int bc = 1.0f;
  
  float4 next = rbezier_curve_point(v, m, pw+m);
  float4 cur = rbezier_curve_point(v, m, pw);
  float4 Sw_der = (next-cur) * _1_u;
  if (n > 1)
    cur = next;

  for (uniform int i = 1; i <= n-2; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    next = rbezier_curve_point(v, m, pw+(i+1)*(m+1));
    Sw_der = (Sw_der + u_n * bc * (next-cur)) * _1_u;
    cur = next;
  }

  next = rbezier_curve_point(v, m, pw+n*(m+1));
  Sw_der += (u_n*u) * (next-cur);

  Sw_der *= n;
  
  float4 res = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
  return res;
}

struct HitInfo
{
  bool hitten;
  float3 pos;
  float3 normal;
  float2 uv;
};

inline
float3 normalize(const float3 &ray)
{
  float length = sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
  return ray / length;
}

inline float3 shuffle_yzx(float3 a) { float3 res = { a.y, a.z, a.x}; return res; }

inline
float3 cross(const float3 &a, const float3 &b)
{
  const float3 a_yzx = shuffle_yzx(a);
  const float3 b_yzx = shuffle_yzx(b);
  return shuffle_yzx(a*b_yzx - a_yzx*b);
}

inline
float dot(const float4 &a, const float4 &b)
{
  return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
}

inline
float dot(const float3 &a, const float3 &b)
{
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

inline 
float4 to_float4(const float3 &src, float w) {
  float4 res = { src.x, src.y, src.z, w };
  return res;
}

inline 
float3 to_float3(const float4 &src) {
  float3 res = { src.x, src.y, src.z };
  return res;
}

inline
float2 bezier_project(
    const float4 &P1,
    const float4 &P2,
    const float4 &point) {
  float2 res = { dot(point, P1), dot(point, P2) };
  return res;
}

inline 
float length(float2 val) {
  return sqrt(val.x*val.x+val.y*val.y);
}

#define MAX_STEPS 5
#define EPS 0.01f
static
HitInfo 
trace_surface_newton(
    const float3 &pos,
    const float3 &ray,
    float2 uv,
    uniform int n, uniform int m, const float4 *pw) {
  float3 ortho_dir1 = { 0.0f, 0.0f, 0.0f };
  if (abs(ray.x) > abs(ray.y) && abs(ray.x) > abs(ray.z)) {
    ortho_dir1.x = -ray.y;
    ortho_dir1.y = ray.x;
    ortho_dir1.z = 0.0f;
  } else {
    ortho_dir1.x = 0.0f;
    ortho_dir1.y = -ray.z;
    ortho_dir1.z = ray.y;
  }
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  
  float4 Sw = surface_point(uv.x, uv.y, n, m, pw);
  float4 surf_point = Sw;
  surf_point /= surf_point.w;
  float2 D = bezier_project(P1, P2, surf_point);
  
  HitInfo res;
  res.hitten = false;

  uniform int steps_left = MAX_STEPS-1;
  while(length(D) > EPS && steps_left--) {
    float2 J[2] = 
    { 
      bezier_project(P1, P2, surface_uder(uv.x, uv.y, Sw, n, m, pw)), //col1
      bezier_project(P1, P2, surface_vder(uv.x, uv.y, Sw, n, m, pw)) //col2
    };

    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float2 J_inversed[2] = 
    {
      { J[1][1]/det, -J[0][1]/det },
      { -J[1][0]/det, J[0][0]/det }
    };

    uv = uv - (J_inversed[0]*D[0]+J_inversed[1]*D[1]);
    uv.x = clamp(uv.x, 0.0f, 1.0f);
    uv.y = clamp(uv.y, 0.0f, 1.0f);

    Sw = surface_point(uv.x, uv.y, n, m, pw);
    surf_point = Sw;
    surf_point /= surf_point.w;
    float2 new_D = bezier_project(P1, P2, surf_point);
    
    if (length(new_D) > length(D))
      return res;
    
    D = new_D;
  }

  if (length(D) > EPS)
    return res;
  
  float3 uder = to_float3(surface_uder(uv.x, uv.y, Sw, n, m, pw));
  float3 vder = to_float3(surface_vder(uv.x, uv.y, Sw, n, m, pw));

  res.hitten = true;
  res.pos = to_float3(surf_point);
  res.normal = normalize(cross(uder, vder));
  res.uv = uv;
  return res;
}

struct SurfaceData
{
  int n;
  int m;
  const float4 **pw;
  float2 uv;
};


template<uniform int N>
void surf_intersectN(const RTCIntersectFunctionNArguments * uniform args)
{
  const uniform SurfaceData * uniform data = (const uniform SurfaceData* uniform)(args->geometryUserPtr);
  RTCRayN * uniform rays = RTCRayHitN_RayN(args->rayhit, N);
  RTCHitN * uniform hits = RTCRayHitN_HitN(args->rayhit, N);
  foreach(i = 0...N-1) {
    if (args->valid[i]) {
      float3 pos = { RTCRayN_org_x(rays, N, i), RTCRayN_org_y(rays, N, i), RTCRayN_org_z(rays, N, i) };
      float3 dir = { RTCRayN_dir_x(rays, N, i), RTCRayN_dir_x(rays, N, i), RTCRayN_dir_x(rays, N, i) };
      float2 uv = data->uv;
      HitInfo info = trace_surface_newton(pos, dir, uv, data->n, data->m, data->pw);
      float t = dot(dir, info.pos-pos);
      if (info.hitten && (t >= RTCRayN_tnear(rays, N, i)) && (t <= RTCRayN_tfar(rays, N, i))) {
        RTCRayN_tfar(rays, N, i) = t;

        RTCHitN_geomID(hits, N, i) = args->geomID;
        RTCHitN_primID(hits, N, i) = args->primID;
        RTCHitN_u(hits, N, i) = info.uv[0];
        RTCHitN_v(hits, N, i) = info.uv[1];
        RTCHitN_Ng_x(hits, N, i) = info.normal.x;
        RTCHitN_Ng_y(hits, N, i) = info.normal.y;
        RTCHitN_Ng_z(hits, N, i) = info.normal.z;
      }
    }
  }
}

export
void surf_intersect4(const RTCIntersectFunctionNArguments * uniform args)
{
  surf_intersectN<4>(args);
}

export
void surf_intersect8(const RTCIntersectFunctionNArguments * uniform args)
{
  surf_intersectN<8>(args);
}

export
void surf_intersect16(const RTCIntersectFunctionNArguments * uniform args)
{
  surf_intersectN<16>(args);
}