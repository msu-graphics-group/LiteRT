#include <cassert>

#include "embree_adaptors.hpp"

using namespace LiteMath;

namespace embree
{
  void errorFunction(void* userPtr, enum RTCError error, const char* str)
  {
    printf("error %d: %s\n", error, str);
  }

  void rbgrid_bounds_function(const RTCBoundsFunctionArguments *args) {
    const RBGridView &view = *reinterpret_cast<const RBGridView*>(args->geometryUserPtr);
    int id = args->primID;
    BoundingBox3d bbox = (*view.p_boxes)[id];

    args->bounds_o->lower_x = bbox.mn.x;
    args->bounds_o->lower_y = bbox.mn.y;
    args->bounds_o->lower_z = bbox.mn.z;

    args->bounds_o->upper_x = bbox.mx.x;
    args->bounds_o->upper_y = bbox.mx.y;
    args->bounds_o->upper_z = bbox.mx.z;
  }

  void rbgrid_intersect_function(const RTCIntersectFunctionNArguments *args) {
    const RBGridView &view = *reinterpret_cast<const RBGridView*>(args->geometryUserPtr);
    if (!view.p_grid->is_visible)
      return;
    assert(args->N == 1);
    if (!args->valid[0])
      return;

    auto &ray_hit = const_cast<RTCRayHit&>(*reinterpret_cast<const RTCRayHit*>(args->rayhit));
    float3 pos{ ray_hit.ray.org_x, ray_hit.ray.org_y, ray_hit.ray.org_z };
    float3 dir{ ray_hit.ray.dir_x, ray_hit.ray.dir_y, ray_hit.ray.dir_z };

    int id = args->primID;
    float2 uv = (*view.p_uvs)[id];

    auto info = trace_surface_newton(pos, dir, *view.p_grid, uv);
    if (!info.hitten)
      return;

    float t = dot(dir, info.pos-pos);
    if (t < ray_hit.ray.tnear || t > ray_hit.ray.tfar)
      return;
    ray_hit.ray.tfar = t;

    auto &hit = ray_hit.hit;
    hit.geomID = args->geomID;
    hit.primID = args->primID;
    hit.u = info.uv[0];
    hit.v = info.uv[1];
    hit.Ng_x = info.normal.x;
    hit.Ng_y = info.normal.y;
    hit.Ng_z = info.normal.z;
  }

  void EmbreeScene::attach_mesh(const Mesh &mesh) {
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    float3 *vertices = reinterpret_cast<float3*>(rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 
        sizeof(float3), mesh.indices.size()));
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
      vertices[i] = to_float3(mesh.vertices[i]/mesh.vertices[i].w);
    }

    uint32_t *indices = reinterpret_cast<uint32_t*>(rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 
        sizeof(uint32_t)*3, mesh.indices.size()/3));
    std::copy(mesh.indices.begin(), mesh.indices.end(), indices);

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scn, geom);
    rtcReleaseGeometry(geom);
  }

  void EmbreeScene::draw(const Camera &camera, FrameBuffer &fb, std::function<ShadeFuncType> shade_func) const {
    float4x4 mat  = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.001f, 100.0f)
                  * lookAt(camera.position, camera.target, camera.up);
    float4x4 inversed_mat = inverse4x4(mat);
    #pragma omp parallel for schedule(dynamic)
    for (uint32_t y = 0; y < fb.col_buf.height(); ++y)
    for (uint32_t x = 0; x < fb.col_buf.width();  ++x)
    {
      float2 ndc_point  = float2{ x+0.5f, y+0.5f } 
                        / float2{ fb.col_buf.width()*1.0f, fb.col_buf.height()*1.0f }
                        * 2.0f
                        - 1.0f;
      float4 ndc_point4 = { ndc_point.x, ndc_point.y, 0.0f, 1.0f };
      float4 point = inversed_mat * ndc_point4;
      point /= point.w;

      float3 ray = normalize(to_float3(point)-camera.position);
      float3 pos = camera.position;
      
      struct RTCRayHit rayhit;
      rayhit.ray.org_x = pos.x;
      rayhit.ray.org_y = pos.y;
      rayhit.ray.org_z = pos.z;
      rayhit.ray.dir_x = ray.x;
      rayhit.ray.dir_y = ray.y;
      rayhit.ray.dir_z = ray.z;
      rayhit.ray.tnear = 0;
      rayhit.ray.tfar = std::numeric_limits<float>::infinity();
      rayhit.ray.mask = -1;
      rayhit.ray.flags = 0;
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

      rtcIntersect1(scn, &rayhit);
      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        float t = rayhit.ray.tfar;
        uint2 xy = uint2{ x, fb.col_buf.height()-1-y };

        if (fb.z_buf[xy] > t) {
          HitInfo info;
          info.hitten = true;
          info.pos = pos + ray * t;
          info.uv = float2{ rayhit.hit.u, rayhit.hit.v };
          info.normal = float3 {
            rayhit.hit.Ng_x,
            rayhit.hit.Ng_y,
            rayhit.hit.Ng_z
          };
          float4 color = shade_func(info, pos);
          fb.z_buf[xy] = t;
          fb.col_buf[xy] = uchar4{ 
            static_cast<u_char>(color[0]*255.0f),
            static_cast<u_char>(color[1]*255.0f),
            static_cast<u_char>(color[2]*255.0f),
            static_cast<u_char>(color[3]*255.0f) }.u32;
        }
      }
    }
  }
} // namespace embree