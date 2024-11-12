#include <cassert>

#include "embree_adaptors.hpp"
#include "ispc_ray_pack_ispc.h"

using namespace LiteMath;

namespace embree
{
  void errorFunction(void* userPtr, enum RTCError error, const char* str)
  {
    printf("error %d: %s\n", error, str);
  }

  void EmbreeScene::attach_surface(
          const RBezierGrid &rbezier, 
          const std::vector<BoundingBox3d> &boxes,
          const std::vector<LiteMath::float2> &uvs) {
    views.push_back(RBGridView{ &rbezier, &boxes, &uvs });
    auto &view = views.back();
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryUserPrimitiveCount(geom, boxes.size());
    rtcSetGeometryUserData(geom, &view);
    rtcSetGeometryBoundsFunction(geom, rbgrid_bounds_function, nullptr);
    rtcSetGeometryIntersectFunction(geom, rbgrid_intersect_function);
    //rtcSetGeometryOccludedFunction(geom, rbgrid_occluded_function);
    rtcCommitGeometry(geom);
    rtcAttachGeometry(scn, geom);
    rtcReleaseGeometry(geom);
  }

  void EmbreeScene::attach_boxes(
        const std::vector<BoundingBox3d> &boxes,
        const std::vector<LiteMath::float2> &uvs) {
    views.push_back(RBGridView{ nullptr, &boxes, &uvs });
    auto &view = views.back();
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
    rtcSetGeometryUserPrimitiveCount(geom, boxes.size());
    rtcSetGeometryUserData(geom, &view);
    rtcSetGeometryBoundsFunction(geom, rbgrid_bounds_function, nullptr);
    rtcSetGeometryIntersectFunction(geom, boxes_intersect_function);
    //rtcSetGeometryOccludedFunction(geom, rbgrid_occluded_function);
    rtcCommitGeometry(geom);
    rtcAttachGeometry(scn, geom);
    rtcReleaseGeometry(geom);
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

  void rbgrid_intersect1_function(const RTCIntersectFunctionNArguments *args) {
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

  ispc::SurfaceData get_data(const RBezierGrid &grid, float2 uv)
  {
    auto spans = grid.get_spans(uv.x, uv.y);
    auto uspan = spans[0], vspan = spans[1];
    auto &rbezier = grid.grid[{uspan, vspan}];
    float umin = grid.uniq_uknots[uspan], umax = grid.uniq_uknots[uspan+1];
    float vmin = grid.uniq_vknots[vspan], vmax = grid.uniq_vknots[vspan+1];
    float u = (uv[0]-umin)/(umax-umin);
    float v = (uv[1]-vmin)/(vmax-vmin);
    return ispc::SurfaceData {
      rbezier.weighted_points.get_n()-1,
      rbezier.weighted_points.get_m()-1,
      reinterpret_cast<const ispc::float4*>(rbezier.weighted_points.data()),
      u, v
    };
  }

  void rbgrid_intersect_function(const RTCIntersectFunctionNArguments *args) {
    int N = args->N;
    assert(N == 1 || N == 4 || N == 8 || N == 16);

    const ispc::RTCIntersectFunctionNArguments *ispc_args = reinterpret_cast<const ispc::RTCIntersectFunctionNArguments*>(args);
    const RBGridView &view = *reinterpret_cast<const RBGridView*>(args->geometryUserPtr);
    float2 uv = (*view.p_uvs)[args->primID];
    
    if (!view.p_grid->is_visible)
      return;
    
    auto data = get_data(*view.p_grid, uv);
    switch (args->N)
    {
    case 4: ispc::surf_intersect4(ispc_args, &data); break;
    case 8: ispc::surf_intersect8(ispc_args, &data); break;
    case 16: ispc::surf_intersect16(ispc_args, &data); break;
    case 1: default: rbgrid_intersect1_function(args); break; 
    };
  }

  void boxes_intersect_function(const RTCIntersectFunctionNArguments *args) {
    const RBGridView &view = *reinterpret_cast<const RBGridView*>(args->geometryUserPtr);
    assert(args->N == 1);
    if (!args->valid[0])
      return;

    auto &ray_hit = const_cast<RTCRayHit&>(*reinterpret_cast<const RTCRayHit*>(args->rayhit));
    float3 pos{ ray_hit.ray.org_x, ray_hit.ray.org_y, ray_hit.ray.org_z };
    float3 dir{ ray_hit.ray.dir_x, ray_hit.ray.dir_y, ray_hit.ray.dir_z };

    int id = args->primID;
    float2 uv = (*view.p_uvs)[id];

    auto tbounds = (*view.p_boxes)[id].tbounds(pos, dir);
    float t = tbounds[0] >= 0 ? tbounds[0] : tbounds[1];
    if (t < ray_hit.ray.tnear || t > ray_hit.ray.tfar)
      return;
    ray_hit.ray.tfar = t;

    auto &hit = ray_hit.hit;
    hit.geomID = args->geomID;
    hit.primID = args->primID;
    hit.u = uv[0];
    hit.v = uv[1];
  }

  template<RayPackSize size>
  struct EmbreeRayHit;
  template<>
  struct EmbreeRayHit<RayPackSize::RAY_PACK_1> { using type = RTCRayHit; };
  template<>
  struct EmbreeRayHit<RayPackSize::RAY_PACK_4> { using type = RTCRayHit4; };
  template<>
  struct EmbreeRayHit<RayPackSize::RAY_PACK_8> { using type = RTCRayHit8; };
  template<>
  struct EmbreeRayHit<RayPackSize::RAY_PACK_16> { using type = RTCRayHit16; };

  template<RayPackSize size>
  struct EmbreeIntersect;
  template<>
  struct EmbreeIntersect<RayPackSize::RAY_PACK_1> {  
    static auto make_func() {  
      return [](int *valid, RTCScene scene, RTCRayHit *rayhit) {
        rtcIntersect1(scene, rayhit);
      };
    }
  };
  
  template<>
  struct EmbreeIntersect<RayPackSize::RAY_PACK_4> {  
    static auto make_func() {  
      return [](int *valid, RTCScene scene, RTCRayHit4 *rayhit) {
        rtcIntersect4(valid, scene, rayhit);
      };
    }
  };

  template<>
  struct EmbreeIntersect<RayPackSize::RAY_PACK_8> {  
    static auto make_func() {  
      return [](int *valid, RTCScene scene, RTCRayHit8 *rayhit) {
        rtcIntersect8(valid, scene, rayhit);
      };
    }
  };

  template<>
  struct EmbreeIntersect<RayPackSize::RAY_PACK_16> {  
    static auto make_func() {  
      return [](int *valid, RTCScene scene, RTCRayHit16 *rayhit) {
        rtcIntersect16(valid, scene, rayhit);
      };
    }
  };

  template<RayPackSize size>
  void EmbreeScene::drawN(
      const Camera &camera, 
      FrameBuffer &fb, 
      std::function<ShadeFuncType> shade_func) const {
    auto intersect_f = EmbreeIntersect<size>::make_func();
    float4x4 mat  = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.001f, 1000.0f)
                * lookAt(camera.position, camera.target, camera.up);
    float4x4 inversed_mat = inverse4x4(mat);
    int ray_pack_dim = std::sqrt(static_cast<int>(size));
    #pragma omp parallel for schedule(dynamic)
    for (uint32_t y = 0; y < fb.col_buf.height(); y += ray_pack_dim)
    for (uint32_t x = 0; x < fb.col_buf.width();  x += ray_pack_dim) {
      typename EmbreeRayHit<size>::type ray_hit;

      int valid[16]; 
      std::fill(valid, valid+16, 1);

      for (uint32_t dx = 0; dx < ray_pack_dim; ++dx)
      for (uint32_t dy = 0; dy < ray_pack_dim; ++dy)
      {
        uint32_t cur_x = x+dx;
        uint32_t cur_y = y+dy;
        if (cur_x >= fb.col_buf.width() || cur_y >= fb.col_buf.height()) {
          valid[ray_pack_dim*dy+dx] = false;
          continue;
        }

        float2 ndc_point  = float2{ x+0.5f, y+0.5f } 
                        / float2{ fb.col_buf.width()*1.0f, fb.col_buf.height()*1.0f }
                        * 2.0f
                        - 1.0f;
        float4 ndc_point4 = { ndc_point.x, ndc_point.y, 0.0f, 1.0f };
        float4 point = inversed_mat * ndc_point4;
        point /= point.w;

        float3 ray = normalize(to_float3(point)-camera.position);
        float3 pos = camera.position;

        ray_hit.ray.org_x[dy*ray_pack_dim+dx] = pos.x;
        ray_hit.ray.org_y[dy*ray_pack_dim+dx] = pos.y;
        ray_hit.ray.org_z[dy*ray_pack_dim+dx] = pos.z;
        ray_hit.ray.dir_x[dy*ray_pack_dim+dx] = ray.x;
        ray_hit.ray.dir_y[dy*ray_pack_dim+dx] = ray.y;
        ray_hit.ray.dir_z[dy*ray_pack_dim+dx] = ray.z;
        ray_hit.ray.tnear[dy*ray_pack_dim+dx] = 0.0f;
        ray_hit.ray.tfar[dy*ray_pack_dim+dx]  = std::numeric_limits<float>::infinity();
        ray_hit.hit.geomID[ray_pack_dim*dy+dx] = RTC_INVALID_GEOMETRY_ID;
        ray_hit.ray.mask[ray_pack_dim*dy+dx] = -1;
        ray_hit.ray.flags[ray_pack_dim*dy+dx] = 0;
      }

      intersect_f(valid, scn, &ray_hit);

      for (uint32_t dx = 0; dx < ray_pack_dim; ++dx)
      for (uint32_t dy = 0; dy < ray_pack_dim; ++dy)
      {
        int idx = dy*ray_pack_dim+dx;
        if (ray_hit.hit.geomID[idx] != RTC_INVALID_GEOMETRY_ID) {
          HitInfo info;
          info.hitten = true;
          info.uv = float2{ ray_hit.hit.u[idx], ray_hit.hit.v[idx] };
          info.normal = float3{ ray_hit.hit.Ng_x[idx], ray_hit.hit.Ng_y[idx], ray_hit.hit.Ng_z[idx] };
          info.pos = float3{ ray_hit.ray.org_x[idx], ray_hit.ray.org_y[idx], ray_hit.ray.org_z[idx] }
                   + float3{ ray_hit.ray.dir_x[idx], ray_hit.ray.dir_y[idx], ray_hit.ray.dir_z[idx] } 
                      * ray_hit.ray.tfar[idx];
                      
          float2 uv = info.uv;
          float3 point = info.pos;
          float t = length(point-camera.position);
          uint2 xy = uint2{ x+dx, fb.col_buf.height()-1-y-dy };

          if (fb.z_buf[xy] > t) {
            float4 color = shade_func(info, camera.position);
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
  }

  void EmbreeScene::draw(
      const Camera &camera, 
      FrameBuffer &fb, 
      std::function<ShadeFuncType> shade_func, 
      RayPackSize ray_pack) const {
    switch (ray_pack)
    {
    case RayPackSize::RAY_PACK_1: draw1(camera, fb, shade_func); break;
    case RayPackSize::RAY_PACK_4: drawN<RayPackSize::RAY_PACK_4>(camera, fb, shade_func); break;
    case RayPackSize::RAY_PACK_8: drawN<RayPackSize::RAY_PACK_8>(camera, fb, shade_func); break;
    case RayPackSize::RAY_PACK_16: drawN<RayPackSize::RAY_PACK_16>(camera, fb, shade_func); break;
    }
  }

  void EmbreeScene::draw1(
      const Camera &camera, 
      FrameBuffer &fb, 
      std::function<ShadeFuncType> shade_func) const {
    float4x4 mat  = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.001f, 1000.0f)
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
      
      
      RTCRayHit ray_hit;
      ray_hit.ray.org_x = pos.x;
      ray_hit.ray.org_y = pos.y;
      ray_hit.ray.org_z = pos.z;
      ray_hit.ray.dir_x = ray.x;
      ray_hit.ray.dir_y = ray.y;
      ray_hit.ray.dir_z = ray.z;
      ray_hit.ray.tnear = 0.0f;
      ray_hit.ray.mask = -1;
      ray_hit.ray.flags = 0;
      ray_hit.ray.tfar = std::numeric_limits<float>::infinity();
      ray_hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rtcIntersect1(scn, &ray_hit);
      if (ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        HitInfo info;
        info.hitten = true;
        info.uv = float2{ ray_hit.hit.u, ray_hit.hit.v };
        info.normal = float3{ ray_hit.hit.Ng_x, ray_hit.hit.Ng_y, ray_hit.hit.Ng_z };
        info.pos = pos + ray * ray_hit.ray.tfar;

        float2 uv = info.uv;
        float3 point = info.pos;
        float t = length(point-camera.position);
        uint2 xy = uint2{ x, fb.col_buf.height()-1-y };

        if (fb.z_buf[xy] > t) {
          float4 color = shade_func(info, camera.position);
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