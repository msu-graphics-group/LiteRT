// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
#ifndef NURBS_SRC_EMBREE_ADAPTORS
#define NURBS_SRC_EMBREE_ADAPTORS

#include <list>
#include <embree4/rtcore.h>
#include <LiteMath.h>
#include <Image2d.h>
#include <map>

#include "Surface.hpp"
#include "raytracer.hpp"

namespace embree
{
  struct RBGridView
  {
    const RBezierGrid *p_grid;
    const std::vector<BoundingBox3d> *p_boxes;
    const std::vector<LiteMath::float2> *p_uvs;
  };
  void rbgrid_bounds_function(const RTCBoundsFunctionArguments *args);
  void rbgrid_intersect_function(const RTCIntersectFunctionNArguments *args);
  void boxes_intersect_function(const RTCIntersectFunctionNArguments *args);
  void rbgrid_occluded_function(const RTCOccludedFunctionNArguments *args);

  enum class RayPackSize
  {
    RAY_PACK_1 = 1,
    RAY_PACK_4 = 4,
    RAY_PACK_8 = 8,
    RAY_PACK_16 = 16
  };

  void errorFunction(void* userPtr, enum RTCError error, const char* str);
  struct EmbreeScene
  {
  public:
    EmbreeScene() {
      device = rtcNewDevice(nullptr);
#ifndef NDEBUG
      rtcSetDeviceErrorFunction(device, errorFunction, nullptr);
#endif
      scn = rtcNewScene(device);
    }
  public:
    void attach_surface(
        const RBezierGrid &rbezier, 
        const std::vector<BoundingBox3d> &boxes,
        const std::vector<LiteMath::float2> &uvs);
    void attach_mesh(const Mesh &mesh);
    void attach_boxes(
      const std::vector<BoundingBox3d> &boxes,
      const std::vector<LiteMath::float2> &uvs);
  public:
    void clear_scene() {
      rtcReleaseScene(scn);
      views.resize(0);
      scn = rtcNewScene(device);
    }
  public:
    void commit_scene() {
      rtcCommitScene(scn);
    }
  public:
    ~EmbreeScene() {
      rtcReleaseScene(scn);
      rtcReleaseDevice(device);
    }
  public:
    void draw(const Camera &camera, FrameBuffer &fb, std::function<ShadeFuncType> shade_func = shade_uv,
                RayPackSize ray_pack = RayPackSize::RAY_PACK_1) const;
    void draw_triangles(const Camera &camera, FrameBuffer &fb, std::function<ShadeFuncType> shade_func = shade_uv) const;
  private:
    void draw1(const Camera &camera, FrameBuffer &fb, std::function<ShadeFuncType> shade_func) const;
    template<RayPackSize size>
    void drawN(const Camera &camera, FrameBuffer &fb, std::function<ShadeFuncType> shade_func) const;
  private:
    std::list<RBGridView> views;
    RTCDevice device;
    RTCScene scn;
  };
} // namespace embree

#endif

