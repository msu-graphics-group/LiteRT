#include <iostream>
#include <curve.hpp>
#include <LiteMath.h>
#include <Image2d.h>

using namespace LiteMath;
using namespace LiteImage;

int main(int argc, const char **argv) {
  std::filesystem::path exec_path = std::filesystem::canonical(argv[0]);
  std::filesystem::current_path(exec_path.parent_path().parent_path());
  auto proj_path = std::filesystem::current_path(); 

  NURBSCurve2D curve = load_nurbs_curve(proj_path / "resources" / "heart.nurbsc");
  auto rbeziers = curve.decompose();

  auto [boxes, leaves] = get_kdtree_leaves(rbeziers);

  int w = 500, h = 500;
  Image2D<uint32_t> img(w, h);


  for (auto &curve: rbeziers) {
    std::vector<float> monotonic_parts;
    auto monotonic_parts_u = curve.monotonic_parts(0);
    auto monotonic_parts_v = curve.monotonic_parts(1);
    std::copy(monotonic_parts_u.begin(), monotonic_parts_u.end(), std::back_inserter(monotonic_parts));
    std::copy(monotonic_parts_v.begin(), monotonic_parts_v.end(), std::back_inserter(monotonic_parts));
    std::sort(monotonic_parts.begin(), monotonic_parts.end());
    monotonic_parts.resize(std::unique(monotonic_parts.begin(), monotonic_parts.end())-monotonic_parts.begin());
    curve.knots = monotonic_parts;
  }

  for (int y = 0; y < h; ++y)
  {
    float u = y * 1.0f / h;
    std::vector<float> vs;
    for (auto &c: rbeziers) {
      auto cur_vs = c.intersections(u);
      std::copy(cur_vs.begin(), cur_vs.end(), std::back_inserter(vs));
    }
    vs.push_back(0.0f);
    vs.push_back(1.0f);
    std::sort(vs.begin(), vs.end());
    vs.back() = 1.0f;
    vs.resize(std::unique(vs.begin(), vs.end())-vs.begin());
    for (int span = 0; span < vs.size()-1; ++span) {
      uchar4 color = (span % 2 == 0) ? uchar4{0, 255, 0, 255} : uchar4{255, 0, 0, 255};
      int x_min = static_cast<int>(vs[span]*w);
      int x_max = static_cast<int>(vs[span+1]*h);
      for (int x = x_min; x < x_max; ++x) {
        img[int2{x, y}] = color.u32;
      }
    }
  }

  int points_count = 1000;
  bool even = true;
  for (auto &c: rbeziers) {
    for (int span = 0; span < c.knots.size()-1; ++span) {
      float tmin = c.knots[span];
      float tmax = c.knots[span+1];
      for (int point_i = 0; point_i < points_count; ++point_i) {
        float t = lerp(tmin, tmax, point_i * 1.0f/points_count);
        auto p = c.get_point(t);
        p /= p.z;
        int x = static_cast<int>(p.y*w);
        int y = static_cast<int>(p.x*h);
        x = clamp(x, 0, w-1);
        y = clamp(y, 0, h-1);
        for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
        {
          int2 xy = { x+dx, y+dy };
          if (any_of(xy < int2{0, 0}) || any_of(xy >= int2{w, h})) {
            continue;
          }
          uchar4 color = even ? uchar4{0, 0, 0, 255} : uchar4{255, 255, 255, 255};
          img[xy] = color.u32;
        }
      }
      even = !even;
    }
  }

  for (auto &box: boxes) {
    int2 scr_min = int2{
      static_cast<int>(box.boxMin.y * w), 
      static_cast<int>(box.boxMin.x * h)};
    int2 scr_max = int2{
      static_cast<int>(box.boxMax.y*w), 
      static_cast<int>(box.boxMax.x*h)};

    for (int y = scr_min.y; y <= scr_max.y; ++y) {
      img[int2{scr_min.x, y}] = 0;
      img[int2{scr_max.x, y}] = 0;
    }

    for (int x = scr_min.x; x <= scr_max.x; ++x) {
      img[int2{x, scr_min.y}] = 0;
      img[int2{x, scr_max.y}] = 0;
    }
  }

  auto save_path = proj_path / "result.bmp";
  SaveBMP(save_path.c_str(), img.data(), w, h);

  return 0;
}