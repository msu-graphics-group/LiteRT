#include "points_visualizer.h"

void draw_points(std::vector<LiteMath::float4> &point_positions,
                 std::vector<LiteMath::float4> &point_colors,
                 LiteImage::Image2D<LiteMath::float4> &out_image)
{
  int cnt = std::min(point_positions.size(), point_colors.size());
  int W = out_image.width(), H = out_image.height();

  std::vector<float> sums(W*H, 0.0f);

  for (int i = 0; i < cnt; i++)
  {
    LiteMath::float2 pos = LiteMath::float2(point_positions[i].x, point_positions[i].y);
    float radius = point_positions[i].w;
    int x0 = std::min(std::max(floorf(W*(pos.x - radius)), 0.0f), W-1.0f);
    int y0 = std::min(std::max(floorf(H*(pos.y - radius)), 0.0f), H-1.0f);
    int x1 = std::min(std::max(floorf(W*(pos.x + radius)), 0.0f), W-1.0f);
    int y1 = std::min(std::max(floorf(H*(pos.y + radius)), 0.0f), H-1.0f);
    for (int y = y0; y <= y1; y++)
    {
      for (int x = x0; x <= x1; x++)
      {
        LiteMath::float2 p = LiteMath::float2((x + 0.5f) / W, (y + 0.5f) / H);
        if (dot(p - pos, p - pos) <= radius*radius)
        {
          out_image.data()[y*W + x] = point_colors[i];
          sums[y*W + x] += 1.0f;
        }
      }
    }
  }

  //for (int i = 0; i< W*H; i++)
  //  out_image.data()[i] /= sums[i];
}