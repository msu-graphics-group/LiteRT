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
    int x1 = std::min(std::max( ceilf(W*(pos.x + radius)), 0.0f), W-1.0f);
    int y1 = std::min(std::max( ceilf(H*(pos.y + radius)), 0.0f), H-1.0f);
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

float point_segment_distance(LiteMath::float2 v, LiteMath::float2 w, LiteMath::float2 p) 
{
  // Return minimum distance between line segment vw and point p
  const float l2 = dot(v-w, v-w);  // i.e. |w-v|^2 -  avoid a sqrt
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  const float t = std::max(0.0f, std::min(1.0f, dot(p - v, w - v) / l2));
  const LiteMath::float2 projection = v + t * (w - v);  // Projection falls on the segment
  return LiteMath::length(p - projection);
}

void draw_line(LiteMath::float2 p0, LiteMath::float2 p1, float width, LiteMath::float4 color,
               LiteImage::Image2D<LiteMath::float4> &out_image)
{
  int W = out_image.width();
  int H = out_image.height();
  int x0 = std::min(std::max(floorf(W*(p0.x - width)), 0.0f), W-1.0f);
  int y0 = std::min(std::max(floorf(H*(p0.y - width)), 0.0f), H-1.0f);
  int x1 = std::min(std::max( ceilf(W*(p1.x + width)), 0.0f), W-1.0f);
  int y1 = std::min(std::max( ceilf(H*(p1.y + width)), 0.0f), H-1.0f);

  for (int y = std::min(y0, y1); y <= std::max(y0, y1); y++)
  {
    for (int x = std::min(x0, x1); x <= std::max(x0, x1); x++)
    {
      LiteMath::float2 p = LiteMath::float2((x + 0.5f) / W, (y + 0.5f) / H);
      if (point_segment_distance(p0, p1, p) <= width)
        out_image.data()[y*W + x] = color;
    }
  }
}