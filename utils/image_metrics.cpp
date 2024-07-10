#include "image_metrics.h"

namespace image_metrics
{
  using LiteMath::uint4;
  uint32_t encode_RGBA8(float4 c)
  {
    uint4 col = uint4(255 * clamp(c, float4(0, 0, 0, 0), float4(1, 1, 1, 1)));
    return (col.w << 24) | (col.z << 16) | (col.y << 8) | col.x;
  }

  float4 decode_RGBA8(uint32_t c)
  {
    uint4 col = uint4(c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF, (c >> 24) & 0xFF);
    return float4(col.x * (1.0f / 255.0f), col.y * (1.0f / 255.0f), col.z * (1.0f / 255.0f), col.w * (1.0f / 255.0f));
  }

  float PSNR(const LiteImage::Image2D<float4> &image_1, const LiteImage::Image2D<float4> &image_2)
  {
    assert(image_1.vector().size() == image_2.vector().size());
    unsigned sz = image_1.vector().size();
    double sum = 0.0;
    for (int i = 0; i < sz; i++)
    {
      float r1 = image_1.vector()[i].x;
      float g1 = image_1.vector()[i].y;
      float b1 = image_1.vector()[i].z;
      float r2 = image_2.vector()[i].x;
      float g2 = image_2.vector()[i].y;
      float b2 = image_2.vector()[i].z;
      sum += ((r1 - r2) * (r1 - r2) + (g1 - g2) * (g1 - g2) + (b1 - b2) * (b1 - b2)) / (3.0f);
    }
    float mse = sum / sz;

    return -10 * log10(std::max<double>(1e-10, mse));
  }

  float PSNR(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2)
  {
    assert(image_1.width() == image_2.width());
    assert(image_1.height() == image_2.height());
    unsigned sz = image_1.vector().size();

    LiteImage::Image2D<float4> image_1_f(image_1.width(), image_1.height(), float4(0, 0, 0, 0));
    LiteImage::Image2D<float4> image_2_f(image_1.width(), image_1.height(), float4(0, 0, 0, 0));

    for (int i = 0; i < sz; i++)
    {
      image_1_f.data()[i] = decode_RGBA8(image_1.vector()[i]);
      image_2_f.data()[i] = decode_RGBA8(image_2.vector()[i]);
    }

    return PSNR(image_1_f, image_2_f);
  }
}