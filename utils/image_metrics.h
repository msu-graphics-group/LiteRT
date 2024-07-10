#include "LiteMath.h"
#include "Image2d.h"

namespace image_metrics
{
  using LiteMath::float4;

  float PSNR(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2);
  float PSNR(const LiteImage::Image2D<float4> &image_1, const LiteImage::Image2D<float4> &image_2);
}