#pragma once
#include "LiteMath.h"
#include "Image2d.h"

namespace image_metrics
{
  using LiteMath::float4;

  LiteImage::Image2D<float> to_grayscale(const LiteImage::Image2D<float4> &image);

  float PSNR(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2);
  float PSNR(const LiteImage::Image2D<float4> &image_1, const LiteImage::Image2D<float4> &image_2);

  float SSIM(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2, int win_size, float data_range = 255.0f);
  float SSIM(const LiteImage::Image2D<float4> &image_1, const LiteImage::Image2D<float4> &image_2, int win_size, float data_range = 255.0f);
  float SSIM(const LiteImage::Image2D<float> &image_1, const LiteImage::Image2D<float> &image_2, int win_size, float data_range = 255.0f);

  float FLIP(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2);
  float FLIP(const LiteImage::Image2D<float4> &image_1, const LiteImage::Image2D<float4> &image_2);
}