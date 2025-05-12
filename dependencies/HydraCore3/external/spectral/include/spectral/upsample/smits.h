#ifndef INCLUDE_SPECTRAL_UPSAMPLERS_SMITS_H
#define INCLUDE_SPECTRAL_UPSAMPLERS_SMITS_H
#include <spectral/upsample/upsampler.h>

namespace spec {
    class SmitsUpsampler : public IUpsampler
    {
    public:
        ISpectralImage::ptr upsample(const Image &sourceImage) const override;
        ISpectrum::ptr upsample_pixel(const Pixel &src) const override;
        ~SmitsUpsampler() = default;
    };
}
#endif