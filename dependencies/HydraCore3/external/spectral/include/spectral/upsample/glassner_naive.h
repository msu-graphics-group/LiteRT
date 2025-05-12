#ifndef INCLUDE_SPECTRAL_UPSAMPLERS_GLASSNER_NAIVE_H
#define INCLUDE_SPECTRAL_UPSAMPLERS_GLASSNER_NAIVE_H
#include <spectral/upsample/upsampler.h>
#include <spectral/spec/basic_spectrum.h>

namespace spec {

    class GlassnerUpsampler : public IUpsampler
    {
    public:
        ISpectralImage::ptr upsample(const Image &sourceImage) const override;
        ISpectrum::ptr upsample_pixel(const Pixel &pixel) const override;
        ~GlassnerUpsampler() = default;
    };
}

#endif