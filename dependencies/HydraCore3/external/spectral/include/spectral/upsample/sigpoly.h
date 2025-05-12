#ifndef INCLUDE_SPECTRAL_UPSAMPLERS_SIGPOLY_H
#define INCLUDE_SPECTRAL_UPSAMPLERS_SIGPOLY_H
#include <spectral/upsample/upsampler.h>
#include <spectral/spec/sigpoly_lut.h>
#include <spectral/spec/sigpoly_spectrum.h>

namespace spec {
    class SigPolyUpsampler : public IUpsampler
    {
    public:
        SigPolyUpsampler();
        SigPolyUpsampler(SigpolyLUT &&lut0, SigpolyLUT &&lut1, SigpolyLUT &&lut2)
            : luts{std::move(lut0), std::move(lut1), std::move(lut2)} {}

        ISpectralImage::ptr upsample(const Image &sourceImage) const override;
        ISpectrum::ptr upsample_pixel(const Pixel &src) const override;
        ~SigPolyUpsampler() = default;
    private:
        const SigpolyLUT luts[3];
    };
}

#endif