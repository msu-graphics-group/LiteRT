#ifndef INCLUDE_SPECTRAL_UPSAMPLERS_FOURIER_H
#define INCLUDE_SPECTRAL_UPSAMPLERS_FOURIER_H
#include <spectral/upsample/upsampler.h>
#include <spectral/spec/fourier_lut.h>
#include <spectral/spec/fourier_spectrum.h>

#error "Invalid header code"

namespace spec {
    class FourierUpsampler : public IUpsampler
    {
    public:
        FourierUpsampler(bool emiss);
        FourierUpsampler(FourierLUT &&lut, bool emiss)
            : lut{std::move(lut)}, emiss{emiss} {}

        ISpectralImage::ptr upsample(const Image &sourceImage) const override;
        ISpectrum::ptr upsample_pixel(const Pixel &src) const override;
        ~FourierUpsampler() = default;
    private:
        const FourierLUT lut;
        const bool emiss;
    };
}

#endif