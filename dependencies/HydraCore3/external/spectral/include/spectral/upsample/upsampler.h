#ifndef INCLUDE_SPECTRAL_UPSAMPLE_UPSAMPLER_H
#define INCLUDE_SPECTRAL_UPSAMPLE_UPSAMPLER_H
#include <spectral/spec/spectrum.h>
#include <spectral/imageutil/image.h>
#include <memory>

namespace spec {

    class IUpsampler
    {
    public:
        virtual ISpectralImage::ptr upsample(const Image &sourceImage) const = 0;
        virtual ISpectrum::ptr upsample_pixel(const Pixel &src) const = 0;

        virtual ~IUpsampler() {}

        using ptr = std::unique_ptr<IUpsampler>;
        using shared_ptr = std::shared_ptr<IUpsampler>;
    };

}


#endif