#ifndef INCLUDE_SPECTRAL_SPEC_METRICS_H
#define INCLUDE_SPECTRAL_SPEC_METRICS_H
#include <spectral/internal/math/math.h>
#include <spectral/internal/common/constants.h>
#include <spectral/spec/spectrum.h>
#include <spectral/imageutil/image.h>


namespace spec::metrics {

    Float cie_delta_e(const vec3 &y1, const vec3 &y2);

    Float mae(const ISpectrum &y1, const ISpectrum &y2, const std::vector<Float> &lambdas = WAVELENGHTS);
    Float sam(const ISpectrum &y1, const ISpectrum &y2, const std::vector<Float> &lambdas = WAVELENGHTS);

    Float mse(const Image &y1, const Image &y2);
    Float psnr(const Image &y1, const Image &y2);
    Float ssim(const Image &y1, const Image &y2);

}

#endif