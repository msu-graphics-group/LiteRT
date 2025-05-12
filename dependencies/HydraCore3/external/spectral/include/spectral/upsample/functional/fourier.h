#ifndef INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_FOURIER_H
#define INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_FOURIER_H
#include <spectral/spec/fourier_spectrum.h>
#include <spectral/internal/math/math.h>
#include <spectral/spec/fourier_lut.h>
#include <spectral/imageutil/pixel.h>

namespace spec::upsample {

    FourierEmissionSpectrum fourier_emiss_int(const Pixel &pixel, Float power, const FourierLUT &lut);
    inline FourierEmissionSpectrum fourier_emiss(const vec3 &rgb, Float power, const FourierLUT &lut) { return fourier_emiss_int(Pixel::from_vec3(rgb), power, lut); }
    inline FourierEmissionSpectrum fourier_emiss(Float r, Float g, Float b, Float power, const FourierLUT &lut) { return fourier_emiss({r, g, b}, power, lut); }


}

#endif