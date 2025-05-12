#ifndef INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_SIGPOLY_H
#define INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_SIGPOLY_H
#include <spectral/spec/sigpoly_spectrum.h>
#include <spectral/internal/math/math.h>
#include <spectral/spec/sigpoly_lut.h>
#include <spectral/imageutil/pixel.h>

namespace spec::upsample {

    SigPolySpectrum sigpoly_int(const Pixel &pixel, const SigpolyLUT lut[3]);
    inline SigPolySpectrum sigpoly(const vec3 &rgb, const SigpolyLUT lut[3]) { return sigpoly_int(Pixel::from_vec3(rgb), lut); }
    inline SigPolySpectrum sigpoly(Float r, Float g, Float b, const SigpolyLUT lut[3]) { return sigpoly({r, g, b}, lut); }


}

#endif