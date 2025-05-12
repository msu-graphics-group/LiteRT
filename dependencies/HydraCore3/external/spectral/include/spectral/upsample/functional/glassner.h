#ifndef INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_GLASSNER_H
#define INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_GLASSNER_H
#include <spectral/spec/basic_spectrum.h>
#include <spectral/internal/math/math.h>

namespace spec::upsample {

    constexpr Float GLASSNER_WAVELENGHTS[]{590.0f, 560.0f, 440.0f};
    constexpr size_t GLASSNER_SPECTRUM_SIZE = 3ul;


    BasicSpectrum glassner(const vec3 &rgb);
    inline BasicSpectrum glassner(Float r, Float g, Float b) { return glassner({r, g, b}); }

}

#endif