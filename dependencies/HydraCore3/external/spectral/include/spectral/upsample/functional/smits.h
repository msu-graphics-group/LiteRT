#ifndef INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_SMITS_H
#define INCLUDE_SPECTRAL_UPSAMPLE_FUNCTIONAL_SMITS_H
#include <spectral/spec/basic_spectrum.h>
#include <spectral/internal/math/math.h>

namespace spec::upsample {

    constexpr Float SMITS_WAVELENGHTS[] = {
        397, 431, 465, 499, 533,
        567, 601, 635, 669, 703
    };
    constexpr size_t SMITS_SPECTRUM_SIZE = sizeof(SMITS_WAVELENGHTS) / sizeof(Float); 

    BasicSpectrum smits(const vec3 &rgb);
    inline BasicSpectrum smits(Float r, Float g, Float b) { return smits({r, g, b}); }

}

#endif