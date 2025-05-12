#include <upsample/functional/glassner.h>
#include <spec/conversions.h>
#include <internal/math/math.h>

namespace spec::upsample {

    namespace {

        const mat3 XYZ_TO_SPECTRE_INV = math::inverse(mat3{ //C
            1.026f, 0.757f, 0.001f,
            0.594f, 0.995f, 0.004f,
            0.348f, 0.023f, 1.747f
        });

        void upsample_to(const vec3 &rgb, BasicSpectrum &spectrum)
        {
            vec3 ampls = math::clamp(rgb2xyz(rgb) * XYZ_TO_SPECTRE_INV, 0.0f, 1.0f);
       
            spectrum.get_or_create(GLASSNER_WAVELENGHTS[0]) = ampls[0];
            spectrum.get_or_create(GLASSNER_WAVELENGHTS[1]) = ampls[1];
            spectrum.get_or_create(GLASSNER_WAVELENGHTS[2]) = ampls[2];
        }

    }


    BasicSpectrum glassner(const vec3 &rgb)
    {
        BasicSpectrum spectrum;
        upsample_to(rgb, spectrum);
        return spectrum;
    }

}