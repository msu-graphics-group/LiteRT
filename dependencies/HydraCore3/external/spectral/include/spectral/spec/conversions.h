#ifndef INCLUDE_SPECTRAL_SPEC_CONVERSIONS_H
#define INCLUDE_SPECTRAL_SPEC_CONVERSIONS_H
#include <spectral/internal/math/math.h>
#include <spectral/spec/basic_spectrum.h>
#include <spectral/spec/spectral_util.h>
#include <spectral/imageutil/image.h>
#include <utility>

namespace spec {

    inline vec3 rgb2xyz(const vec3 &rgb)
    {
        return RGB_TO_XYZ * rgb;
    }

    inline vec3 xyz2rgb_unsafe(const vec3 &xyz)
    {
        return XYZ_TO_RGB * xyz;
    }

    inline vec3 xyz2rgb(const vec3 &xyz)
    {
        return math::clamp(XYZ_TO_RGB * xyz, 0.0f, 1.0f);
    }

    vec3 xyz2cielab(const vec3 &xyz);

    inline vec3 rgb2cielab(const vec3 &rgb) 
    {
        return xyz2cielab(rgb2xyz(rgb));
    }

    vec3 spectre2xyz(const ISpectrum &spectre, const ISpectrum &light = util::CIE_D6500);

    vec3 spectre2xyz0(const ISpectrum &spectre);

    Image spectral_image2rgb(const ISpectralImage &img, const ISpectrum &light = util::CIE_D6500);


    inline vec3 spectre2rgb(const ISpectrum &spectre, const ISpectrum &light = util::CIE_D6500)
    {
        return xyz2rgb(spectre2xyz(spectre, light));
    }

    vec3 sigpoly2xyz(Float a1, Float a2, Float a3);


    //Approximation based on http://jcgt.org/published/0003/04/03/paper.pdf
    std::pair<Float, Float> color2ior(Float reflectivity, Float edgetint);

    inline std::pair<Float, Float> color2ior(Float color)
    {   
        return color2ior(color, color);
    }

    vec3 edgetint_exp(const vec3 &rgb);


}
#endif