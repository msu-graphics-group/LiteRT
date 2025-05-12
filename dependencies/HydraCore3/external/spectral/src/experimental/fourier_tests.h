#ifndef FOURIER_TESTS_H
#define FOURIER_TESTS_H
#include <spec/spectral_util.h>
#include <spec/basic_spectrum.h>
#include <spec/fourier_spectrum.h>
#include <spec/conversions.h>
#include <internal/serialization/csv.h>
#include <internal/math/math.h>
#include <internal/math/fourier.h>
#include <internal/common/constants.h>
#include <internal/common/format.h>
#include <iostream>
#include <fstream>
#include <cassert>

using namespace spec;

inline void fourier_tests(const char *path)
{
    ISpectrum::ptr spec;
    ISpectrum::csptr light;
    util::load_spectrum(path, spec, light);

    std::vector<Float> wavelenghts;
    std::vector<Float> values;
    for(unsigned i = WAVELENGHTS_START; i <= WAVELENGHTS_END; i += 1) {
        wavelenghts.push_back(Float(i));
        values.push_back(spec->get_or_interpolate(Float(i)));
    }
    std::vector<Float> phases = math::wl_to_phases(wavelenghts);

    std::vector<Float> moments = math::real_fourier_moments_of(phases, values, 4);


    FourierReflectanceSpectrum fspec{std::move(moments)};

    vec3 rgb_gt = spectre2rgb(*spec);
    vec3 rgb = spectre2rgb(fspec);

    std::cout << rgb_gt << " " << rgb << std::endl;

/*
    std::cout << "gt: ";
    for(unsigned i = 0; i < values.size(); ++i) {
        std::cout << values[i] << " ";
    }
    std::cout << std::endl << "fr: ";
    for(unsigned i = 0; i < values.size(); ++i) {
        std::cout << fspec.get_or_interpolate(wavelenghts[i]) << " ";
    }
    std::cout << std::endl;
*/
}

inline void fourier_tests1() {
    std::ifstream file{"input/leds.csv"};

    std::vector<Float> wavelenghts = std::get<0>(*csv::parse_line_m<Float>(file));
    auto data = csv::load_as_vector_m<Float>(file);

    const std::vector<Float> spd = std::get<0>(data[1192]);

    BasicSpectrum spec_gt;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        spec_gt.set(wavelenghts[i], spd[i]);
    }

    std::vector<Float> phases = math::wl_to_phases(wavelenghts);

    std::vector<Float> moments = math::real_fourier_moments_of(phases, spd, 16);

    std::vector<Float> res = math::mese(phases, moments);

    BasicSpectrum spec;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        spec.set(wavelenghts[i], res[i]);
    }

    vec3 rgb_gt = spectre2rgb(spec_gt);
    vec3 rgb = spectre2rgb(spec);
    std::cout << rgb_gt << " " << rgb << std::endl;
}

#endif