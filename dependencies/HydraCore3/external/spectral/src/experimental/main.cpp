#include <spec/spectral_util.h>
#include <spec/basic_spectrum.h>
#include <spec/conversions.h>
#include <internal/serialization/csv.h>
#include <internal/math/math.h>
#include <internal/common/constants.h>
#include <spec/basic_spectrum.h>
#include <upsample/functional/smits.h>
#include <upsample/sigpoly.h>
#include <internal/common/format.h>
#include <numeric>
#include <iostream>
#include <fstream>
#include <cassert>
#include <set>
#include <map>
#include <iomanip>

#include "fourier_tests.h"

using namespace spec;


//wavelenghts must be sampled to CURVES_WAVELENGHTS values
template<typename T>
math::base_vec3<T> _spectre2xyz(const std::vector<Float> &wavelenghts, const std::vector<T> &values)
{
   static T cieyint = T(util::get_cie_y_integral());

    math::base_vec3<T> xyz{};

    unsigned idx = 0u;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        const T val_lv = values[i] * T(util::CIE_D6500.get_or_interpolate(wavelenghts[i]));
        
        xyz.x += T(util::_interp<X_CURVE>(wavelenghts[i])) * val_lv;
        xyz.y += T(util::_interp<Y_CURVE>(wavelenghts[i])) * val_lv;
        xyz.z += T(util::_interp<Z_CURVE>(wavelenghts[i])) * val_lv;
        idx += 1;
    }
   // xyz /= cieyint;
    return xyz / cieyint;
}

//wavelenghts must be sampled to CURVES_WAVELENGHTS values
template<typename T>
math::base_vec3<T> _spectre2xyz0(const std::vector<Float> &wavelenghts, const std::vector<T> &values)
{
    //static T cieyint = T(util::get_cie_y_integral());

    math::base_vec3<T> xyz{};

    unsigned idx = 0u;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        const T val_lv = values[i];// * T(util::CIE_D6500.get_or_interpolate(wavelenghts[i]));
        
        xyz.x += T(util::_interp<X_CURVE>(wavelenghts[i])) * val_lv;
        xyz.y += T(util::_interp<Y_CURVE>(wavelenghts[i])) * val_lv;
        xyz.z += T(util::_interp<Z_CURVE>(wavelenghts[i])) * val_lv;
        idx += 1;
    }
   // xyz /= cieyint;
    return xyz;/// cieyint;
}


constexpr Float MIN_DIST = 5.0f;

template<typename T>
T _get_cie_y_integral(const std::vector<Float> &wavelenghts, const std::vector<T> &values)
{
    T val = 0.0f;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
    //for(int lambda : wl) {

       // Float delta = 1.0f / Float(wavelenghts.size());
       /* if(i == 0) {
            delta = (wavelenghts[1] - wavelenghts[0]) * 0.5f;
        }
        else if(i == wavelenghts.size() - 1) {
            delta = (wavelenghts.back() - wavelenghts[wavelenghts.size() - 2]) * 0.5f;
        }
        else {
            delta = (wavelenghts[i + 1] - wavelenghts[i - 1]) * 0.5f;
        }
       // std::cout << "delta " << delta << std::endl;
        delta = delta + 1;*/
        val += T(util::_interp<Y_CURVE>(wavelenghts[i])) * values[i];
    }
    return val;
}

template<>
struct std::hash<std::pair<vec3, Float>> {
    size_t operator()(const std::pair<vec3, Float> &obj) const
    {
        return std::hash<vec3>{}(obj.first) * 31 + std::hash<Float>{}(obj.second);
    }
};

bool validate_spec(const std::vector<Float> &spec)
{
    for(Float v : spec) {
        if(v < 0.0f) return false;
    }
    return true;
}

 /*  Float ciey = util::get_cie_y_integral();
        Float max = 1.00f;

        for(unsigned i = 0; i < spec.size(); ++i) {
            if(spec[i] > max) max = spec[i];
        }
      
        for(unsigned i = 0; i < spec.size(); ++i) {
            spec[i] *= 1.0f;// / max;
        }

        vec3 xyz0 = _spectre2xyz0(wavelenghts, spec);
        vec3 rgb0 = xyz2rgb_unsafe(xyz0);

        std::cout << rgb0 << std::endl;

        Float cmax = rgb0[0] > rgb0[1] ? rgb0[0] : rgb0[1];
        cmax = cmax > rgb0[2] ? cmax : rgb0[2];
        //if(cmax < 1.0f) cmax = 1.0f;

        for(unsigned i = 0; i < spec.size(); ++i) {
            spec[i] *= 1 / cmax;
        }

        Float spec_ciey = _get_cie_y_integral(wavelenghts, spec);
        std::vector<Float> uniform25(wavelenghts.size(), 25.0f);
        Float ciey25 = _get_cie_y_integral(wavelenghts, uniform25);

        std::vector<Float> spec_cpy = spec;
        for(unsigned i = 0; i < spec.size(); ++i) {
            spec[i] *= 25;
        }
        */

inline void to_csv(std::ofstream &str, const std::vector<Float> &spec)
{
    for(unsigned i = 0; i < spec.size() - 1; ++i) {
        str << spec[i] << ",";
    }
    str << spec.back() << std::endl;
}

const vec3 COLOR_POWER{27.4722f, 71.8074f, 7.63813f};

void dataset_tests()
{
    std::ifstream file{"input/leds.csv"};

    std::vector<Float> wavelenghts = std::get<0>(*csv::parse_line_m<Float>(file));
    auto data = csv::load_as_vector_m<Float>(file);

    std::ofstream output_sp("output/dataset_spectra.csv");
    std::ofstream output_rgb("output/dataset_rgb.csv");
    std::ofstream output_rgb_val("output/dataset_rgb_val.csv");
    std::ofstream output_sp_val("output/dataset_spectra_val.csv");
    int n = 0;
    assert(wavelenghts.size() != 0);

    const Float ciey = util::get_cie_y_integral();

    const Float uniform_ciey = _get_cie_y_integral(wavelenghts, std::vector<Float>(wavelenghts.size(), 1.0f)); 

    std::vector<Float> d65_1(wavelenghts.size());
    for(unsigned i = 0; i < wavelenghts.size(); ++i) d65_1[i] = util::CIE_D6500.get_or_interpolate(wavelenghts[i]) / ciey * 25.0f * uniform_ciey;
    
    
    //csv head
    output_rgb << "r,g,b" << std::endl << std::setprecision(10);
    output_sp << std::setprecision(16);
    output_rgb_val << "r,g,b" << std::endl << std::setprecision(10);
    output_sp_val << std::setprecision(16);

    to_csv(output_sp, wavelenghts);
    to_csv(output_sp_val, wavelenghts);

    std::set<vec3> used_lab{};
    std::map<vec3, const std::vector<Float> *> specs{{vec3(1.0f, 1.0f, 1.0f), &d65_1}};
    

    for(auto &e : data) {
        std::vector<Float> &spec = std::get<0>(e);
        assert(spec.size() == wavelenghts.size());

        Float illum = _get_cie_y_integral(wavelenghts, spec);

        vec3 xyz0 = _spectre2xyz0(wavelenghts, spec);
        vec3 rgb0 = xyz2rgb_unsafe(xyz0);
        std::cout << "Orig rgb: " << rgb0 << ", illum: " << illum << std::endl;

        for(unsigned i = 0; i < wavelenghts.size(); ++i) {
            spec[i] /= illum;
        }

        vec3 xyz = _spectre2xyz0(wavelenghts, spec);
        vec3 rgb = xyz2rgb_unsafe(xyz);

        for(unsigned i = 0; i < spec.size(); ++i) {
            spec[i] *= uniform_ciey * 25.0f;
        }

        std::cout << rgb << std::endl;
    
        bool t = rgb.min() >= 0.0f; //rgb.max() <= 1.0f;
        if(t && validate_spec(spec)) {
            vec3 lab = rgb2cielab(rgb);
            bool not_used = true;
            for(const auto &l : used_lab) {
                if(vec3::distance(l, lab) < MIN_DIST) {
                    not_used = false;
                    break;
                }
            }

            if(not_used) {
                used_lab.insert(lab);
                bool constraint = true;

                if(constraint) {
                    specs.insert({rgb, &spec});
                }

            }
            else {
                const vec3 rgb_norm = rgb / rgb.max();
                if(rgb_norm.min() > 0.75f) {
                    output_rgb_val << format("%d,%d,%d\n", int(rgb.x * 255), int(rgb.y * 255), int(rgb.z * 255));
                    to_csv(output_sp_val, spec);
                }
               // std::cout << format("Excluded %d, %f, %f, %f via DE", n, rgb.x, rgb.y, rgb.z) << std::endl;
            }
        }
        else {
           // std::cout << format("Excluded %d, %f, %f, %f", n, rgb.x, rgb.y, rgb.z) << std::endl;
        }
        n += 1;
    }
    
    for(const auto &[color, spec] : specs) {

        output_rgb << format("%d,%d,%d\n", int(color.x * 255), int(color.y * 255), int(color.z * 255));

        to_csv(output_sp, *spec);
    }

    output_sp.close();
    output_rgb.close();
    output_sp_val.close();
    output_rgb_val.close();
}


const BasicSpectrum UNIFORM {
    {WAVELENGHTS_START, 1.0f},
    {0.5f * (WAVELENGHTS_START + WAVELENGHTS_END), 1.0f},
    {WAVELENGHTS_END, 1.0f}
};


int main(int argc, char **argv)
{
    (void) argc; (void) argv;
/*

    std::vector<Float> wavelenghts;
    for(int i = WAVELENGHTS_START; i <= WAVELENGHTS_END; ++i) {
        wavelenghts.push_back(Float(i));
    }
    BasicSpectrum spec = upsample::smits({1.0f, 1.0f, 1.0f});
    std::vector<Float> val(wavelenghts.size());
    for(unsigned i = 0; i < wavelenghts.size(); ++i) val[i] = spec(wavelenghts[i]);

    std::cout << _get_cie_y_integral(wavelenghts, val);
*/


    dataset_tests();

    //SigPolyUpsampler upsampler{};
    //ISpectrum::ptr spec = upsampler.upsample_pixel({255, 255, 255});
    //std::cout << util::get_cie_y_integral(*spec) << " " << spectre2rgb(*spec) << std::endl;

    return 0;
} 

