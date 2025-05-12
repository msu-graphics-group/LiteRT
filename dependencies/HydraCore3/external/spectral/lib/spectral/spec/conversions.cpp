#include <spec/conversions.h> 
#include <spec/spectral_util.h>
#include <internal/common/lazy_value.h>
#include <memory>
#ifdef SPECTRAL_ENABLE_OPENMP
#include <omp.h>
#endif

#include <iostream>
namespace spec {
    namespace {

        const vec3 D6500_WHITE_POINT{95.0489f, 100.0f, 108.8840f};

        Float _xyz2cielab_f(Float t)
        {
            static constexpr Float delta = 6.0f / 29.0f;
            static constexpr Float f_tn = 4.0f / 29.0f;
            static constexpr Float delta_div = 3.0f * delta * delta;
            static constexpr Float delta3 = delta * delta * delta;

            return t > delta3 ? std::cbrt(t) : (t / delta_div + f_tn);
        }
    }

    vec3 spectre2xyz(const ISpectrum &spectrum, const ISpectrum &light)
    {   
        Float cieyint = &light == &util::CIE_D6500 ? util::get_cie_y_integral() : util::get_cie_y_integral(light);
        vec3 xyz{0.0f, 0.0f, 0.0f};

        unsigned idx = 0u;
        for(int lambda = WAVELENGHTS_START; lambda <= WAVELENGHTS_END; lambda += WAVELENGHTS_STEP) {
            const Float val_lv = spectrum(lambda) * light.get_or_interpolate(lambda);
            
            xyz.x += X_CURVE[idx] * val_lv;
            xyz.y += Y_CURVE[idx] * val_lv;
            xyz.z += Z_CURVE[idx] * val_lv;
            idx += 1;
        }
        xyz /= cieyint;
        return xyz;
    }

    vec3 spectre2xyz0(const ISpectrum &spectrum)
    {
        vec3 xyz{0.0f, 0.0f, 0.0f};

        unsigned idx = 0u;
        for(int lambda = WAVELENGHTS_START; lambda <= WAVELENGHTS_END; lambda += WAVELENGHTS_STEP) {
            const Float val_lv = spectrum(lambda);
            
            xyz.x += X_CURVE[idx] * val_lv;
            xyz.y += Y_CURVE[idx] * val_lv;
            xyz.z += Z_CURVE[idx] * val_lv;
            idx += 1;
        }
        return xyz;
    }

    Image spectral_image2rgb(const ISpectralImage &img, const ISpectrum &light)
    {
        Image image{img.get_width(), img.get_height()};
        const unsigned w = img.get_width();
        const unsigned h = img.get_height();
        vec3 rgb;

     //   #pragma omp parallel for private(rgb) shared(image, img)
        for(unsigned j = 0; j < h; ++j) {
            for(unsigned i = 0; i < w; ++i) {
                rgb = spectre2rgb(img.at(i, j), light);
                image.at(i, j) = Pixel::from_vec3(rgb);
            }
        }
        return image;
    }

    vec3 xyz2cielab(const vec3 &xyz)
    {
        const vec3 xyz_norm = xyz / D6500_WHITE_POINT;
        const Float f_x = _xyz2cielab_f(xyz_norm.x);
        const Float f_y = _xyz2cielab_f(xyz_norm.y);
        const Float f_z = _xyz2cielab_f(xyz_norm.z);

        return vec3{
            116.0f * f_y - 16.0f,
            500.0f * (f_x - f_y),
            200.0f * (f_y - f_z)
        };
    }

    vec3 sigpoly2xyz(Float a1, Float a2, Float a3)
    {
        vec3 xyz{0.0f, 0.0f, 0.0f};
        const Float coef[3]{a1, a2, a3};
        unsigned idx = 0u;
        for(int lambda = WAVELENGHTS_START; lambda <= WAVELENGHTS_END; lambda += WAVELENGHTS_STEP) {
            const Float val_lv = math::sigmoid_polynomial(lambda, coef) * util::CIE_D6500.get_or_interpolate(lambda);
            
            xyz.x += X_CURVE[idx] * val_lv;
            xyz.y += Y_CURVE[idx] * val_lv;
            xyz.z += Z_CURVE[idx] * val_lv;
            idx += 1;
        }
        xyz /= util::get_cie_y_integral();
        return xyz;
    }

    std::pair<Float, Float> color2ior(Float r, Float g)
    {
        r = math::clamp(r, 0.0f, 0.9999999f);

        const Float r_sqrt = std::sqrt(r);

        const Float n = g * (1 - r) / (1 + r) + (1 - g) * (1 + r_sqrt) / (1 - r_sqrt);
        const Float k = std::sqrt((r * (n + 1) * (n + 1) - (n - 1) * (n - 1)) / (1 - r));

        return {n, k};
    }

    vec3 edgetint_exp(const vec3 &rgb)
    {
        vec3 diff = vec3(1, 1, 1) - rgb;
        return rgb + diff * 0.5f;
    }

}