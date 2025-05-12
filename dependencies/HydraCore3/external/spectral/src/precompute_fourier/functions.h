#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <internal/math/fourier.h>
#include <internal/math/levinson.h>
#include <internal/common/constants.h>
#include <spec/spectral_util.h>
#include <vector>
#include <optional>
#include <utility>
#include <iostream>

#ifndef PRECOMPUTE_M
#define PRECOMPUTE_M 16
#endif

using namespace spec;
using math::base_vec3;


constexpr double XYZ_TO_CIELAB_XYZN[3]{95.0489f, 100.0f, 108.8840f};
constexpr int M = PRECOMPUTE_M;
extern const Float CIEY_UNIFORM;
extern const vec3 COLOR_POWER;

template<typename T>
std::vector<T> &operator*=(std::vector<T> &vec, T val)
{
    for(T &v : vec) v *= val;
    return vec;
}

template<typename T>
std::vector<T> operator*(const std::vector<T> &vec, T val)
{
    std::vector<T> copy = vec;
    return copy *= val;
}

template<typename T>
base_vec3<T> _xyz2rgb_unsafe(const base_vec3<T> &xyz)
{
    return {3.2404542 * xyz.x - 1.5371385 * xyz.y - 0.4985314 * xyz.z,
           -0.9692660 * xyz.x + 1.8760108 * xyz.y + 0.0415560 * xyz.z,
            0.0556434 * xyz.x - 0.2040259 * xyz.y + 1.0572252 * xyz.z};
}

template<typename T>
T _xyz2cielab_f(const T &t)
{
    static constexpr double delta = 6.0 / 29.0;
    static constexpr double f_tn = 4.0 / 29.0;
    static constexpr double delta_div = 1 / (delta * delta);
    static constexpr double delta3 = delta * delta * delta;

    return t - delta3 > spec::EPSILON ? cbrt(t) : fma(t / 3.0, T(delta_div), T(f_tn));//std::fma(t, delta_div, f_tn);
}

template<typename T>
base_vec3<T> _xyz2cielab(const base_vec3<T> &v)
{
    const T f_x = _xyz2cielab_f<T>(v.x / XYZ_TO_CIELAB_XYZN[0]);
    const T f_y = _xyz2cielab_f<T>(v.y / XYZ_TO_CIELAB_XYZN[1]);
    const T f_z = _xyz2cielab_f<T>(v.z / XYZ_TO_CIELAB_XYZN[2]);

    return base_vec3<T> {
        f_y * 116.0 - 16.0,
        (f_x - f_y) * 500.0,
        (f_y - f_z) * 200.0
    };
}


//wavelenghts must be sampled to CURVES_WAVELENGHTS values
template<typename T>
base_vec3<T> _spectre2xyz(const std::vector<Float> &wavelenghts, const std::vector<T> &values)
{
    static T cieyint = T(util::get_cie_y_integral());

    base_vec3<T> xyz{};

    unsigned idx = 0u;
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        const T val_lv = values[i] * T(util::CIE_D6500.get_or_interpolate(wavelenghts[i]));
        
        xyz.x += T(X_CURVE[idx]) * val_lv;
        xyz.y += T(Y_CURVE[idx]) * val_lv;
        xyz.z += T(Z_CURVE[idx]) * val_lv;
        idx += 1;
    }
    //xyz /= cieyint;
    return xyz / cieyint;
}

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

template<typename T>
T _get_cie_y_integral(const std::vector<Float> &wavelenghts, const std::vector<T> &values)
{
    T val = T(0.0);
    for(unsigned i = 0; i < wavelenghts.size(); ++i) {
        val += T(util::_interp<Y_CURVE>(wavelenghts[i])) * values[i];
    }
    return val;
}

template<typename T>
std::vector<T> _real_fourier_moments_of(const std::vector<T> &phases, const std::vector<T> &values, int n)
{
    static const std::complex<T> I_VAL{T(0.0), T(1.0)};
    const unsigned N = phases.size();
    int M = n - 1;
    std::vector<T> moments(M + 1);
    for(int i = 0; i <= M; ++i) {
        std::complex<T> val = 0.0;
        for(unsigned j = 0; j < N; ++j) {
            val += values[j] * std::exp(-I_VAL * T(i) * phases[j]);
        }
        moments[i] = std::real(val) / T(N);
    }
    return moments;
}

template<typename T>
std::vector<T> _mese(const std::vector<Float> &phases, const T *gamma_arr, int M)
{
    static const std::complex<T> I_VAL{T(0.0), T(1.0)};

    std::vector<T> gamma(M + 1);
    std::copy(gamma_arr, gamma_arr + M + 1, gamma.data());
//    gamma[0] -= gamma[0] * T(0.15);

    std::vector<T> res(phases.size());
    std::vector<T> e0(M + 1, T(0.0));
    e0[0] = T(1.0);

    std::vector<T> data(2 * M + 1); //Matrix values
    data[M] = T(math::INV_TWO_PI) * gamma[0];
    for(int i = 1; i <= M; ++i) {
        data[M + i] = T(math::INV_TWO_PI) * gamma[i];
        data[M - i] = T(math::INV_TWO_PI) * gamma[i]; 
    }

    std::vector<T> q = math::levinson<T>(data, e0);

    //if(q[0] < T(0.0)) std::cout << "neg q" << std::endl;

    for(unsigned k = 0; k < phases.size(); ++k) {
        std::complex<T> t = T(0.0);
        for(int i = 0; i <= M; ++i) t += T(math::INV_TWO_PI) * q[i] * exp(-I_VAL * T(i) * T(phases[k])); 


        T div = std::fabs(t);
        div *= div;

        res[k] = (T(math::INV_TWO_PI) * q[0]) / div;
    }
    return res;
}

void solve_for_rgb(const vec3 &target_rgb, Float power, std::vector<double> &x, const std::vector<Float> &wavelenghts, const std::vector<Float> &values);

std::vector<double> adjust_and_compute_moments(const vec3 &target_rgb, Float power, const std::vector<Float> &wavelenghts, const std::vector<Float> &values);


#endif