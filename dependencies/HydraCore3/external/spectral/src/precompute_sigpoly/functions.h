#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <optional>
#include <utility>
#include <internal/math/math.h>
#include <internal/common/constants.h>
#include <spec/spectral_util.h>


using spec::math::base_vec3;
using spec::math::vec3;
using spec::math::vec3d;
using spec::Float;

extern bool enable_logging;

constexpr double XYZ_TO_CIELAB_XYZN[3]{95.0489f, 100.0f, 108.8840f};

template<typename T>
inline T _sigmoid(const T &x)
{
    return fma(T(0.5), x / sqrt(fma(x, x, T(1))), T(0.5));
}

template<typename T>
inline T _sigmoid_polynomial(const T &x, const T coef[3])
{
    return _sigmoid<T>(fma(fma(coef[0], x, coef[1]), x, coef[2]));
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

template<typename T>
base_vec3<T> _sigpoly2xyz(const T *x)
{
    base_vec3<T> xyz{};
    //const BasicSpectrum &d65 = get_D6500();
    unsigned idx = 0u;
    for(int lambda = spec::WAVELENGHTS_START; lambda <= spec::WAVELENGHTS_END; lambda += spec::WAVELENGHTS_STEP) {
        const T val_lv = _sigmoid_polynomial<T>(T(lambda), x) * static_cast<double>(spec::util::CIE_D6500.get_or_interpolate(lambda));
        
        xyz.x += val_lv * static_cast<double>(spec::X_CURVE[idx]);
        xyz.y += val_lv * static_cast<double>(spec::Y_CURVE[idx]);
        xyz.z += val_lv * static_cast<double>(spec::Z_CURVE[idx]);
        idx += 1;
    }
    const Float cieyint = spec::util::get_cie_y_integral();

    xyz.x /= cieyint;
    xyz.y /= cieyint;
    xyz.z /= cieyint;
    return xyz;
}

vec3d solve_for_rgb(const vec3 &rgb, const vec3d &init);

void solve_for_rgb_d(const vec3 &rgb, vec3d &x);

std::optional<std::pair<vec3, vec3>> find_stable(const vec3d &init, vec3d &solution, int div, int mdiv, spec::Float epsilon);



#endif