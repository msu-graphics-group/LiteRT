#ifndef INCLUDE_SPECTRAL_INTERNAL_MATH_MATH_H
#define INCLUDE_SPECTRAL_INTERNAL_MATH_MATH_H
#include <spectral/internal/math/math_fwd.h>
#include <spectral/internal/math/vec3.h>
#include <spectral/internal/math/mat3.h>

namespace spec {

    namespace math {

        constexpr Float PI = 3.14159265358979323846f;
        constexpr Float INV_PI = 1.0f / PI;
        constexpr Float TWO_PI = 2.0f * PI;
        constexpr Float INV_TWO_PI = 1.0f / TWO_PI;

        inline Float interpolate(Float point, Float a, Float b, Float f_a, Float f_b)
        {
            return f_a + (f_b - f_a) * (point - a) / (b - a);
        }

        Float clamp(Float x, Float a, Float b);
        vec3 clamp(const vec3 &x, Float a, Float b);

        Float sigmoid_polynomial(Float x, const Float coef[3]);

        Float smoothstep(Float x);
        
        inline Float smoothstep2(Float x)
        {
            return smoothstep(smoothstep(x));
        }

        Float inv_smoothstep(Float x);

        inline Float inv_smoothstep2(Float x)
        {
            return inv_smoothstep(inv_smoothstep(x));
        }
    }

    using math::mat3;
    using math::vec3;
    using math::vec3d;
    using math::vec3ui;
    using math::vec3i;

}

#endif