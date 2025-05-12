#ifndef INCLUDE_SPECTRAL_INTERNAL_MATH_MAT3_H
#define INCLUDE_SPECTRAL_INTERNAL_MATH_MAT3_H
#include <spectral/internal/math/math_fwd.h>
#include <spectral/internal/math/vec3.h>

namespace spec {

    namespace math {

        union mat3
        {
            Float data[9];
            struct {
                Float e11, e12, e13;
                Float e21, e22, e23;
                Float e31, e32, e33;
            };

            mat3() : data{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {}
            mat3(const mat3 &m) = default;
            mat3(Float a11, Float a12, Float a13, 
                 Float a21, Float a22, Float a23,
                 Float a31, Float a32, Float a33) :data{a11, a12, a13, a21, a22, a23, a31, a32, a33} {}

            mat3 &operator=(const mat3 &m) = default;

            mat3 operator-() const;

            mat3 &operator+=(const mat3 &v);
            mat3 &operator-=(const mat3 &v);

            mat3 &operator*=(Float f);
            mat3 &operator/=(Float f);


            mat3 operator+(const mat3 &m) const
            {
                mat3 c(*this);
                c += m;
                return c;
            }

            mat3 operator-(const mat3 &m) const
            {
                mat3 c(*this);
                c -= m;
                return c;
            }

            mat3 operator*(Float f) const
            {
                mat3 c(*this);
                c *= f;
                return c;
            }

            mat3 operator/(Float f) const
            {
                mat3 c(*this);
                c /= f;
                return c;
            }

            inline Float &operator[](int i)
            {
                return data[i];
            }

            inline Float operator[](int i) const
            {
                return data[i];
            }

        };

        vec3 operator*(const mat3 &m, const vec3 &v);

        vec3 operator*(const vec3 &v, const mat3 &m);

        Float determinant(const mat3 &m);
        mat3 inverse(const mat3 &m);

    }
}
#endif