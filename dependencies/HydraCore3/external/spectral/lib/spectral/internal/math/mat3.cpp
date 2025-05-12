#include <internal/math/mat3.h>
 
namespace spec {
    namespace math {
        
        mat3 mat3::operator-() const
        {
            return {-data[0], -data[1], -data[2],
                    -data[3], -data[4], -data[5],
                    -data[6], -data[7], -data[8]};
        }

        mat3 &mat3::operator+=(const mat3 &m)
        {
            data[0] += m.data[0];
            data[1] += m.data[1];
            data[2] += m.data[2];
            data[3] += m.data[3];
            data[4] += m.data[4];
            data[5] += m.data[5];
            data[6] += m.data[6];
            data[7] += m.data[7];
            data[8] += m.data[8];
            return *this;
        }

        mat3 &mat3::operator-=(const mat3 &m) {
            data[0] -= m.data[0];
            data[1] -= m.data[1];
            data[2] -= m.data[2];
            data[3] -= m.data[3];
            data[4] -= m.data[4];
            data[5] -= m.data[5];
            data[6] -= m.data[6];
            data[7] -= m.data[7];
            data[8] -= m.data[8];
            return *this;
        }

        mat3 &mat3::operator*=(Float f)
        {
            data[0] *= f;
            data[1] *= f;
            data[2] *= f;
            data[3] *= f;
            data[4] *= f;
            data[5] *= f;
            data[6] *= f;
            data[7] *= f;
            data[8] *= f;
            return *this;
        }

        mat3 &mat3::operator/=(Float f)
        {
            const Float invf = 1.0f / f;
            data[0] *= invf;
            data[1] *= invf;
            data[2] *= invf;
            data[3] *= invf;
            data[4] *= invf;
            data[5] *= invf;
            data[6] *= invf;
            data[7] *= invf;
            data[8] *= invf;
            return *this;
        } 


        vec3 operator*(const mat3 &m, const vec3 &v)
        {
            return {m.e11 * v.x + m.e12 * v.y + m.e13 * v.z,
                    m.e21 * v.x + m.e22 * v.y + m.e23 * v.z,
                    m.e31 * v.x + m.e32 * v.y + m.e33 * v.z};
        }

        vec3 operator*(const vec3 &v, const mat3 &m)
        {
            return {v.x * m.e11 + v.y * m.e21 + v.z * m.e31,
                    v.x * m.e12 + v.y * m.e22 + v.z * m.e32,
                    v.x * m.e13 + v.y * m.e23 + v.z * m.e33};
        }

        Float determinant(const mat3 &m)
        {
            return m.e11 * m.e22 * m.e33
                 + m.e21 * m.e13 * m.e32
                 + m.e12 * m.e23 * m.e31
                 - m.e13 * m.e22 * m.e31
                 - m.e11 * m.e23 * m.e32
                 - m.e12 * m.e21 * m.e33;
        }

        mat3 inverse(const mat3 &m)
        {
            Float det = determinant(m);

            mat3 adjoint{
                m.e22 + m.e33 - m.e23 * m.e32, m.e13 * m.e32 - m.e12 * m.e33, m.e12 * m.e23 - m.e13 * m.e22,
                m.e23 * m.e31 - m.e21 * m.e33, m.e11 * m.e33 - m.e13 * m.e31, m.e13 * m.e21 - m.e11 * m.e23,
                m.e21 * m.e32 - m.e22 * m.e31, m.e12 * m.e31 - m.e11 * m.e32, m.e11 * m.e22 - m.e12 * m.e21
            };
            return adjoint / det;
        }
    }
}