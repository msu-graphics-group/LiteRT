#ifndef INCLUDE_SPECTRAL_INTERNAL_MATH_FOURIER_H
#define INCLUDE_SPECTRAL_INTERNAL_MATH_FOURIER_H
#include <spectral/internal/math/math.h>
#include <spectral/internal/common/constants.h>
#include <complex>
#include <vector>

namespace spec {
    using Complex = std::complex<Float>;

    namespace math {
        constexpr Complex I{0.0f, 1.0f};

        Float to_phase(Float wl, Float start = WAVELENGHTS_START, Float end = WAVELENGHTS_END);

        inline std::vector<Float> wl_to_phases(std::vector<Float> wavelenghts)
        {
            for(unsigned i = 0; i < wavelenghts.size(); ++i) {
                wavelenghts[i] = to_phase(wavelenghts[i]);
            }
            return wavelenghts;
        }

        std::vector<Float> real_fourier_moments_of(const std::vector<Float> &phases, const std::vector<Float> &values, int n);

        std::vector<Complex> fourier_moments_of(const std::vector<Float> &phases, const std::vector<Float> &values, int n);

        std::vector<Complex> precompute_mese_coeffs(const std::vector<Complex> &gamma);
        std::vector<Float> precompute_mese_coeffs(const std::vector<Float> &gamma);

        std::vector<Complex> lagrange_multipliers(const std::vector<Float> &moments);

        Float bounded_mese_l(Float phase, const std::vector<Complex> &lagrange_m);

        inline Float bounded_mese_m(Float phase, const std::vector<Float> &moments)
        {
            return bounded_mese_l(phase, lagrange_multipliers(moments));
        }

        Float mese_precomp(Float phase, const std::vector<Float> &moments);

        inline std::vector<Float> mese(std::vector<Float> phases, const std::vector<Float> &moments)
        {
            std::vector<Float> q = precompute_mese_coeffs(moments);

            for(unsigned k = 0; k < phases.size(); ++k) {
                phases[k] = mese_precomp(phases[k], q);
            }
            return phases;
        }

        inline Float mese(Float phase, const std::vector<Float> &moments)
        {
            return mese_precomp(phase, precompute_mese_coeffs(moments));
        }

    }

}


#endif