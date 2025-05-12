#include <spec/fourier_spectrum.h>

namespace spec {

    Float FourierReflectanceSpectrum::get_or_interpolate(Float w) const
    {
        if(lagrange_m.empty()) {
            lagrange_m = math::lagrange_multipliers(coef);
        }

        return math::bounded_mese_l(math::to_phase(w), lagrange_m);
    }

    Float FourierEmissionSpectrum::get_or_interpolate(Float w) const
    {
        if(coef[0] == 0) return 0.0f;
        if(q_vector.empty()) {
            q_vector = math::precompute_mese_coeffs(coef);
        }

        return math::mese_precomp(math::to_phase(w), q_vector);
    }
/*
    Float LFourierSpectrum::get_or_interpolate(Float w) const
    {
        return math::bounded_mese_l(math::to_phase(w), coef);  
    }
*/

    template class SpectralImage<FourierReflectanceSpectrum>;
    template class SpectralImage<FourierEmissionSpectrum>;
   // template class SpectralImage<LFourierSpectrum>;

}