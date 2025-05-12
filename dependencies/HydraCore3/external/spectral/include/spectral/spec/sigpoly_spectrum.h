#ifndef INCLUDE_SPECTRAL_SPEC_SIGPOLY_SPECTRUM_H
#define INCLUDE_SPECTRAL_SPEC_SIGPOLY_SPECTRUM_H
#include <spectral/spec/spectral_image.h>
#include <stdexcept>

namespace spec {
    
    class SigPolySpectrum : public ISpectrum
    {
    public:

        INJECT_REFL(SigPolySpectrum);

        SigPolySpectrum() noexcept(true)
            : ISpectrum(), coef{0, 0, 0} {}

        SigPolySpectrum(const SigPolySpectrum &s) = default;

        SigPolySpectrum(const vec3 &c) noexcept(true)
            : ISpectrum(), coef{c} {}
        
        Float &operator[](int i) noexcept(true)
        {
            return coef[i];
        }

        Float operator[](int i) const noexcept(true)
        {
            return coef[i];
        }

        const vec3 &get() const
        {
            return coef;
        }

        void set(const vec3 &c)
        {
            coef = c;
        }

        Float get_or_interpolate(Float w) const override;

        SigPolySpectrum &operator=(const SigPolySpectrum &other) = default;

    private:
        vec3 coef;
    };

    extern template class SpectralImage<SigPolySpectrum>;

    using SigPolySpectralImage = SpectralImage<SigPolySpectrum>;
    
}

#endif
