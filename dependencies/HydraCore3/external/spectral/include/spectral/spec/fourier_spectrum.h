#ifndef INCLUDE_SPECTRAL_SPEC_FOURIER_SPECTRUM_H
#define INCLUDE_SPECTRAL_SPEC_FOURIER_SPECTRUM_H
#include <spectral/spec/spectrum.h>
#include <spectral/spec/spectral_image.h>
#include <spectral/internal/math/fourier.h>
#include <vector>

namespace spec {

    class FourierReflectanceSpectrum : public ISpectrum 
    {
    public:
        INJECT_REFL(FourierReflectanceSpectrum);

        FourierReflectanceSpectrum()
            : coef{} {}

        FourierReflectanceSpectrum(const std::vector<Float> &coef)
            : coef(coef) {}

        FourierReflectanceSpectrum(std::vector<Float> &&coef)
            : coef(std::move(coef)) {}

        FourierReflectanceSpectrum(const FourierReflectanceSpectrum &other) = default;

        FourierReflectanceSpectrum(FourierReflectanceSpectrum &&other)
            : coef(std::move(other.coef)) {}

        FourierReflectanceSpectrum &operator=(const FourierReflectanceSpectrum &other) = default;

        FourierReflectanceSpectrum &operator=(FourierReflectanceSpectrum &&other)
        {
            coef = std::move(other.coef);
            return *this;
        }

        Float &operator[](unsigned i)
        {
            return coef[i];
        }

        Float operator[](unsigned i) const
        {
            return coef[i];
        }

        const std::vector<Float> &get() const
        {
            return coef;
        }

        void set(const std::vector<Float> &c)
        {
            coef = c;
        }

        Float get_or_interpolate(Float w) const override;


    private:
        std::vector<Float> coef;
        mutable std::vector<Complex> lagrange_m{};
    };


    extern template class SpectralImage<FourierReflectanceSpectrum>;

    using FourierReflectanceSpectralImage = SpectralImage<FourierReflectanceSpectrum>;

    class FourierEmissionSpectrum : public ISpectrum 
    {
    public:
        INJECT_REFL(FourierReflectanceSpectrum);

        FourierEmissionSpectrum()
            : coef{} {}

        FourierEmissionSpectrum(const std::vector<Float> &coef)
            : coef(coef) {}

        FourierEmissionSpectrum(std::vector<Float> &&coef)
            : coef(std::move(coef)) {}

        FourierEmissionSpectrum(const FourierEmissionSpectrum &other) = default;

        FourierEmissionSpectrum(FourierEmissionSpectrum &&other)
            : coef(std::move(other.coef)) {}

        FourierEmissionSpectrum &operator=(const FourierEmissionSpectrum &other) = default;

        FourierEmissionSpectrum &operator=(FourierEmissionSpectrum &&other)
        {
            coef = std::move(other.coef);
            return *this;
        }

        Float &operator[](unsigned i)
        {
            return coef[i];
        }

        Float operator[](unsigned i) const
        {
            return coef[i];
        }

        const std::vector<Float> &get() const
        {
            return coef;
        }

        void set(const std::vector<Float> &c)
        {
            coef = c;
        }

        Float get_or_interpolate(Float w) const override;


    private:
        std::vector<Float> coef;
        mutable std::vector<Float> q_vector{};
    };


    extern template class SpectralImage<FourierEmissionSpectrum>;

    using FourierEmissionSpectralImage = SpectralImage<FourierEmissionSpectrum>;
/*
    class LFourierSpectrum : public ISpectrum 
    {
    public:
        INJECT_REFL(LFourierSpectrum);

        LFourierSpectrum()
            : coef{} {}

        LFourierSpectrum(const std::vector<Complex> &coef)
            : coef(coef) {}

        LFourierSpectrum(std::vector<Complex> &&coef)
            : coef(std::move(coef)) {}

        LFourierSpectrum(const LFourierSpectrum &other) = default;

        LFourierSpectrum(LFourierSpectrum &&other)
            : coef(std::move(other.coef)) {}

        LFourierSpectrum &operator=(const LFourierSpectrum &other) = default;

        LFourierSpectrum &operator=(LFourierSpectrum &&other)
        {
            coef = std::move(other.coef);
            return *this;
        }

        Complex &operator[](unsigned i)
        {
            return coef[i];
        }

        Complex operator[](unsigned i) const
        {
            return coef[i];
        }

        const std::vector<Complex> &get() const
        {
            return coef;
        }

        void set(const std::vector<Complex> &c)
        {
            coef = c;
        }

        Float get_or_interpolate(Float w) const override;


    private:
        std::vector<Complex> coef;
    };


    extern template class SpectralImage<LFourierSpectrum>;

    using LFourierSpectralImage = SpectralImage<LFourierSpectrum>;
*/
}
#endif