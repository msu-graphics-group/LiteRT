#ifndef INCLUDE_SPECTRAL_SPEC_SPECTRUM_H
#define INCLUDE_SPECTRAL_SPEC_SPECTRUM_H
#include <spectral/internal/math/math.h>
#include <spectral/internal/common/refl.h>
#include <memory>

namespace spec {

    class ISpectrum
    {
    public:
        INJECT_ABSTRACT_REFL(ISpectrum);

        virtual Float get_or_interpolate(Float w) const = 0;
        Float operator()(Float w) const { return get_or_interpolate(w); }
        virtual ~ISpectrum() = default;

        static const ISpectrum &none();

        using ptr = std::unique_ptr<ISpectrum>;
        using uptr = std::unique_ptr<ISpectrum>;
        using sptr = std::shared_ptr<ISpectrum>;

        using csptr = std::shared_ptr<ISpectrum const>;
    };

    class ISpectralImage {
    public:
        INJECT_ABSTRACT_REFL(ISpectralImage);
        
        ISpectralImage(int width, int height) 
            : width(width), height(height) {}

        virtual ISpectrum &at(int i, int j) = 0;
        virtual const ISpectrum &at(int i, int j) const = 0;

        inline int get_width() const { return width; }
        inline int get_height() const { return height; }

        virtual ~ISpectralImage() = default;

        using ptr = std::unique_ptr<ISpectralImage>;
        using uptr = std::unique_ptr<ISpectralImage>;
        using sptr = std::shared_ptr<ISpectralImage>;

    protected:
        int width, height;
    };

}

#endif