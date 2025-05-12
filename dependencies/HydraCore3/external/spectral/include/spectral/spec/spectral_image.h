#ifndef INCLUDE_SPECTRAL_SPEC_SPECTRAL_IMAGE_H
#define INCLUDE_SPECTRAL_SPEC_SPECTRAL_IMAGE_H
#include <spectral/spec/spectrum.h>
#include <vector>

namespace spec {
   
    template <typename SpectrumType>
    class SpectralImage : public ISpectralImage
    {
    public:
        INJECT_REFL(SpectralImage<SpectrumType>);

        SpectralImage()
            : ISpectralImage(0, 0), data() {}

        SpectralImage(int w, int h)
            : ISpectralImage(w, h), data(w * h) {}

        SpectralImage(int w, int h, const SpectrumType &p)
            : ISpectralImage(w, h), data(w * h, p) {}    

        SpectralImage(const SpectralImage &image) = default;

        SpectralImage(SpectralImage &&image)
            : ISpectralImage(image.width, image.height), data(std::move(image.data)) {}

        inline const SpectrumType *raw_data() const {
            return data.data();
        }

        inline SpectrumType *raw_data() {
            return data.data();
        }

        SpectrumType &at(int i, int j) override
        {
            long pos = (i + j * width);
            if(pos < 0 || pos >= width * height) throw std::out_of_range("Requested pixel is out of range");
            return data[pos];
        }

        const SpectrumType &at(int i, int j) const override
        {
            long pos = (i + j * width);
            if(pos < 0 || pos >= width * height) throw std::out_of_range("Requested pixel is out of range");
            return data[pos];
        }

        SpectralImage &operator=(SpectralImage &&other) {
            if(this == &other) return *this;

            data = std::move(other.data);
            width = std::move(other.width);
            height = std::move(other.height);
            return *this;
        }



    protected:
        std::vector<SpectrumType> data;

    };

}


#endif 
