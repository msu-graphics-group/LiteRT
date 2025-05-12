#include <upsample/fourier.h>
#include <upsample/functional/fourier.h>
#include <internal/common/util.h>
#include <fstream>

namespace spec {

    namespace {

        FourierLUT load_from_file(const std::string &path)
        {
            std::ifstream file{path};
            return FourierLUT::load_from(file);
        }

    }

    FourierUpsampler::FourierUpsampler(bool emiss)
        : lut{load_from_file("resources/f_emission_lut.efif")}, emiss{emiss}
    {

    }

    ISpectrum::ptr FourierUpsampler::upsample_pixel(const Pixel &src) const
    {
        if(emiss) {
            return ISpectrum::ptr(new FourierEmissionSpectrum(upsample::fourier_emiss_int(src, lut)));
        }
        else {
            return {}; //TODO
        }
    }

    ISpectralImage::ptr FourierUpsampler::upsample(const Image &sourceImage) const
    {   
        if(emiss) {
            FourierEmissionSpectralImage *dest = new FourierEmissionSpectralImage(sourceImage.get_width(), sourceImage.get_height());
           
            const long img_size = sourceImage.get_width() * sourceImage.get_height();
            init_progress_bar(img_size, 1000);

            const Pixel *ptr = sourceImage.raw_data();
            FourierEmissionSpectrum *s_ptr = dest->raw_data();
            for(int i = 0; i < img_size; ++i) {
                s_ptr[i] = upsample::fourier_emiss_int(ptr[i], luts);
                print_progress(i + 1);
            }

            finish_progress_bar();
            return ISpectralImage::ptr(dest);
        }
        else {
            return {}; //TODO
        }
    }

}