#include <upsample/glassner_naive.h> 
#include <upsample/functional/glassner.h>
#include <internal/common/util.h>

namespace spec {


    ISpectrum::ptr GlassnerUpsampler::upsample_pixel(const Pixel &pixel) const
    {
        return ISpectrum::ptr(new BasicSpectrum(upsample::glassner(pixel.to_vec3())));
    }

    ISpectralImage::ptr GlassnerUpsampler::upsample(const Image &sourceImage) const
    {
        BasicSpectralImage *dest = new BasicSpectralImage(sourceImage.get_width(), sourceImage.get_height());
        const long img_size = sourceImage.get_width() * sourceImage.get_height();
        init_progress_bar(img_size, 1000);

        for(unsigned i = 0; i < upsample::GLASSNER_SPECTRUM_SIZE; ++i) {
            dest->add_wavelenght(upsample::GLASSNER_WAVELENGHTS[i]);
        }

        const Pixel *ptr = sourceImage.raw_data();
        BasicSpectrum *s_ptr = dest->raw_data();
        for(int i = 0; i < img_size; ++i) {
            s_ptr[i] = upsample::glassner(ptr[i].to_vec3());
            print_progress(i + 1);
        }
        finish_progress_bar();
        return ISpectralImage::ptr(dest);
    }
}