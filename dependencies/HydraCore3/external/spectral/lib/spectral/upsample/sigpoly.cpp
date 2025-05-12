#include <upsample/sigpoly.h>
#include <upsample/functional/sigpoly.h>
#include <internal/common/util.h>
#include <fstream>

namespace spec {

    namespace {

        SigpolyLUT load_from_file(const std::string &path)
        {
            std::ifstream file{path};
            return SigpolyLUT::load_from(file);
        }

    }

    SigPolyUpsampler::SigPolyUpsampler()
        : luts{load_from_file("resources/sp_lut0.slf"), load_from_file("resources/sp_lut1.slf"), load_from_file("resources/sp_lut2.slf")}
    {

    }

    ISpectrum::ptr SigPolyUpsampler::upsample_pixel(const Pixel &src) const
    {
        return ISpectrum::ptr(new SigPolySpectrum(upsample::sigpoly_int(src, luts)));
    }

    ISpectralImage::ptr SigPolyUpsampler::upsample(const Image &sourceImage) const
    {
        SigPolySpectralImage *dest = new SigPolySpectralImage(sourceImage.get_width(), sourceImage.get_height());
       
        const long img_size = sourceImage.get_width() * sourceImage.get_height();
        init_progress_bar(img_size, 1000);

        const Pixel *ptr = sourceImage.raw_data();
        SigPolySpectrum *s_ptr = dest->raw_data();
        for(int i = 0; i < img_size; ++i) {
            s_ptr[i] = upsample::sigpoly_int(ptr[i], luts);
            print_progress(i + 1);
        }

        finish_progress_bar();
        return ISpectralImage::ptr(dest);
    }

}