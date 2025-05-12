#ifndef INCLUDE_SPECTRAL_IMAGEUTIL_IMAGE_H
#define INCLUDE_SPECTRAL_IMAGEUTIL_IMAGE_H
#include <spectral/imageutil/pixel.h>
#include <spectral/imageutil/base_image.h>
#include <string>

namespace spec {

    extern template class BaseImage<Pixel>;

    class Image : public BaseImage<Pixel>
    {
    public:
        using BaseImage<Pixel>::BaseImage;

        explicit Image(const std::string &path);

        bool save(const std::string &path, const std::string &format = "") const;
    };

}

#endif 
