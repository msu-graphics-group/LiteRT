#ifndef INCLUDE_SPECTRAL_IMAGEUTIL_BASE_IMAGE_H
#define INCLUDE_SPECTRAL_IMAGEUTIL_BASE_IMAGE_H
#include <string>
#include <stdexcept>
#include <memory>
#include <algorithm>

namespace spec {

    template <typename PixelType>
    class BaseImage
    {
    private:
        static void default_deleter(PixelType *ptr)
        {
            delete[] ptr;
        }
    public:

        using DeleterType = void (*)(PixelType *);

        BaseImage()
            : BaseImage(nullptr, 0, 0) {}

        BaseImage(int w, int h)
            : BaseImage(new PixelType[w * h], w, h) {}

        BaseImage(int w, int h, const PixelType &p)
            : BaseImage(w, h)
        {
            std::fill(data.get(), data.get() + w * h, p);
        }    

        BaseImage(const BaseImage &image)
            : BaseImage(new PixelType[image.width * image.height], image.width, image.height)
        {
            std::copy(image.data.get(), image.data.get() + width * height, data.get());
        }

        BaseImage(BaseImage &&image)
            : data(std::move(image.data)), width(std::move(image.width)), height(std::move(image.height)) {}

        inline const PixelType *raw_data() const {
            return data.get();
        }

        inline PixelType *raw_data() {
            return data.get();
        }

        PixelType &at(int i, int j)
        {
            long pos = (i + j * width);
            if(pos < 0 || pos >= width * height) throw std::out_of_range("Requested pixel is out of range");
            return data[pos];
        }

        const PixelType &at(int i, int j) const
        {
            long pos = (i + j * width);
            if(pos < 0 || pos >= width * height) throw std::out_of_range("Requested pixel is out of range");
            return data[pos];
        }

        BaseImage &operator=(BaseImage &&other) {
            if(this == &other) return *this;

            std::swap(data, other.data);
            width = std::move(other.width);
            height = std::move(other.height);
            return *this;
        }

        inline int get_width() const { return width; }
        inline int get_height() const { return height; }

    protected:

        BaseImage(PixelType *ptr, int width, int height, DeleterType deleter = default_deleter)
            : data(ptr, deleter), width(width), height(height) {}


        std::unique_ptr<PixelType [], DeleterType> data;
        int width, height;
    };

}


#endif 
