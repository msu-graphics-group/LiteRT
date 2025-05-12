#include <imageutil/image.h>
#include <internal/common/format.h>
#include <utility>
#include <stdexcept>
#include <stb_image.h>
#include <stb_image_write.h>
#include <filesystem>

namespace fs = std::filesystem;

namespace spec {

    template class BaseImage<Pixel>;

    namespace {
        void free_image(Pixel *ptr)
        {
            stbi_image_free(ptr);
        }
    }

    Image::Image(const std::string &path)
        : BaseImage<Pixel>()
    {
        int n;
        
        unsigned char *ptr = stbi_load(path.c_str(), &width, &height, &n, sizeof(Pixel));
        if(ptr == nullptr) {
            stbi_failure_reason();
            throw std::runtime_error(format("Error reading image at %s", path.c_str()));
        }
        data = std::unique_ptr<Pixel[], BaseImage<Pixel>::DeleterType>(reinterpret_cast<Pixel *>(ptr), free_image);
    }

    bool Image::save(const std::string &path, const std::string &format) const
    {
        std::string fmt = format;
        if(fmt == "") {
            fmt = fs::path(path).extension().string().substr(1);
        }

        if(fmt == "png") {
            return stbi_write_png(path.c_str(), width, height, sizeof(Pixel), reinterpret_cast<const unsigned char *>(data.get()), 0) != 0;
        }
        if(fmt == "jpg") {
            return stbi_write_jpg(path.c_str(), width, height, sizeof(Pixel), reinterpret_cast<const unsigned char *>(data.get()), 90) != 0;
        }
        return false;
    }

}