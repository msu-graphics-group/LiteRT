#pragma once
#include <string>
#include <optional>
#include <testing_framework/core/logging.h>
#include <testing_framework/core/environment.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/helpers/files.h>
#include <testing_framework/helpers/format.h>
#include "LiteMath/Image2d.h"

namespace testing
{
    
    using DefaultImageType = uint32_t;

    template<typename T>
    using Image = LiteImage::Image2D<T>;
    
    template<typename T = DefaultImageType>
    Image<T> create_image(size_t width = image_width(), size_t height = image_height())
    {
        Image<T> image(width, height);
        return image;
    }

    template<typename T>
    void save_image_by_path(const Image<T>&image, const std::string&path, std::string_view name = "image")
    {
        prepare_directoty_for_saving(path);
        log(bar_info) << "Saving " << name << " to "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        LiteImage::SaveImage<T>(path.c_str(), image);
    }

    template<typename T>
    void save_image(const Image<T>&image, const std::string&file_name, std::string_view name = "image")
    {
        std::string path = saves_directory() + "/" + std::string(get_test_name()) + "/" + file_name + ".png";
        save_image_by_path(image, path, name);
    }

    template<typename T = DefaultImageType>
    std::optional<Image<T>> load_image_by_path(
        const std::string&path,
        std::string_view name  = "image",
        bool treat_failure_as_error = true,
        source_location loc = source_location::current()
    )
    {
        log(bar_info) << "Loading " << name <<  " from "
                          << foreground(highlight_color_2) << path << default_color
                          << std::endl;
        if (!assert_file_existance(path, treat_failure_as_error, loc))
        {
            return std::nullopt;
        }
        Image<T> image = LiteImage::LoadImage<T>(path.c_str());
        if (image.data())
        {
            return std::move(image);
        }
        log(treat_failure_as_error ? bar_error : bar_warning) << "Failed to load image from "
            << foreground(highlight_color_2) << path << default_color
            << " at " << loc
            << std::endl;
        if (treat_failure_as_error)
        {
            skip();
            return std::nullopt;
        }
        else
        {
            return std::nullopt;
        }
    }

}