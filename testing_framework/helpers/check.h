#pragma once
#include <testing_framework/helpers/image.h>
#include <testing_framework/helpers/format.h>

#include "../../utils/image_metrics.h"

namespace testing
{
    template <typename T>
    float measure_psnr(Image<T> &ref, Image<T> &other)
    {
        return image_metrics::PSNR(ref, other);
    }

    void log_metric(
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold,
        float value,
        std::string_view metric_name);

    template <typename T>
    void check_psnr(
        Image<T> &ref,
        Image<T> &other,
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold)
    {
        float psnr = measure_psnr(ref, other);
        log_metric(ref_desc, other_desc, threshold, psnr, "PSNR");
    }

    /*
        Saved reference if stored in file
        <saved-references>/<test-name>/<name>.png
    */
    template <typename T>
    void saved_reference_check_psnr(
        Image<T> &image,
        const std::string &desc,
        const std::string &name,
        float threshold)
    {
        std::string path = saved_references_directory() + "/" + std::string(get_test_name()) + "/" + name + ".png";
        if (should_rewrite_saved_reference("'" + desc + " saved reference'", path))
        {
            save_image_by_path(image, path);
        }
        else
        {
            auto ref = load_image_by_path<T>(path, desc + " saved reference", !ignore_saved_references());
            if (!ref) // failed to load
            {
                return;
            }
            LiteMath::float2 image_size(image.width(), image.height());
            LiteMath::float2 ref_size(ref->width(), ref->height());
            if (image_size.x != ref_size.x || image_size.y != ref_size.y)
            {
                log(ignore_saved_references() ? bar_warning : bar_error)
                    << "Size of " << std::fixed << std::setprecision(0)
                    << foreground(highlight_color_3) << "image " << image_size << default_color
                    << " and size of "
                    << foreground(highlight_color_1) << "saved reference " << ref_size << default_color
                    << " are different"
                    << std::endl;
                if (ignore_saved_references())
                {
                    return;
                }
                else
                {
                    skip();
                }
            }
            check_psnr(*ref, image, desc + " saved reference", desc, threshold);
        }
    }

}