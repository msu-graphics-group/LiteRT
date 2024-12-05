#pragma once
#include <testing_framework/helpers/image.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/helpers/source_location.h>
#include "utils/image_metrics.h"

namespace testing
{

    /*
        Checks if a > b
    */
    void check_greater(
        float a,
        float b,
        std::string_view a_desc,
        std::string_view b_desc,
        bool eq = false,
        source_location loc = source_location::current()
    );
    
    /*
        Checks if a == b
    */
    void check_equal(
        float a,
        float b,
        std::string_view a_desc,
        std::string_view b_desc,
        float threshold = 1e-12f,
        source_location loc = source_location::current()
    );    

    /*
        Checks if a > b
    */
    void check_less(
        float a,
        float b,
        std::string_view a_desc,
        std::string_view b_desc,
        bool eq = false,
        source_location loc = source_location::current()
    );

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
        std::string_view metric_name,
        source_location loc
    );

    template <typename T>
    float check_psnr(
        Image<T> &ref,
        Image<T> &other,
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold,
        source_location loc = source_location::current()
    )
    {
        float psnr = measure_psnr(ref, other);
        log_metric(ref_desc, other_desc, threshold, psnr, "PSNR", loc);
        return psnr;
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
        float threshold,
        source_location loc = source_location::current()
    )
    {
        std::string path = saved_references_directory() + "/" + std::string(get_test_name()) + "/" + name + ".png";
        if (should_rewrite_saved_reference("'" + desc + " saved reference'", path))
        {
            save_image_by_path(image, path);
        }
        else
        {
            auto ref = load_image_by_path<T>(path, desc + " saved reference", !ignore_saved_references(), loc);
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
                    << " are different at "
                    << loc
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