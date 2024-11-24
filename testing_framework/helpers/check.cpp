#include <testing_framework/helpers/check.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/core/environment.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/helpers/render.h>
#include <iomanip>
#include <filesystem>

#include "../../utils/image_metrics.h"

namespace testing
{

    float measure_psnr(Image &ref, Image &other)
    {
        return image_metrics::PSNR(ref, other);
    }

    void check_psnr(
        Image &ref,
        Image &other,
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold)
    {

        auto pass = bar_passed;
        auto fail = bar_failed;
        pass.align = 0;
        fail.align = 0;

        float psnr = measure_psnr(ref, other);

        size_t colors_offset = 7 * 2 + 4 * 2;

        bool ok = psnr >= threshold;
        add_check_result(ok);
        log(ok ? pass : fail) << begin_aligned(MESSAGE_WIDTH + (colors_are_enabled() ? colors_offset : 0), -1, 0) << "PSNR "
                              << foreground(highlight_color_3) << "'" << other_desc << "'" << default_color
                              << " and "
                              << foreground(highlight_color_1) << "'" << ref_desc << "'" << default_color
                              << end_aligned
                              << std::fixed << std::setprecision(1)
                              << foreground(highlight_color_3) << psnr << default_color
                              << " " << (ok ? ">" : "<") << " "
                              << foreground(highlight_color_1) << threshold << default_color
                              << std::endl;
    }

    void saved_reference_check_psnr(
        Image &image,
        const std::string& desc,
        const std::string &name,
        float threshold)
    {
        std::string path = saved_references_directory() + "/" + std::string(get_test_name()) + "/" + name + ".png";
        if (should_rewrite_saved_reference("'" + desc + " saved reference'" , path))
        {
            save_image_by_path(image, path);
        }
        else
        {
            log(bar_info) << "Loading saved reference from "
                          << foreground(highlight_color_2) << path << default_color
                          << std::endl;
            if (!std::filesystem::exists(path)) // no file
            {
                log(ignore_saved_references() ? bar_warning : bar_error) << "File "
                                                                         << foreground(highlight_color_2) << path << default_color
                                                                         << " does not exist" << std::endl;
                if (ignore_saved_references())
                {
                    return;
                }
                else
                {
                    skip();
                }
            }
            auto ref = LiteImage::LoadImage<uint32_t>(path.c_str());
            if (ref.data() == nullptr) // failed to read
            {
                log(ignore_saved_references() ? bar_warning : bar_error) << "Failed to read image from "
                                                                         << foreground(highlight_color_2) << path << default_color
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
            float2 image_size(image.width(), image.height());
            float2 ref_size(ref.width(), ref.height());
            if (ref.width() != image.width() || ref.height() != image.height())
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
            check_psnr(ref, image, desc + " saved reference", desc, threshold);
        }
    }

}