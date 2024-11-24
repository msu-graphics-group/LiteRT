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

   void log_metric(
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold,
        float value,
        std::string_view metric_name)
    {
        auto pass = bar_passed;
        auto fail = bar_failed;
        pass.align = 0;
        fail.align = 0;

        size_t colors_offset = 7 * 2 + 4 * 2;

        bool ok = value >= threshold;
        add_check_result(ok);
        log(ok ? pass : fail) << begin_aligned(MESSAGE_WIDTH + (colors_are_enabled() ? colors_offset : 0), -1, 0) << metric_name << " "
                              << foreground(highlight_color_3) << "'" << other_desc << "'" << default_color
                              << " and "
                              << foreground(highlight_color_1) << "'" << ref_desc << "'" << default_color
                              << end_aligned
                              << std::fixed << std::setprecision(1)
                              << foreground(highlight_color_3) << value << default_color
                              << " " << (ok ? ">" : "<") << " "
                              << foreground(highlight_color_1) << threshold << default_color
                              << std::endl;
    }

}