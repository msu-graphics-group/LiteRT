#include <testing_framework/helpers/check.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/core/environment.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/helpers/render.h>
#include <iomanip>
#include <filesystem>

#include "utils/image_metrics.h"

namespace testing
{

    void log_cmp(
        float a,
        float b,
        std::string_view a_desc,
        std::string_view b_desc,
        std::string_view s,
        std::string_view neg_s,
        bool ok,
        source_location loc)
    {
        auto pass = bar_passed;
        auto fail = bar_failed;
        pass.align = 0;
        fail.align = 0;

        size_t colors_offset = 7 * 2 + 4 * 2;

        add_check_result(ok);
        log(ok ? pass : fail) << begin_aligned(MESSAGE_WIDTH + (colors_are_enabled() ? colors_offset : 0), -1, 0)
                              << foreground(highlight_color_3) << "'" << a_desc << "'" << default_color
                              << " " << s << " "
                              << foreground(highlight_color_1) << "'" << b_desc << "'" << default_color
                              << end_aligned
                              << std::fixed << std::setprecision(1)
                              << foreground(highlight_color_3) << a << default_color
                              << " " << (ok ? s : neg_s) << " "
                              << foreground(highlight_color_1) << b << default_color
                              << (ok ? "" : " at ") << print_conditional(loc, !ok)
                              << std::endl;
    }

    void check_greater(float a, float b, std::string_view a_desc, std::string_view b_desc, bool eq, source_location loc)
    {
        std::string_view sign = eq ? ">=" : ">";
        std::string_view neg = eq ? "<" : "<=";
        bool ok = eq ? a >= b : a > b;
        log_cmp(a, b, a_desc, b_desc, sign, neg, ok, loc);
    }

    void check_equal(float a, float b, std::string_view a_desc, std::string_view b_desc, float threshold,source_location loc)
    {
        bool ok = std::abs(a - b) <= threshold;
        log_cmp(a, b, a_desc, b_desc, "=", "!=", ok, loc);
    } 

    void check_less(float a, float b, std::string_view a_desc, std::string_view b_desc, bool eq, source_location loc)
    {
        std::string_view sign = eq ? "<=" : "<";
        std::string_view neg = eq ? ">" : ">=";
        bool ok = eq ? a <= b : a < b;
        log_cmp(a, b, a_desc, b_desc, sign, neg, ok, loc);
    }


    void log_metric(
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold,
        float value,
        std::string_view metric_name,
        source_location loc)
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
                              << (ok ? "" : " at ") << print_conditional(loc, !ok)
                              << std::endl;
    }

}