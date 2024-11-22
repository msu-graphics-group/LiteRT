#include <testing_framework/core/environment.h>
#include <testing_framework/core/execution.h>
#include <testing_framework/core/cli.h>
#include <testing_framework/core/logging.h>

namespace testing
{

    void skip()
    {
        skip_current_test();
    }

    void add_check_result(bool passed)
    {
        add_current_test_check_result(passed);
    }

    bool should_rewrite_saved_reference(std::string_view what, std::string_view path)
    {
        if (rewrite())
        {

            log(bar_warning) << "Are you sure you want to rewrite " << foreground(highlight_color_1) << what << default_color
                << " in file " << foreground(highlight_color_2) << "'" << path << "'" << default_color << "?" << std::endl;

            bool res = false;
            if (yes_or_no_dialogue(res))
            {
                return res;
            }
            else
            {
                skip_current_test();
            }
        }
        return false;
    }

    bool get_flag(std::string_view name)
    {
        return get_test_flag(name);
    }

    std::string_view get_param(std::string_view name, const std::type_info*parsing_type)
    {
        return get_test_param(name, parsing_type);
    }

}