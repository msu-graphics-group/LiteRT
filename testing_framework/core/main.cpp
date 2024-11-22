#include <testing_framework/core/cli.h>
#include <testing_framework/core/test_options.h>

int main(int argc, char**argv)
{
    testing::add_test_flag(
        "ignore_saved_references",
        "",
        "--ignore-saved-references", 
        " - specifies, if invalid saved references should be reason for test skipping"
    );
    testing::add_test_param<int64_t>(
        "renderings_count",
        "",
        "--renderings",
        "1",
        " - now many renderings should be performed to meashure mean rendering time, defaulted to 1",
        testing::validate_is_positive_param        
    );
    testing::add_test_param<int64_t>(
        "image_width",
        "-w",
        "--width",
        "1000",
        " - width of rendered images, defaulted to 1000",
        testing::validate_is_positive_param
    );
    testing::add_test_param<int64_t>(
        "image_height",
        "-h",
        "--height",
        "1000",
        " - height of rendered images, defaulted to 1000",
        testing::validate_is_positive_param
    );
    return !testing::handle_args(argc, argv);
}