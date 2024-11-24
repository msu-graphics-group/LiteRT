#include <testing_framework/helpers/options.h>
#include <testing_framework/core/environment.h>
#include <testing_framework/core/test_options.h>
#include <testing_framework/core/param_validators.h>

namespace testing
{

    std::string scenes_directory()
    {
        return get_param<std::string>("scenes");
    }
    std::string saves_directory()
    {
        return get_param<std::string>("saves");
    }
    std::string saved_references_directory()
    {
        return get_param<std::string>("saved_references");
    }

    bool ignore_saved_references()
    {
        return get_flag("ignore_saved_references");
    }

   
    size_t renderings_count()
    {
        return get_param<int64_t>("renderings_count");
    }
    size_t image_width()
    {
        return get_param<int64_t>("image_width");
    }
    size_t image_height()
    {
        return get_param<int64_t>("image_height");
    }

    void add_options()
    {

        /*
            Paths
        */
       testing::add_test_param<std::string>(
            "scenes",
            "",
            "--scenes",
            "./scenes",
            " - path to directory, where test scenes are stored, defaulted to './scenes'",
            validate_is_not_empty_param
       );
       testing::add_test_param<std::string>(
            "saves",
            "",
            "--saves",
            "./saves",
            " - path to directory, where rendered images are stored, defaulted to './saves'",
            validate_is_not_empty_param
       );
       testing::add_test_param<std::string>(
            "saved_references",
            "",
            "--saved-references",
            "./refs",
            " - path to directory, where saved references are stored, defaulted to './refs'",
            validate_is_not_empty_param
       );

        /*
            Saved references
        */
        testing::add_test_flag(
            "ignore_saved_references",
            "",
            "--ignore-saved-references",
            " - specifies, if invalid saved references should be reason for test skipping"
        );
        
        /*
            Rendering
        */
        testing::add_test_param<int64_t>(
            "renderings_count",
            "",
            "--renderings",
            "1",
            " - now many renderings should be performed to measure mean rendering time, defaulted to 1",
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
    }

}