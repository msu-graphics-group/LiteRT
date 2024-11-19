#pragma once
#include <vector>
#include <string>
#include <filesystem>

namespace testing
{

    /*
        Returns args which should be put in cmdline to 
        run unsafe command with specified params
    */
    std::vector<std::string> cmdline_to_unsafe(
        bool enable_colors,
        size_t loggin_level,
        std::string test_name,

        bool ignore_saved_references,

        std::filesystem::path saves,
        std::filesystem::path references,

        size_t width,
        size_t height,
        size_t renderings_count
    );

    bool handle_args(size_t argc, char**argv);

}
