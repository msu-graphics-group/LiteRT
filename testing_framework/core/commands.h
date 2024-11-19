#pragma once
#include <testing_framework/core/test.h>
#include <vector>
#include <filesystem>

namespace testing
{

    bool list(
        const std::vector<const Test*>& tests,
        bool write_descriptions
    );
    
    bool run(
        size_t jobs,
        const std::vector<const Test*>& tests,
        bool ignore_saved_references,
        std::filesystem::path saves,
        std::filesystem::path references,
        size_t width,
        size_t height,
        size_t rendering_count
    );
    
    bool unsafe(
        const Test* test,
        bool ignore_saved_references,
        std::filesystem::path saves,
        std::filesystem::path references,
        size_t width,
        size_t height,
        size_t renderings_count
    );
    
    bool rewrite(
        const Test* test,
        std::filesystem::path saves,
        std::filesystem::path references,
        size_t width,
        size_t height,
        size_t renderings_count
    );

}