#pragma once
#include <filesystem>

namespace testing
{

    struct SkipTestException{};

    extern size_t passed_checks_, failed_checks_;

    extern bool rewrite_mode_;

    extern bool ignore_saved_references_;

    extern std::filesystem::path saves_, refs_;

    extern size_t width_, height_;

    extern size_t renderings_count_;

}