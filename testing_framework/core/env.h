#pragma once
#include <string>
#include <filesystem>

namespace testing
{

    // skips current test
    void skip();

    // adds score to current test
    void add_check_result(bool passed);

    /*
        If in rewrite mode
            Asks if user wants <comment>
        
        Always returns false in non-rewrite mode
    */
    bool should_rewrite_saved_reference(std::string_view comment);
    
    /*
        Returns true if invalid saved references should be ignored or test must be skipped
        For example, reference image can have different size, image file can be invalid or does not exist
    */
    bool ignore_saved_references();

    /*
        Path to directory where test saves produces files
    */
    std::filesystem::path saves_directory();

    /*
        Path to directory where test stores it's saved references
    */
    std::filesystem::path saved_references_directory();

    /*
        Default size for rendered images
    */
    size_t image_width();
    size_t image_height();

    /*
        Amount of times to execute rendering to calculate mean rendering time
    */
    size_t renderings_count();

}