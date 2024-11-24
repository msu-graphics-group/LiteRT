#pragma once
#include <string>

namespace testing
{

    /*
        Should be called in main function
    */
    void add_options();

    /*
        Paths
    */
    std::string scenes_directory();
    std::string saves_directory();
    std::string saved_references_directory();

    /*
        If saved references is invalid (file is missing or size is different),
            test is not skipped, but check is just ignored
    */
    bool ignore_saved_references();

    /*
        Rendering
    */
    size_t renderings_count(); // how many renderings to do
    size_t image_width(); // width of image to render
    size_t image_height(); // height of image to render

}