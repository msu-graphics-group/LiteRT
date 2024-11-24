#pragma once
#include <testing_framework/helpers/render.h>

namespace testing
{

    float measure_psnr(Image&ref, Image&other);

    void check_psnr(
        Image&ref,
        Image&other,
        std::string_view ref_desc,
        std::string_view other_desc,
        float threshold
    );

    /*
        Saved reference if stored in file
        <saved-references>/<test-name>/<name>.png
    */
    void saved_reference_check_psnr(
        Image&image,
        const std::string&desc,
        const std::string&name,
        float threshold
    );

}