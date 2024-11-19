#pragma once
#include <string>
#include <ostream>

namespace testing
{

    size_t logging_level();
    void set_logging_level(size_t);

    /*
        Returns output stream for given logging level
        Probably cout or empty ostream
    */
    std::ostream& log(size_t logging_level);

}