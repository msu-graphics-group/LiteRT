#pragma once
#include <iostream>

namespace testing
{

    size_t get_message_max_width();
    size_t get_message_max_length();

   struct comment_begin_t{};
    
    /*
        Returns output stream for given logging level
    */
    std::ostream& out(size_t logging_level);

    

}