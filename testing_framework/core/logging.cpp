#include <testing_framework/core/logging.h>
#include <iostream>

namespace testing
{

    static size_t level = 0;
    static size_t message_width = 50;

    size_t logging_level()
    {
        return level;
    }

    void set_logging_level(size_t l)
    {
        level = l;
    }

    std::ostream& log(size_t level)
    {
        static std::ostream null_output(nullptr);
        if (level <= logging_level())
        {
            return std::cout;
        }
        else
        {
            return null_output;
        }
    }

    void aligned_output(std::ostream&out,
        std::string_view str,
        size_t width,
        int align,
        size_t padding,
        char fill
    )
    {

        size_t left_padding = 0;
        size_t right_padding = 0;

        if (str.length() < width)
        {
            size_t space = width - str.length();
            if (align == 0)
            {
                left_padding = space / 2;
                right_padding = space - left_padding;
            }
            else
            {
                size_t best_padding = space / 2;
                if (space % 2 == 0)
                {
                    best_padding--;
                }
                left_padding = std::min(best_padding, padding);
                right_padding = space - left_padding;
                if (align > 0)
                {
                    std::swap(left_padding, right_padding);
                }
            }
        }

        for (size_t i = 0; i < left_padding; i++)
        {
            out << fill;
        }

        out << str;

        for (size_t i = 0; i < right_padding; i++)
        {
            out << fill;
        }
        
    }

}