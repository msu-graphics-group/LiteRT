#pragma once
#include <string>

namespace testing
{

    enum class COLOR_HUE
    {
        WHITE,
        BLACK,
        RED,
        GREEN,
        BLUE,
        MAGENTA,
        CYAN,
        YELLOW
    };

    struct Color
    {
        COLOR_HUE hue;
        bool brightness;
        const char * foreground_escape_sequence() const;
        const char *background_escape_sequence() const;
        static const char *default_escape_sequence();
    };

}
