#include "color.h"

namespace testing
{

     const char * Color::foreground_escape_sequence() const
     {
        
        switch(hue)
        {
            case COLOR_HUE::RED:
                return "\33[31m";
            case COLOR_HUE::GREEN:
                return "\33[32m";
            default:
                return "";            
        }
     }
    const char *Color::background_escape_sequence() const
    {
        return "";
    }
    const char *Color::default_escape_sequence()
    {
        return "\33[0m";
    }
}