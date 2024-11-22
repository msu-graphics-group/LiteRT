#include <testing_framework/core/colors.h>
#include <string>

namespace testing
{

    constexpr bool COLORS_ENABLED_DEFAULT = true;

    static bool enabled = COLORS_ENABLED_DEFAULT;

    bool get_colors_enabled_default()
    {
        return COLORS_ENABLED_DEFAULT;
    }

    static void enable_colors()
    {
        
    }

    void set_colors_enabled(bool e)
    {
        enabled = e;
        if (e)
        {
            enable_colors();
        }
    }

    bool colors_are_enabled()
    {
        return enabled;
    }

    static size_t color_number(Color color, bool is_background)
    {
        auto base = [](bool bright, bool bg)->int{
            if (!bg && !bright)
            {
                return 30;
            }
            else if(!bg && bright)
            {
                return 90;
            }
            else if(bg && !bright)
            {
                return 40;
            }
            else
            {
                return 100;
            }
        };
        int red = color.red;
        int green = color .green;
        int blue = color.blue;
        int n = (blue << 2) | (green << 1) | red;
        return base(color.bright, is_background) + n; 
    }

    static std::string color_escape_sequence(std::optional<Color> foreground, std::optional<Color> background)
    {
        if (!colors_are_enabled())
        {
            return "";
        }
        if (!foreground && !background)
        {
            return "";
        }
        std::string out = "\33[";
        if (foreground)
        {
            out += std::to_string(color_number(*foreground, false));
        }
        if (foreground && background)
        {
            out += ";";
        }
        if (background)
        {
            out += std::to_string(color_number(*background, true));
        }
        return out + "m";
    }

    static std::string default_color_escape_sequence()
    {
        if (!colors_are_enabled())
        {
            return "";
        }
        return "\33[0m";
    }

    void set_output_color(std::ostream&out, std::optional<Color> foreground, std::optional<Color> background)
    {
        out << color_escape_sequence(foreground, background);
    }
    void set_output_color(std::FILE*out, std::optional<Color> foreground, std::optional<Color> background)
    {
        std::fputs(color_escape_sequence(foreground, background).c_str(), out);
    }

    void set_default_color(std::ostream&out)
    {
        out << default_color_escape_sequence();
    }
    void set_default_color(std::FILE*out)
    {
        std::fputs(default_color_escape_sequence().c_str(), out);
    }

}