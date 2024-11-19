#pragma once
#include <ostream>
#include <optional>

namespace testing
{

    void set_colors_enabled(bool enabled);
    bool colors_are_enabled();

    struct Color
    {
        bool red, green, blue, bright;
    };

    void set_output_color(std::ostream&out, std::optional<Color> foreground, std::optional<Color> background);
    void set_output_color(std::FILE*out, std::optional<Color> foreground, std::optional<Color> background);

    void set_default_color(std::ostream&out);
    void set_default_color(std::FILE*out);

    struct ForegroundApplier
    {
        Color color;
    };

    struct BackgroundApplier
    {
        Color color;
    };

    struct DefaultColor{};
    constexpr DefaultColor default_color;

    inline ForegroundApplier foreground(Color color)
    {
        return {color};
    }

    inline BackgroundApplier background(Color color)
    {
        return {color};
    }

    inline std::ostream& operator<<(std::ostream&out, ForegroundApplier a)
    {
        set_output_color(out, a.color, std::nullopt);
        return out;
    }

    inline std::ostream& operator<<(std::ostream&out, BackgroundApplier a)
    {
        set_output_color(out, std::nullopt, a.color);
        return out;
    }

    inline std::ostream& operator<<(std::ostream&out, DefaultColor)
    {
        set_default_color(out);
        return out;
    }

#define DECL_COLOR(name, r, g, b, br) \
    constexpr Color name { r, g, b, br };

#define DECL_COLOR_2(name, r, g, b) \
    DECL_COLOR(name, r, g, b, 0) \
    DECL_COLOR(bright_##name, r, g, b, 1)

    DECL_COLOR_2(red, 1, 0, 0)
    DECL_COLOR_2(green, 0, 1, 0)
    DECL_COLOR_2(blue, 0, 0, 1)
    DECL_COLOR_2(yellow, 1, 1, 0)
    DECL_COLOR_2(majenta, 1, 0, 1)
    DECL_COLOR_2(cyan, 0, 1, 1)

    DECL_COLOR(black, 0, 0, 0, 0)
    DECL_COLOR(gray, 0, 0, 0, 1)
    DECL_COLOR(bright_gray, 1, 1, 1, 0)
    DECL_COLOR(white, 1, 1, 1, 1)

#undef DECL_COLOR_2
#undef DECL_COLOR

}