#pragma once
#include <string>
#include <ostream>
#include <sstream>
#include <testing_framework/core/colors.h>
#include <optional>

namespace testing
{

    constexpr size_t BAR_WIDTH = 12;

    size_t logging_level();
    void set_logging_level(size_t);

    /*
        Returns output stream for given logging level
        Probably cout or empty ostream
    */
    std::ostream& log(size_t logging_level);

    void set_message_width(size_t width);


    struct EndAligned {};
    constexpr EndAligned end_aligned;

    void aligned_output(std::ostream&out,
        std::string_view str,
        size_t width,
        int align,
        size_t padding,
        char fill
    );

    class Aligned
    {
    public:

        Aligned(std::ostream&out, size_t width, int align, size_t padding, char fill) :
            out_(&out),
            width_(width),
            align_(align),
            padding_(padding),
            fill_(fill)
        {}

        std::ostream& operator<<(const EndAligned&)
        {
            aligned_output(*out_, data_.str(), width_, align_, padding_, fill_);
            return *out_;
        }

        template<typename T>
        Aligned& operator<<(const T&value)
        {
            data_ << value;
            return *this;
        }

        Aligned& operator<<(std::ostream&(*f)(std::ostream&))
        {
            data_ << f;
            return *this;
        }

    private:
        std::ostream*out_;
        size_t width_;
        int align_;
        size_t padding_;
        char fill_;
        std::stringstream data_;
    };

    struct BeginAligned
    {
        size_t width;
        int align;
        size_t padding;
        char fill;
    };

    inline BeginAligned begin_aligned(size_t width, int align, size_t padding, char fill = ' ')
    {
        return BeginAligned{width, align, padding, fill};
    }

    inline Aligned operator<<(std::ostream&out, const BeginAligned&a)
    {
        return Aligned{out, a.width, a.align, a.padding, a.fill};
    }

    struct Bar
    {
        const char* text;
        size_t logging_level;
        int align;
        std::optional<Color> color;
        char fill = ' ';
    };

    inline std::ostream& operator<<(std::ostream&out, const Bar&bar)
    {
        if (bar.color)
        {
            out << foreground(*bar.color);
        }
        return  out << "[" << begin_aligned(BAR_WIDTH, bar.align, 1, bar.fill) << bar.text << end_aligned << "]" << default_color;
    }

    /*
        Returns logger of bar's logging_level and prints bar into it
    */
    inline std::ostream& log(const Bar&bar)
    {
        std::ostream&out =  log(bar.logging_level);
        return out << bar << " ";
    }

    /*
        Colors in messages
    */
    constexpr auto info_color = intense_gray;
    constexpr auto warning_color = intense_bright_yellow;
    constexpr auto error_color = intense_red;

    constexpr auto success_color = intense_bright_green;
    constexpr auto failure_color = intense_red;
    constexpr auto skip_color = intense_bright_yellow;

    constexpr auto highlight_color_1 = intense_bright_cyan;
    constexpr auto highlight_color_2 = intense_bright_majenta;

    constexpr auto command_color = intense_bright_yellow;
    constexpr auto option_color = intense_bright_cyan;

    /*
        Default logging levels
    */
    constexpr size_t TEST_RESULT_LOGGING_LEVEL = 0;
    constexpr size_t WARNING_LOGGING_LEVEL = 2;
    constexpr size_t ERROR_LOGGING_LEVEL  =1;
    constexpr size_t INFO_LOGGING_LEVEL = 3;
    
    /*
        Lines
    */
   constexpr Bar bar_line{"", TEST_RESULT_LOGGING_LEVEL, 0, success_color, '-'};
   constexpr Bar bar_bold_line{"", TEST_RESULT_LOGGING_LEVEL, 0, success_color, '='};

    /*
        Run
    */
    constexpr Bar bar_run{"RUN", TEST_RESULT_LOGGING_LEVEL, -1, success_color};

    /*
        Test results
    */
    constexpr Bar bar_passed{"PASSED", TEST_RESULT_LOGGING_LEVEL, 1, success_color};
    constexpr Bar bar_failed{"FAILED", TEST_RESULT_LOGGING_LEVEL, 1, failure_color};
    constexpr Bar bar_skipped{"SKIPPED", TEST_RESULT_LOGGING_LEVEL, 1, skip_color};
    constexpr Bar bar_crashed{"CRASHED", TEST_RESULT_LOGGING_LEVEL, 1, failure_color};

    /*
        Messages
    */
    constexpr Bar bar_info{"INFO", INFO_LOGGING_LEVEL, 0, info_color};
    constexpr Bar bar_warning{"WARNING", WARNING_LOGGING_LEVEL, 0, warning_color};
    constexpr Bar bar_error{"ERROR", ERROR_LOGGING_LEVEL, 0, error_color};

}