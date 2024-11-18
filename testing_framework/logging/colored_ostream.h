#pragma once
#include <ostream>
#include <testing_framework/logging/color.h>
#include <type_traits>

namespace testing
{

    template<typename Char, typename Traits>
    class basic_colored_ostream
    {
    public:
        static_assert(std::is_same_v<Char, char>);
        using ostream_type = std::basic_ostream<Char, Traits>;
        basic_colored_ostream(ostream_type&out, Color color, Color bg_color) :
            out_(&out)
        {
            out << color.foreground_escape_sequence() << bg_color.background_escape_sequence();
        }
        template<typename T>
        basic_colored_ostream& operator<<(const T&v)
        {
            (*out_) << v;
            return *this;
        }
        ~basic_colored_ostream()
        {
            (*out_) << Color::default_escape_sequence();
        }
    private:
        ostream_type* out_;
    };

}
