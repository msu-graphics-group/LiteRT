#include <internal/common/util.h>
#include <iostream>
#include <algorithm>

namespace spec {
#ifndef SPECTRAL_DISABLE_PROGRESS_BAR

    unsigned _progress_max_count = 1;
    unsigned _progress_min_update_step = 1;
    unsigned _prev_val = 0;

#endif

    void init_progress_bar(unsigned maxcount, unsigned minupdate) {
#ifndef SPECTRAL_DISABLE_PROGRESS_BAR

        _progress_max_count = maxcount;
        _progress_min_update_step = minupdate;
        _prev_val = 0;
        std::cout << std::endl;

#else
        (void) maxcount; (void) minupdate;
#endif
    }

    void finish_progress_bar() {
#ifndef SPECTRAL_DISABLE_PROGRESS_BAR
        print_progress(_progress_max_count, true);
#endif
    }

    void print_progress(unsigned count, bool force) 
    {
#ifndef SPECTRAL_DISABLE_PROGRESS_BAR

        static const unsigned bar_length = 100;
        if(force || count - _prev_val > _progress_min_update_step) {
            _prev_val = (count / _progress_min_update_step) * _progress_min_update_step;
            unsigned progress_points = bar_length * (count / static_cast<float>(_progress_max_count));
            std::cout << "\033[F[";
            std::cout << std::string(progress_points, '*') + std::string(bar_length - progress_points, ' ');
            std::cout << "] " << count << "/" << _progress_max_count << " - " << progress_points << "%\n";
        }

#else
        (void) count; (void) force;
#endif
    }

    bool is_little_endian()
    {
        int i = 1;
        return *(reinterpret_cast<char *>(&i));
    }

    void serial_copy(const char *src, char *dst, unsigned size)
    {
        std::copy(src, src + size, dst);
        if(is_little_endian()) {
            std::reverse(dst, dst + size);
        } 
    }

    void convert_to_native_order(const char *src, char *dst, unsigned size, bool from_big_endian)
    { 
        std::copy(src, src + size, dst);
        if(is_little_endian() ^ !from_big_endian) {
            std::reverse(dst, dst + size);
        } 
    }

    void convert_from_native_order(const char *src, char *dst, unsigned size, bool to_big_endian)
    {
        convert_to_native_order(src, dst, size, to_big_endian);
    }

}