#ifndef INCLUDE_SPECTRAL_SPEC_SIGPOLY_LUT_H
#define INCLUDE_SPECTRAL_SPEC_SIGPOLY_LUT_H
#include <spectral/internal/math/math.h>
#include <vector>
#include <cinttypes>
#include <istream>

namespace spec {
    
    class SigpolyLUT
    {
    public:
        static constexpr uint64_t FILE_MARKER = 0xfafa0000ab0ba000;

        SigpolyLUT(std::vector<vec3> &&data, unsigned step) : step{step}, size{256 / step + (255 % step != 0)}, data{std::move(data)} {}

        SigpolyLUT(unsigned step) : step{step}, size{256 / step + (255 % step != 0)}, data(size * size * size) {}

        SigpolyLUT(const SigpolyLUT &) = default;
        SigpolyLUT &operator=(const SigpolyLUT &) = delete;

        SigpolyLUT(SigpolyLUT &&other) : step{}, size{}, data{}
        {
            *this = std::move(other);
        }

        SigpolyLUT &operator=(SigpolyLUT &&other)
        {
            data = std::move(other.data);
            std::swap(step, other.step);
            std::swap(size, other.size);
            return *this;
        }

        unsigned get_size() const
        {
            return size;
        }

        unsigned get_step() const
        {
            return step;
        }

        const vec3 *get_raw_data() const
        {
            return data.data();
        }

        vec3 eval(int a, int b, int alpha) const;

        static SigpolyLUT load_from(std::istream &src);

    private:
        unsigned step;
        unsigned size;
        std::vector<vec3> data;

        const vec3 &at(int i, int j, int k) const
        {
            return data[((k * size) + i) * size + j];
        }
        Float aid_to_alpha(int c) const;
    };


}

#endif