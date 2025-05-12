#ifndef INCLUDE_SPECTRAL_SPEC_FOURIER_LUT_H
#define INCLUDE_SPECTRAL_SPEC_FOURIER_LUT_H
#include <spectral/internal/math/math.h>
#include <spectral/internal/math/fourier.h>
#include <vector>
#include <cinttypes>
#include <istream>

namespace spec {
    
    class FourierLUT
    {
    public:
        static constexpr uint64_t FILE_MARKER = 0xfafa0000ab0ba001;

        FourierLUT(std::vector<Float> &&data, const std::vector<Float> &power_values, unsigned step, unsigned m) : step{step}, size{256 / step + 1 + (255 % step != 0)}, m{m},
            power_values(power_values), data{std::move(data)} {}

        FourierLUT(const std::vector<Float> &power_values, unsigned step, unsigned m) : step{step}, size{256 / step + 1 + (255 % step != 0)}, m{m},
            power_values(power_values), data(power_values.size() * size * size * size * (m + 1)) {}

        FourierLUT() = default;
        
        FourierLUT(const FourierLUT &) = delete;
        FourierLUT &operator=(const FourierLUT &) = delete;

        FourierLUT(FourierLUT &&other) : step{}, size{}, m{}, power_values{}, data{}
        {
            *this = std::move(other);
        }

        FourierLUT &operator=(FourierLUT &&other)
        {
            data = std::move(other.data);
            power_values = std::move(other.power_values);
            std::swap(m, other.m);
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

        unsigned get_m() const
        {
            return m;
        }

        const std::vector<Float> get_power_vals() const
        {
            return power_values;
        }

        const Float *get_raw_data() const
        {
            return data.data();
        }

        std::vector<Float> eval(int r, int g, int b, Float power) const;

        static FourierLUT load_from(std::istream &src);

    private:
        unsigned step;
        unsigned size;
        unsigned m;
        std::vector<Float> power_values;
        std::vector<Float> data;

        void add(std::vector<Float> &res, unsigned r, unsigned g, unsigned b, unsigned n, Float mul) const;
    };


}

#endif
