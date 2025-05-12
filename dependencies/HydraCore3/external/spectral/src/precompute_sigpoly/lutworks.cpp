#include "lutworks.h"
#include <internal/serialization/binary.h>
#include <internal/common/util.h>
#include <vector>

namespace bin = spec::binary;

void write_header(std::ostream &dst)
{
    bin::write<uint64_t>(dst, spec::SigpolyLUT::FILE_MARKER);
    bin::write<uint16_t>(dst, sizeof(Float));
}

void write_lut(std::ostream &dst, const SigpolyLUT &lut)
{  
    unsigned step = lut.get_step();
    bin::write<uint16_t>(dst, step);
    unsigned count = lut.get_size();
    count = count * count * count;
    const vec3 *ptr = lut.get_raw_data();
    for(unsigned i = 0; i < count ; ++i) {
        bin::write_vec<Float>(dst, ptr[i]);
    }
}



namespace {

    inline vec3 fill_vector(int main_channel, Float a, Float b, Float alpha)
    {
        switch(main_channel) {
        case 0:
            return {alpha, a, b};
        case 1:
            return {b, alpha, a};
        case 2:
            return {a, b, alpha};
        default:
            return {};
        }
    }

    class LutBuilder
    {
    public:
        const int main_channel;
        const int step, size;
        const int stable, stable_id;
        const bool force_last;
        int i = 0, j = 0, k = 0;

        LutBuilder(int main_channel, int step, int stable)
            : main_channel{main_channel}, step{step}, size{256 / step + (255 % step != 0)},
              stable{stable}, stable_id{stable / step}, 
              force_last{255 % step != 0}, data(size * size * size) {}

        SigpolyLUT build_and_clear()
        {
            data.shrink_to_fit();
            return SigpolyLUT(std::move(data), step);
        }

        int idx_to_color(int i) const
        {
            return i == size - 1 ? 255 : i * step;
        }

        void set_target(int i, int j, int k);

        int color_to_idx(int c) const
        {
            return c == 255 ? size - 1 : c / step; 
        }

        Float acolorf(int i) const
        {
            return spec::math::smoothstep2(i / static_cast<Float>(size - 1));
        }

        void init_solution(vec3d &solution) const
        {
            if(k == stable_id) {
                std::fill_n(solution.v, 3, 0.0);
            }
            else {
                solution = at(i, j, k < stable_id ? k + 1 : k - 1);
            }
        }

        const vec3 &at(int i1, int j1, int k1) const
        {
            return data[((k1 * size) + i1) * size + j1]; 
        }

        const vec3 &current() const
        {
            return data[((k * size) + i) * size + j]; 
        }

        vec3 &current()
        {
            return data[((k * size) + i) * size + j]; 
        }

        vec3 spaced_color() const;
    private:
        std::vector<vec3> data;
    };

    vec3 LutBuilder::spaced_color() const
    {
        Float ac = acolorf(k);
        return fill_vector(main_channel, ac * idx_to_color(i) / 255.0f, ac * idx_to_color(j) / 255.0f, ac);
    }


    unsigned color_processed = 0;

    void fill(LutBuilder &ctx)
    {
        vec3d solution;
        for(ctx.i = 0; ctx.i < ctx.size; ++ctx.i) {
            for(ctx.j = 0; ctx.j < ctx.size; ++ctx.j) {
                ctx.init_solution(solution);

                solve_for_rgb_d(ctx.spaced_color(), solution);
                ctx.current() = solution;
                color_processed += 1;
            }
            spec::print_progress(color_processed);
        }

    }

}

#include <iostream>

SigpolyLUT generate_lut(int zeroed_idx, int step, int stable_val)
{
    LutBuilder ctx{zeroed_idx, step, stable_val};

    color_processed = 0u;

    spec::init_progress_bar(ctx.size * ctx.size * ctx.size, 1000);

    for(ctx.k = ctx.stable_id; ctx.k >= 0; --ctx.k) {
        fill(ctx);
    }
    for(ctx.k = ctx.stable_id + 1; ctx.k < ctx.size; ++ctx.k) {
        fill(ctx);
    }

    spec::finish_progress_bar();
    return ctx.build_and_clear();
}