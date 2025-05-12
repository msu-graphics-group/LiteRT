#include <upsample/functional/smits.h>

namespace spec::upsample {

    namespace {

        constexpr Float WHITE_SPECTRUM[] = {
            1.0000f, 1.0000f, 0.9999f, 0.9993f, 0.9992f,
            0.9998f, 1.0000f, 1.0000f, 1.0000f, 1.0000f
        };

        constexpr Float CYAN_SPECTRUM[] = {
            0.9710f, 0.9426f, 1.0007f, 1.0007f, 1.0007f,
            1.0007f, 0.1564f, 0.0000f, 0.0000f, 0.0000f
        };

        constexpr Float MAGENTA_SPECTUM[] = {
            1.0000f, 1.0000f, 0.9685f, 0.2229f, 0.0000f,
            0.0458f, 0.8369f, 1.0000f, 1.0000f, 0.9959f
        };

        constexpr Float YELLOW_SPECTUM[] = {
            0.0001f, 0.0000f, 0.1088f, 0.6651f, 1.0000f,
            1.0000f, 0.9996f, 0.9586f, 0.9685f, 0.9840f
        };

        constexpr Float RED_SPECTRUM[] = {
            0.1012f, 0.0515f, 0.0000f, 0.0000f, 0.0000f, 
            0.0000f, 0.8325f, 1.0149f, 1.0149f, 1.0149f
        };

        constexpr Float GREEN_SPECTRUM[] = {
            0.0000f, 0.0000f, 0.0273f, 0.7937f, 1.0000f,
            0.9418f, 0.1719f, 0.0000f, 0.0000f, 0.0025f
        };

        constexpr Float BLUE_SPECTRUM[] = {
            1.0000f, 1.0000f, 0.8916f, 0.3323f, 0.0000f,
            0.0000f, 0.0003f, 0.0369f, 0.0483f, 0.0496f
        }; 

        void add_array_multiplied(BasicSpectrum &s, const Float mul, const Float *ptr)
        {
            for(unsigned i = 0; i < SMITS_SPECTRUM_SIZE; ++i) {
                int wl = SMITS_WAVELENGHTS[i];
                s.get_or_create(wl) += mul * (*(ptr++));
            }
        }

        void upsample_to(const vec3 &rgb, BasicSpectrum &s) 
        {
            if(rgb.x <= rgb.y && rgb.x <= rgb.z) {
                add_array_multiplied(s, rgb[0], WHITE_SPECTRUM);

                if(rgb.y <= rgb.z) {
                    add_array_multiplied(s, rgb[1] - rgb[0], CYAN_SPECTRUM);
                    add_array_multiplied(s, rgb[2] - rgb[1], BLUE_SPECTRUM);
                }
                else {
                    add_array_multiplied(s, rgb[2] - rgb[0], CYAN_SPECTRUM);
                    add_array_multiplied(s, rgb[1] - rgb[2], GREEN_SPECTRUM);
                }
            }
            else if(rgb.y <= rgb.x && rgb.y <= rgb.z) {
                add_array_multiplied(s, rgb[1], WHITE_SPECTRUM);

                if(rgb.x <= rgb.z) {
                    add_array_multiplied(s, rgb[0] - rgb[1], MAGENTA_SPECTUM);
                    add_array_multiplied(s, rgb[2] - rgb[0], BLUE_SPECTRUM);
                }
                else {
                    add_array_multiplied(s, rgb[2] - rgb[1], MAGENTA_SPECTUM);
                    add_array_multiplied(s, rgb[0] - rgb[2], RED_SPECTRUM);
                }
            }
            else /*if(rgb.z <= rgb.x && rgb.z <= rgb.y)*/ {
                add_array_multiplied(s, rgb[2], WHITE_SPECTRUM);

                if(rgb.x <= rgb.y) {
                    add_array_multiplied(s, rgb[0] - rgb[2], YELLOW_SPECTUM);
                    add_array_multiplied(s, rgb[1] - rgb[0], GREEN_SPECTRUM);
                }
                else {
                    add_array_multiplied(s, rgb[1] - rgb[2], YELLOW_SPECTUM);
                    add_array_multiplied(s, rgb[0] - rgb[1], RED_SPECTRUM);
                }
            }
        }

    }

    BasicSpectrum smits(const vec3 &rgb)
    {
        BasicSpectrum spectrum;
        upsample_to(rgb, spectrum);
        return spectrum;
    }

}