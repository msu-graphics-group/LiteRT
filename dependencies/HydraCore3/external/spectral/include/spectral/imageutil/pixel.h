#ifndef INCLUDE_SPECTRAL_IMAGEUTIL_PIXEL_H
#define INCLUDE_SPECTRAL_IMAGEUTIL_PIXEL_H
#include <spectral/internal/math/vec3.h>
#include <type_traits>
#include <cinttypes>

namespace spec {

	#pragma pack(push, 1)
	union PixelRGB {
		uint8_t rgb[3];
		struct {
			uint8_t r, g, b;
		};
		//required for triviality
		PixelRGB() = default; // @suppress("Class members should be properly initialized")
		PixelRGB(const PixelRGB &) = default;
		PixelRGB(uint8_t r, uint8_t g, uint8_t b);
		PixelRGB &operator=(const PixelRGB&) = default;
		
		PixelRGB(uint32_t rgb) 
		{
			*this = rgb;
		}

		PixelRGB &operator=(uint32_t rgb);

		operator uint32_t() const
		{
			return as_rgb();
		}

		uint8_t operator[](int i) const
		{
			return rgb[i];
		}
		uint8_t &operator[](int i)
		{
			return rgb[i];
		}

		math::vec3 to_vec3() const;
		
		uint32_t as_rgb() const;

		static PixelRGB from_rgb(uint32_t rgb);
		static PixelRGB from_vec3(const math::vec3 &rgb);
		static const PixelRGB none;
	};
	#pragma pack(pop)

	using Pixel = PixelRGB;

	static_assert(std::is_trivial<Pixel>() && std::is_standard_layout<Pixel>(), "Requires being trivial standard layout object for outputting");

}

#endif
