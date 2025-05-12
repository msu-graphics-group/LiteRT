#include <imageutil/pixel.h>

namespace spec {

	PixelRGB::PixelRGB(uint8_t r, uint8_t g, uint8_t b)
	{
		rgb[0] = r;
		rgb[1] = g;
		rgb[2] = b;
	}

	PixelRGB &PixelRGB::operator=(uint32_t rgb_in)
	{
		rgb[0] = (rgb_in >> 24);
		rgb[1] = (rgb_in >> 16) & 0xFF;
		rgb[2] = (rgb_in >> 8) & 0xFF;
		return *this;
	}

	uint32_t PixelRGB::as_rgb() const
	{
		return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
	}

	math::vec3 PixelRGB::to_vec3() const
	{
		return {
			rgb[0] / 255.0f,
			rgb[1] / 255.0f,
			rgb[2] / 255.0f
		};
	}

	PixelRGB PixelRGB::from_rgb(uint32_t rgb)
	{
		return PixelRGB(
				(rgb >> 16) & 0xFF,
				(rgb >> 8) & 0xFF,
				(rgb) & 0xFF
				);
	}

	PixelRGB PixelRGB::from_vec3(const math::vec3 &rgb)
	{
		return PixelRGB(
				static_cast<uint8_t>(rgb.x * 255.999),
				static_cast<uint8_t>(rgb.y * 255.999),
				static_cast<uint8_t>(rgb.z * 255.999)
				);
	}

	const PixelRGB PixelRGB::none{0, 0, 0};

}