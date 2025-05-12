#ifndef LUTWORKS_H
#define LUTWORKS_H
#include "functions.h"
#include <spec/fourier_lut.h>
#include <istream>
#include <ostream>
#include <cinttypes>

using namespace spec;

void write_header(std::ostream &dst);
void write_lut(std::ostream &dst, const FourierLUT &lut);

FourierLUT generate_lut(const std::vector<Float> &wavelenghts, const std::vector<std::vector<Float>> &seeds, std::vector<vec3i> rgbs, unsigned step, unsigned knearest);

#endif