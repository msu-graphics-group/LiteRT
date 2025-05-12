#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdint>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif



struct Pixel { unsigned char r, g, b; };

bool SaveImage4fToEXR(const float* rgb, int width, int height, const char* outfilename, float a_normConst, bool a_invertY);
bool SaveImage3DToEXR(const float* data, int width, int height, int channels, const char* outfilename);
void SaveImage3DToImage3D1f(const float* data, int width, int height, int channels, const char* outfilename);
void FlipYAndNormalizeImage2D1f(float* data, int width, int height, float a_normConst);
void SaveFrameBufferToEXR(float* data, int width, int height, int channels, const char* outfilename, float a_normConst = 1.0f);
inline float linearToSRGB(float l);
std::vector<uint32_t> FrameBufferColorToLDRImage(const float* rgb, int width, int height, float a_normConst, float a_gamma);
bool SaveImage4fToBMP(const float* rgb, int width, int height, int channels, const char* outfilename, float a_normConst = 1.0f, float a_gamma = 2.2f);
std::vector<float> LoadImage1fFromEXR(const char* infilename, int* pW, int* pH);
std::vector<float> LoadImage4fFromEXR(const char* infilename, int* pW, int* pH);
float* LoadImage4fFromEXRUnsafe(const char* infilename, int* pW, int* pH);

bool SaveImage4fByExtension(const float* data, int width, int height, int channels, const char* outfilename, float a_normConst  = 1.0f, float a_gamm = 2.2f) ;

