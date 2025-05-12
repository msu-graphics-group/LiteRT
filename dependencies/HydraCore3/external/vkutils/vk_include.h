#ifndef VK_UTILS_VULKAN_INCLUDE_H
#define VK_UTILS_VULKAN_INCLUDE_H

#if defined(USE_VOLK)
#include "volk.h"
#elif defined(USE_ETNA)
#include <etna/Vulkan.hpp>
#else
#include <vulkan/vulkan.h>
#endif

#endif // VK_UTILS_VULKAN_INCLUDE_H
