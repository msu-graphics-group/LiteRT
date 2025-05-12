## Vulkan helpers collection

One of memory allocation helpers uses [VMA](https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator) which
is included in this repo for convenience (see 'external' directory).

If you are loading Vulkan via [volk](https://github.com/zeux/volk) in your project, you will need to define _USE_VOLK_
symbol. For Cmake-based projects you can do this by adding the following line to CMakeLists.txt:
```cmake
add_compile_definitions(USE_VOLK)
```

Main repo URL: https://gitlab.com/vsan/vkutils

Mirror: https://github.com/msu-graphics-group/vk-utils