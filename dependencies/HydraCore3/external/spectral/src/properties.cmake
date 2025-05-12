add_subdirectory(converter)
add_subdirectory(comparsion)
add_subdirectory(experimental) 
add_subdirectory(exporter)

if(NOT SPECTRAL_NO_PRECOMPUTERS)
    add_subdirectory(precompute_sigpoly)
    add_subdirectory(precompute_fourier)
endif()