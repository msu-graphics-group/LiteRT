

add_executable(
    test
    
    help.h
    help.cpp
    args.h
    args.cpp

    unittest.h
    unittest.cpp

    regression.h
    regression.cpp

    test.h
    test.cpp
)

target_link_libraries(test PRIVATE render_lib)