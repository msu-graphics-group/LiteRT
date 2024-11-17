#pragma once
#include "test.h"

#define __ADD_TEST(id, name, description, file, line)   \
    class id : public ::testing::Test {                 \
        using ::testing::Test::Test;                    \
        void run() const override;                      \
    };                                                  \
    id id##_instance(name, description, file, line);    \
    void id::run() const

#define ADD_TEST(name, description) \
    __ADD_TEST(Test_##name, #name, description, __FILE__, __LINE__)
