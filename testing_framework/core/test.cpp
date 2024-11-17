#include "test.h"
#include <iostream>
#include "supervisor.h"
#include <filesystem>

namespace testing
{

    Test::Test(std::string name, std::string description, std::string file, size_t line) :
        name_(name),
        description_(description),
        file_(file),
        line_(line)
    {
        tests().push_back(this);
    }

    std::string_view Test::name() const
    {
        return name_;
    }
    std::string_view Test::description() const
    {
        return description_;
    }
    std::string_view Test::file() const
    {
        return file_;
    }
    size_t Test::line() const
    {
        return line_;
    }

    const std::vector<const Test*>& Test::all()
    {
        return tests();
    }

    std::vector<const Test*>& Test::tests()
    {
        static std::vector<const Test*> tests_;
        return tests_;
    }

}