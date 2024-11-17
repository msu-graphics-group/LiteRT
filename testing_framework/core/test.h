#pragma once
#include <string>
#include <vector>

namespace testing
{

    class Test
    {
    public:
        
        Test(std::string name, std::string description, std::string file, size_t line);

        std::string_view name() const;
        std::string_view description() const;
        std::string_view file() const;
        size_t line() const;

        /*
            Method with code of test
        */
        virtual void run() const = 0;

        static const std::vector<const Test*>& all();

    private:

        static std::vector<const Test*>& tests();

        std::string name_, description_, file_;
        size_t line_;
        
    };

}
