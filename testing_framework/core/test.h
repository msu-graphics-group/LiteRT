#pragma once
#include <string>
#include <vector>

namespace testing
{

    class Test
    {
    public:
        
        Test(std::string name, std::string description, std::string file, size_t line);

        const std::string& name() const;
        const std::string& description() const;
        const std::string& file() const;
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
