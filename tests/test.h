#pragma once
#include <string>
#include <vector>

namespace test
{
    class Unittest
    {
    public:
        Unittest(const std::string&name, const std::string&description) : 
            name_(name),
            description_(description)
        {
            tests_.push_back(this);
        }
        virtual ~Unittest() = default;
        virtual bool run() const = 0;
        const std::string& name() const {
            return name_;
        }
        const std::string& description() const {
            return description_;
        }

        static const std::vector<Unittest*>& list()
        {
            return tests_;
        }

    private:
        std::string name_, description_;
        static std::vector<Unittest*> tests_;
    };
}

#define ADD_UNITTEST(name, short_name, description)                     \
class Unittest_##name : public test::Unittest {                        \
                                                                        \
    using test::Unittest::Unittest;                                     \
    bool run() const override;                                          \
};                                                                      \
Unittest_##name unittest_##name##_instance(#short_name, description);   \
bool Unittest_##name ::run() const
