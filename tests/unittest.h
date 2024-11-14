#pragma once
#include <string>
#include <vector>

namespace test
{
    class Unittest
    {
    public:
        Unittest(const std::string&name, const std::string&description);
        virtual ~Unittest() = default;
        virtual bool run() const = 0;
        const std::string& name() const;
        const std::string& description() const;

        static const std::vector<const Unittest*>& all();

        bool execute() const;
 
    private:
        std::string name_, description_;
        static std::vector<const Unittest*> tests_;
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
