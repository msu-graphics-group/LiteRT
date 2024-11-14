#pragma once
#include <string>
#include <vector>

namespace test
{
    class Regression
    {
    public:
        Regression(const std::string&name, const std::string&description);
        virtual ~Regression() = default;
        virtual bool run() const = 0;
        const std::string& name() const;
        const std::string& description() const;

        static const std::vector<const Regression*>& all();

        bool execute() const;
        bool rewrite() const;
 
    private:
        std::string name_, description_;
        static std::vector<const Regression*> tests_;
    };
}

#define ADD_REGRESSION(name, short_name, description)                       \
class Regression_##name : public test::Regression {                         \
                                                                            \
    using test::Regression::Regression;                                     \
    bool run() const override;                                              \
};                                                                          \
Regression_##name regression_##name##_instance(#short_name, description);   \
bool Regression_##name ::run() const
