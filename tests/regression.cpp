#include "regression.h"

namespace test
{

    std::vector<const Regression*> Regression::tests_;

    Regression::Regression(const std::string&name, const std::string&description) : 
        name_(name),
        description_(description)
    {
        tests_.push_back(this);
    }
       
    const std::string& Regression::name() const
    {
        return name_;
    }
    const std::string& Regression::description() const
    {
        return description_;
    }

    const std::vector<const Regression*>& Regression::all()
    {
        return tests_;
    }

    bool Regression::execute() const
    {
        return run();
    }
    bool Regression::rewrite() const
    {
        return false;
    }

}