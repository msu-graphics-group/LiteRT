#include "unittest.h"

namespace test
{

    std::vector<const Unittest*> Unittest::tests_;

    Unittest::Unittest(const std::string&name, const std::string&description) : 
        name_(name),
        description_(description)
    {
        tests_.push_back(this);
    }
       
    const std::string& Unittest::name() const
    {
        return name_;
    }
    const std::string& Unittest::description() const
    {
        return description_;
    }

    const std::vector<const Unittest*>& Unittest::all()
    {
        return tests_;
    }

    bool Unittest::execute() const
    {
        return run();
    }

}