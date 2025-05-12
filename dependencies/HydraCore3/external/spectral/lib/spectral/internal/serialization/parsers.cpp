#include <internal/serialization/parsers.h>

namespace spec {

    int Parser<int>::parse(const std::string &str)
    {
        return stoi(str);
    }

    long Parser<long>::parse(const std::string &str)
    {
        return stol(str);
    }

    long long Parser<long long>::parse(const std::string &str)
    {
        return stoll(str);
    }

    unsigned Parser<unsigned>::parse(const std::string &str)
    {
        return stoul(str);
    }

    unsigned long Parser<unsigned long>::parse(const std::string &str)
    {
        return stoul(str);
    }

    unsigned long long Parser<unsigned long long>::parse(const std::string &str)
    {
        return stoull(str);
    }

    float Parser<float>::parse(const std::string &str)
    {
        return stof(str);
    }

    double Parser<double>::parse(const std::string &str)
    {
        return stod(str);
    }

    long double Parser<long double>::parse(const std::string &str)
    {
        return stold(str);
    }

    std::string Parser<std::string>::parse(const std::string &str)
    {
        return str;
    }

}