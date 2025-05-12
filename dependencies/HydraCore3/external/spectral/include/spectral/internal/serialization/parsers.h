#ifndef INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_PARSERS_H
#define INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_PARSERS_H
#include <string>
#include <sstream>
#include <array>

namespace spec {

    template<typename T>
    struct Parser;

    template<typename T>
    T parse(const std::string &str)
    {
        return Parser<T>::parse(str);
    }


    template<typename T>
    struct Parser
    {
        static T parse(const std::string &str);
    };

    template<typename T>
    T Parser<T>::parse(const std::string &str)
    {
        std::stringstream ss(str);
        T val;
        ss >> val;
        return val;
    }

    template<>
    struct Parser<int>
    {
        static int parse(const std::string &str);
    };

    template<>
    struct Parser<long>
    {
        static long parse(const std::string &str);
    };

    template<>
    struct Parser<long long>
    {
        static long long parse(const std::string &str);
    };

    template<>
    struct Parser<unsigned>
    {
        static unsigned parse(const std::string &str);
    };

    template<>
    struct Parser<unsigned long>
    {
        static unsigned long parse(const std::string &str);
    };

    template<>
    struct Parser<unsigned long long>
    {
        static unsigned long long parse(const std::string &str);
    };

    template<>
    struct Parser<float>
    {
        static float parse(const std::string &str);
    };

    template<>
    struct Parser<double>
    {
        static double parse(const std::string &str);
    };

    template<>
    struct Parser<long double>
    {
        static long double parse(const std::string &str);
    };

    template<>
    struct Parser<std::string>
    {
        static std::string parse(const std::string &str);
    };

}

#endif